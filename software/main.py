"""
시각장애인을 위한 실시간 시각-촉각 대체 보조장치
Jetson Orin Developer Kit 구동용 메인 파이프라인

파이프라인:
  웹캠 → Depth Anything V2 (TensorRT) → 4×6 그리드(가로6×세로4) → 시리얼 → Pico → PCA9685×2 → SG90×24

실행 예시:
  python main.py --debug                         # 시각화 창 포함 실행
  python main.py --no-hw                         # 하드웨어 없이 시각화만
  python main.py --port /dev/ttyACM1 --debug     # 시리얼 포트 지정
"""

import argparse
import logging
import time

import cv2
import numpy as np
import serial
import torch
import tensorrt as trt

# ──────────────────────────────────────────────────────────────────
# 로깅 설정
# ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)


# ──────────────────────────────────────────────────────────────────
# Depth Anything V2 – TensorRT 추론 클래스
# ──────────────────────────────────────────────────────────────────
class DepthEstimatorTRT:
    """
    Depth Anything V2 ViT-S TensorRT 엔진 래퍼.
    입력: BGR 518×518 ndarray
    출력: uint8 깊이맵 (0~255, 값이 클수록 가까움 – disparity 기반)
    """

    # ImageNet 정규화 상수
    MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(3, 1, 1)
    STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(3, 1, 1)

    def __init__(self, engine_path: str, input_shape: tuple = (518, 518)):
        self.input_shape = input_shape

        trt_logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(trt_logger, "")

        with open(engine_path, "rb") as f, trt.Runtime(trt_logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()
        self.stream  = torch.cuda.Stream()

        # 추론 버퍼 사전 할당 (매 프레임 메모리 재할당 방지)
        self.d_input  = torch.empty((1, 3, *input_shape), dtype=torch.float32, device="cuda")
        self.d_output = torch.empty((1, *input_shape),    dtype=torch.float32, device="cuda")
        self.context.set_tensor_address("input",  int(self.d_input.data_ptr()))
        self.context.set_tensor_address("output", int(self.d_output.data_ptr()))

        logger.info("DepthEstimator TRT 로드 완료: %s", engine_path)

    def infer(self, frame_518: np.ndarray) -> np.ndarray:
        """BGR 518×518 → 정규화 깊이맵 (uint8)"""
        # BGR → RGB → float32 정규화
        img = cv2.cvtColor(frame_518, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = (img.transpose(2, 0, 1) - self.MEAN) / self.STD  # (3, H, W)

        with torch.cuda.stream(self.stream):
            self.d_input.copy_(
                torch.from_numpy(np.ascontiguousarray(img[None])), non_blocking=True
            )
            self.context.execute_async_v3(stream_handle=self.stream.cuda_stream)
            self.stream.synchronize()

        depth_raw  = self.d_output.cpu().numpy()[0]
        depth_norm = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        return depth_norm


# ──────────────────────────────────────────────────────────────────
# 시리얼 통신 클래스 (젯슨 → 피코)
# ──────────────────────────────────────────────────────────────────
class HardwareCommunicator:
    """
    Raspberry Pi Pico와의 UART 시리얼 통신.
    전송 포맷: "GRID:v0,v1,...,v23\\n"  (ASCII 문자열)
    """

    def __init__(self, port: str, baudrate: int = 115200):
        self.conn = None
        try:
            self.conn = serial.Serial(port, baudrate, timeout=1.0)
            time.sleep(2.0)  # 피코 부팅 대기

            # 피코 준비 신호 수신 확인
            resp = self.conn.readline().decode("utf-8", errors="ignore").strip()
            if resp == "PICO_READY":
                logger.info("피코 연결 확인 (%s)", port)
            else:
                logger.warning("피코 응답 불명확: '%s' — 계속 진행합니다.", resp)

        except serial.SerialException as e:
            logger.warning("시리얼 연결 실패 (%s): %s → 오프라인 모드", port, e)

    @property
    def connected(self) -> bool:
        return self.conn is not None and self.conn.is_open

    def transmit(self, grid_values: list) -> bool:
        """24개 그리드 값을 피코로 전송. 성공 시 True 반환."""
        if not self.connected:
            return False
        packet = "GRID:" + ",".join(map(str, grid_values)) + "\n"
        try:
            self.conn.write(packet.encode("utf-8"))
            return True
        except serial.SerialException as e:
            logger.error("전송 실패: %s", e)
            return False

    def close(self):
        if self.connected:
            self.conn.close()
            logger.info("시리얼 포트 닫힘")


# ──────────────────────────────────────────────────────────────────
# 6×4 그리드 깊이 산출
# ──────────────────────────────────────────────────────────────────
def compute_grid(depth_norm: np.ndarray,
                 grid_h: int = 4,
                 grid_w: int = 6) -> list:
    """
    깊이맵 → 24개 구역 평균값 리스트 (행 우선, 좌→우, 위→아래)
    가로 6구역 × 세로 4구역 (사람 눈/모니터 비율 기준)

    인덱스 배치:
      [ 0  1  2  3  4  5]  ← 1행
      [ 6  7  8  9 10 11]  ← 2행  → PCA9685 0x40 (채널 0~11)
      [12 13 14 15 16 17]  ← 3행
      [18 19 20 21 22 23]  ← 4행  → PCA9685 0x41 (채널 0~11)
    """
    result = []
    for row_strip in np.array_split(depth_norm, grid_h, axis=0):
        for cell in np.array_split(row_strip, grid_w, axis=1):
            result.append(int(np.mean(cell)))
    return result  # len == 24


# ──────────────────────────────────────────────────────────────────
# 디버그 시각화
# ──────────────────────────────────────────────────────────────────
def draw_debug(depth_norm: np.ndarray,
               grid_values: list,
               fps: float,
               grid_h: int = 4,
               grid_w: int = 6) -> np.ndarray:
    """깊이맵에 그리드 오버레이 + FPS 표시"""
    vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_INFERNO)
    h, w = vis.shape[:2]
    rh, rw = h // grid_h, w // grid_w

    for r in range(grid_h):
        for c in range(grid_w):
            idx  = r * grid_w + c
            x1, y1 = c * rw, r * rh
            # 두 보드 경계(인덱스 12, 3행 시작) 강조
            border = (0, 255, 255) if idx == 12 else (180, 180, 180)
            cv2.rectangle(vis, (x1, y1), (x1 + rw, y1 + rh), border, 1)
            cv2.putText(
                vis, str(grid_values[idx]),
                (x1 + 4, y1 + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (255, 255, 255), 1, cv2.LINE_AA,
            )

    cv2.putText(
        vis, f"FPS: {fps:.1f}",
        (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (0, 255, 0), 2, cv2.LINE_AA,
    )
    return vis


# ──────────────────────────────────────────────────────────────────
# 인자 파싱
# ──────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="시각-촉각 대체 보조장치 — Jetson 메인 파이프라인"
    )
    p.add_argument(
        "--depth", default="checkpoints/depth_anything_v2_vits.engine",
        help="Depth Anything V2 TRT 엔진 경로",
    )
    p.add_argument(
        "--port", default="/dev/ttyACM0",
        help="피코 시리얼 포트 (예: /dev/ttyACM0, /dev/ttyUSB0)",
    )
    p.add_argument("--baud",  default=115200, type=int, help="시리얼 보드레이트")
    p.add_argument("--no-hw", action="store_true",      help="하드웨어 없이 실행")
    p.add_argument("--debug", action="store_true",      help="디버그 시각화 창 표시")
    return p.parse_args()


# ──────────────────────────────────────────────────────────────────
# 메인 파이프라인
# ──────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    PROC_SIZE      = (518, 518)   # Depth Anything V2 ViT-S 최소 입력 (14×37)
    GRID_H, GRID_W = 4, 6        # 가로 6 × 세로 4 = 24구역 (모니터/눈 비율)

    # ── 모델 및 하드웨어 초기화 ──
    logger.info("시스템 초기화 중...")
    depth_estimator = DepthEstimatorTRT(args.depth, input_shape=PROC_SIZE)
    hardware = None if args.no_hw else HardwareCommunicator(args.port, args.baud)

    # ── 카메라 초기화 (V4L2 + MJPEG, 720p) ──
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        logger.error("카메라를 열 수 없습니다.")
        return

    logger.info("파이프라인 시작. 종료: Ctrl+C%s",
                " 또는 디버그 창에서 'q'" if args.debug else "")

    try:
        while True:
            t0 = time.perf_counter()

            ret, frame = cap.read()
            if not ret:
                logger.warning("프레임 읽기 실패, 재시도...")
                continue

            # 1. 518×518 리사이즈
            frame_518 = cv2.resize(frame, PROC_SIZE, interpolation=cv2.INTER_LINEAR)

            # 2. Depth Anything V2 TRT 추론
            depth_norm = depth_estimator.infer(frame_518)

            # 3. 6×4 그리드 평균 깊이 산출 (24개 값)
            grid_values = compute_grid(depth_norm, GRID_H, GRID_W)

            # 4. 피코로 전송: "GRID:v0,...,v23\n"
            if hardware:
                hardware.transmit(grid_values)

            fps = 1.0 / (time.perf_counter() - t0)
            logger.debug(
                "FPS: %.1f | pwm1(0x40)=%s | pwm2(0x41)=%s",
                fps, grid_values[:12], grid_values[12:],
            )

            # 5. 디버그 시각화 (--debug 플래그 시)
            if args.debug:
                vis = draw_debug(depth_norm, grid_values, fps, GRID_H, GRID_W)
                cv2.imshow("Tactile Vision Debug", vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        logger.info("사용자 중단 요청")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if hardware:
            hardware.close()
        logger.info("자원 해제 완료")


if __name__ == "__main__":
    main()
