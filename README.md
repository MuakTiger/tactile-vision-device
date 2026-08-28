# 시각장애인을 위한 실시간 시각-촉각 대체 보조장치

> 카메라로 촬영한 장면을 깊이 정보로 변환하고, 24개의 모터로 공간을 손끝에 전달하는 보조장치입니다.

---

## 시스템 구성

```
웹캠
 │
 ▼
Jetson Orin Developer Kit (main.py)
 ├─ Depth Anything V2 TensorRT 추론
 ├─ 4×6 그리드 깊이 산출 (가로6×세로4, 24구역)
 └─ UART 시리얼 전송 "GRID:v0,...,v23\n"
       │
       ▼
Raspberry Pi Pico (pico/main.cpp)
 ├─ I2C → PCA9685 #1 (0x40) → SG90 × 12 (상단 2행)
 └─ I2C → PCA9685 #2 (0x41) → SG90 × 12 (하단 2행)
              │
              ▼
         랙앤피니언 기구부 (Z축 막대 돌출)
```

## 하드웨어 사양

| 구성품 | 사양 |
|---|---|
| 메인 보드 | NVIDIA Jetson Orin Developer Kit |
| 마이크로컨트롤러 | Raspberry Pi Pico (RP2040) |
| PWM 드라이버 | PCA9685 × 2 (I2C: 0x40, 0x41) |
| 액추에이터 | SG90 서보모터 × 24 |
| 기구부 | 랙앤피니언 방식 Z축 돌출 핀 |
| 카메라 | USB 웹캠 (MJPEG, 720p) |

## 그리드 배치

가로 6구역 × 세로 4구역 (사람 눈/모니터 비율 기준)

```
[ 0][ 1][ 2][ 3][ 4][ 5]  ← 1행
[ 6][ 7][ 8][ 9][10][11]  ← 2행  } PCA9685 0x40 (채널 0~11)
[12][13][14][15][16][17]  ← 3행
[18][19][20][21][22][23]  ← 4행  } PCA9685 0x41 (채널 0~11)
```

깊이값이 클수록 (물체가 가까울수록) 해당 구역의 핀이 더 높이 돌출됩니다.

---

## 설치 및 실행

### Jetson 환경 설정

```bash
# 1. 패키지 설치 (torch, tensorrt는 JetPack에 포함)
pip install -r requirements.txt

# 2. Depth Anything V2 TRT 엔진을 checkpoints/ 에 배치
#    파일명: depth_anything_v2_vits.engine
```

### 실행

```bash
# 기본 실행
python main.py

# 디버그 시각화 창 포함
python main.py --debug

# 하드웨어 없이 시각화만 (개발/테스트용)
python main.py --no-hw --debug

# 시리얼 포트 지정
python main.py --port /dev/ttyACM1 --debug
```

### 실행 옵션

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--depth` | `checkpoints/depth_anything_v2_vits.engine` | TRT 엔진 경로 |
| `--port` | `/dev/ttyACM0` | 피코 시리얼 포트 |
| `--baud` | `115200` | 시리얼 보드레이트 |
| `--no-hw` | `False` | 하드웨어 없이 실행 |
| `--debug` | `False` | 디버그 시각화 창 표시 |

---

## 피코 펌웨어 빌드

**Raspberry Pi Pico (RP2040)** 용 펌웨어입니다.  
[arduino-pico 코어](https://github.com/earlephilhower/arduino-pico) (RP2040용 Arduino 지원 코어)를 사용해 Arduino IDE 또는 PlatformIO로 빌드합니다.

**의존 라이브러리:**
- `Adafruit PWM Servo Driver Library`

**I2C 핀 (arduino-pico 기본값):**
- `GP4` → SDA
- `GP5` → SCL

**PCA9685 주소 설정:**
- 보드 #1: A0~A5 모두 오픈 → `0x40`
- 보드 #2: A0 납땜(HIGH) → `0x41`

**SAFE_MAX 조정 (중요):**
```cpp
// pico/main.cpp 상단
static constexpr uint16_t SAFE_MAX = 490;  // 랙 스트로크에 맞게 조정
```
처음 테스트 시 `350`으로 낮춰서 기구부 한계 확인 후 서서히 올릴 것.

---

## 프로젝트 구조

```
tactile-vision-device/
├── main.py              # 젯슨 메인 파이프라인
├── requirements.txt
├── README.md
├── .gitignore
├── checkpoints/         # TRT 엔진 파일 (git 추적 제외)
│   └── depth_anything_v2_vits.engine
└── pico/
    └── main.cpp         # 피코 펌웨어
```

---

## 참고

- [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2)
- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
- [Adafruit PCA9685 Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
- [arduino-pico](https://github.com/earlephilhower/arduino-pico)
