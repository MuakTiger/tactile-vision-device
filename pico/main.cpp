/**
 * 시각-촉각 대체 보조장치 — Raspberry Pi Pico 펌웨어
 *
 * 구성:
 *   Jetson → USB UART → Pico → I2C → PCA9685 x2 → SG90 x24
 *   PCA9685 #1 (0x40): 채널 0~11 → 그리드 상단 12구역
 *   PCA9685 #2 (0x41): 채널 0~11 → 그리드 하단 12구역
 *
 * 수신 포맷: "GRID:v0,v1,...,v23\n"  (ASCII, 115200bps)
 *
 * 의존 라이브러리 (platformio.ini 또는 Arduino IDE):
 *   - Wire (내장)
 *   - adafruit/Adafruit PWM Servo Driver Library
 *
 * 하드웨어 연결:
 *   GP4 (SDA) → PCA9685 SDA
 *   GP5 (SCL) → PCA9685 SCL
 *   3V3 / GND → PCA9685 VCC / GND
 *   외부 5V   → PCA9685 V+ (서보 전원)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ──────────────────────────────────────────────
// 설정 상수
// ──────────────────────────────────────────────
static constexpr int    MOTOR_COUNT    = 24;
static constexpr int    BOARD_CH       = 12;   // 보드당 채널 수 (24 / 2)

// SG90 안전 PWM 범위 (50 Hz 기준, 4096 tick 단위)
// 랙앤피니언 실제 스트로크에 맞게 SAFE_MAX 조정 권장
// 처음 테스트 시: SAFE_MAX = 350 으로 낮게 시작
static constexpr uint16_t SAFE_MIN     = 250;  // ~0.5 ms → 핀 하강 (홈)
static constexpr uint16_t SAFE_MAX     = 490;  // ~2.3 ms → 핀 최대 상승

// 깊이값 하한 임계치 — 이 값 미만은 '물체 없음' → 홈 유지
static constexpr uint8_t  DEPTH_THR    = 20;

// 통신 Failsafe: 이 시간(ms) 동안 패킷 없으면 전체 홈 복귀
static constexpr unsigned long COMM_TIMEOUT = 2000;

// ──────────────────────────────────────────────
// PCA9685 객체
// ──────────────────────────────────────────────
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // 기본 보드
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // A0 납땜 보드

unsigned long lastReceiveTime = 0;

// ──────────────────────────────────────────────
// 전체 서보 홈 복귀
// ──────────────────────────────────────────────
void resetAllServos() {
  for (int i = 0; i < BOARD_CH; i++) {
    pwm1.setPWM(i, 0, SAFE_MIN);
    pwm2.setPWM(i, 0, SAFE_MIN);
  }
}

// ──────────────────────────────────────────────
// 깊이값(0~255) → PWM tick 변환
// Depth Anything V2: 값이 클수록 가까움 (disparity)
// → 가까울수록 막대가 더 올라옴 (SAFE_MAX 방향)
// ──────────────────────────────────────────────
uint16_t depthToPWM(uint8_t rawVal) {
  if (rawVal < DEPTH_THR) return SAFE_MIN;  // 물체 없음 → 홈
  return (uint16_t)map(rawVal, DEPTH_THR, 255, SAFE_MIN, SAFE_MAX);
}

// ──────────────────────────────────────────────
// 패킷 파싱: "GRID:v0,v1,...,v23"
// 반환값: 24개 파싱 성공 시 true
// ──────────────────────────────────────────────
bool parsePacket(const String& packet, uint8_t values[MOTOR_COUNT]) {
  if (!packet.startsWith("GRID:")) return false;

  String data = packet.substring(5);
  data.trim();

  char buf[128];
  data.toCharArray(buf, sizeof(buf));

  int    index = 0;
  char*  token = strtok(buf, ",");

  while (token != nullptr && index < MOTOR_COUNT) {
    int v       = atoi(token);
    values[index++] = (uint8_t)constrain(v, 0, 255);
    token = strtok(nullptr, ",");
  }

  return (index == MOTOR_COUNT);
}

// ──────────────────────────────────────────────
// 24개 서보 일괄 업데이트
// 가로 6 × 세로 4 그리드 (행 우선)
// index  0~11 → pwm1 (PCA9685 0x40) 채널 0~11  ← 상단 2행
// index 12~23 → pwm2 (PCA9685 0x41) 채널 0~11  ← 하단 2행
// ──────────────────────────────────────────────
void applyServos(uint8_t values[MOTOR_COUNT]) {
  for (int i = 0; i < BOARD_CH; i++) {
    pwm1.setPWM(i, 0, depthToPWM(values[i]));
    pwm2.setPWM(i, 0, depthToPWM(values[i + BOARD_CH]));
  }
}

// ──────────────────────────────────────────────
// 초기화
// ──────────────────────────────────────────────
void setup() {
  Wire.begin();               // GP4=SDA, GP5=SCL (arduino-pico 기본값)
  Serial.begin(115200);
  Serial.setTimeout(100);

  pwm1.begin();
  pwm1.setPWMFreq(50);        // SG90 표준 주파수 50 Hz

  pwm2.begin();
  pwm2.setPWMFreq(50);

  resetAllServos();           // 부팅 시 전체 홈 복귀
  delay(500);

  Serial.println("PICO_READY");
  lastReceiveTime = millis();
}

// ──────────────────────────────────────────────
// 메인 루프
// ──────────────────────────────────────────────
void loop() {
  // ── A. 패킷 수신 및 처리 ──
  if (Serial.available() > 0) {
    String packet = Serial.readStringUntil('\n');
    packet.trim();

    uint8_t values[MOTOR_COUNT];
    if (parsePacket(packet, values)) {
      applyServos(values);
      lastReceiveTime = millis();
    }
    // 파싱 실패 시 이전 포지션 유지 (갑작스러운 움직임 방지)
  }

  // ── B. Failsafe: 통신 단절 시 홈 복귀 ──
  if (millis() - lastReceiveTime > COMM_TIMEOUT) {
    resetAllServos();
    lastReceiveTime = millis();  // 반복 호출 방지
  }
}
