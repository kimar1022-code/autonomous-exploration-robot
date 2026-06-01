/*
 * base_controller.ino (UART 버전)
 * 자율주행 탐사로봇 - 통합 펌웨어
 * 보드: Arduino Nano 33 BLE Sense (3.3V, 내장 IMU LSM9DS1)
 *
 * ★ 라즈베리파이와 GPIO UART(Serial1)로 통신:
 *     아두이노 D1(TX) → Pi RX(GPIO15, 핀10)
 *     아두이노 D0(RX) → Pi TX(GPIO14, 핀8)
 *     아두이노 GND    → Pi GND(핀6)
 *   USB(Serial)는 노트북 디버깅 모니터링용으로 함께 동작
 *
 * 역할:
 *   - 라즈베리파이로부터 모터 명령 수신
 *   - 엔코더 카운트 + IMU raw 데이터를 50Hz로 송신
 *   - 0.5초 명령 없으면 안전 정지
 *
 * 시리얼 프로토콜 (115200 baud):
 *   [받기]  M,<leftDir>,<rightDir>\n   (-1 후진 / 0 정지 / 1 전진)
 *   [보내기] R,ready                    부팅 완료
 *           E,<lcnt>,<rcnt>            엔코더 카운트 (50Hz)
 *           I,ax,ay,az,gx,gy,gz        IMU 가속도+자이로 (50Hz)
 *           Mag,mx,my,mz              지자계 (약 10Hz)
 *           S,timeout                  0.5초 명령 없어 자동 정지
 */

#include <Arduino_LSM9DS1.h>

// ===== 핀 정의 =====
// 모터 드라이버 (L298N)
const int IN1 = 2;   // 왼쪽 방향 1
const int IN2 = 3;   // 왼쪽 방향 2
const int IN3 = 4;   // 오른쪽 방향 1
const int IN4 = 5;   // 오른쪽 방향 2

// 엔코더
const int ENC_L_A = 6;
const int ENC_L_B = 7;
const int ENC_R_A = 8;
const int ENC_R_B = 9;

// ===== 엔코더 카운트 (인터럽트에서 변경) =====
volatile long countL = 0;
volatile long countR = 0;

// ===== 타이밍 =====
const unsigned long SEND_INTERVAL = 20;     // 50Hz = 20ms (엔코더, IMU)
const unsigned long MAG_INTERVAL  = 100;    // 10Hz = 100ms (지자계)
const unsigned long CMD_TIMEOUT   = 500;    // 0.5초 명령 타임아웃
unsigned long lastSendTime = 0;
unsigned long lastMagTime  = 0;
unsigned long lastCmdTime  = 0;
bool timedOut = false;

// ===== 시리얼 수신 버퍼 =====
char buf[32];
int bufIdx = 0;

// ===== IMU 가용 여부 =====
bool imuOk = false;

void setup() {
  Serial.begin(115200);    // USB - 노트북 디버깅 모니터링용
  delay(3000);             // Nano 33 BLE USB 시리얼 안정화 대기
  Serial1.begin(115200);   // 하드웨어 UART (D0=RX, D1=TX) - 라즈베리파이 통신용

  // 모터 핀 출력 + 정지
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  // 엔코더 핀 입력 (내부 풀업)
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, CHANGE);

  // IMU 시작 (실패해도 멈추지 않고 계속 동작)
  if (IMU.begin()) {
    imuOk = true;
    Serial1.println("R,imu_ok");
  } else {
    imuOk = false;
    Serial1.println("R,imu_not_connected");
  }

  lastCmdTime = millis();
  Serial1.println("R,ready");
  Serial.println("[USB] base_controller UART ready");  // 노트북 확인용
}

void loop() {
  // 1) 시리얼 명령 수신 처리
  readSerial();

  // 2) 명령 타임아웃 체크 (0.5초 무명령 -> 정지)
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    if (!timedOut) {
      stopMotors();
      Serial1.println("S,timeout");
      timedOut = true;
    }
  }

  // 3) 50Hz로 엔코더 + IMU(가속도/자이로) 송신
  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;
    sendEncoder();
    sendImu();
  }

  // 4) 10Hz로 지자계 송신
  if (now - lastMagTime >= MAG_INTERVAL) {
    lastMagTime = now;
    sendMag();
  }
}

// ===== 시리얼 수신: M,<l>,<r> 파싱 =====
void readSerial() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buf[bufIdx] = '\0';
      parseCommand(buf);
      bufIdx = 0;
    } else if (bufIdx < (int)sizeof(buf) - 1) {
      buf[bufIdx++] = c;
    }
  }
}

void parseCommand(char *s) {
  // 형식: M,<leftDir>,<rightDir>
  if (s[0] != 'M') return;

  int l = 0, r = 0;
  // M,l,r 파싱
  char *p = strchr(s, ',');
  if (!p) return;
  l = atoi(p + 1);
  p = strchr(p + 1, ',');
  if (!p) return;
  r = atoi(p + 1);

  setMotor(l, r);
  lastCmdTime = millis();
  timedOut = false;
}

// ===== 모터 제어 =====
// dir: 1 전진, -1 후진, 0 정지
void setMotor(int leftDir, int rightDir) {
  // 왼쪽
  if (leftDir > 0)      { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else if (leftDir < 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else                  { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }

  // 오른쪽
  if (rightDir > 0)      { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else if (rightDir < 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else                   { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ===== 엔코더 인터럽트 =====
void onLeftA() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) countL++;
  else countL--;
}

void onRightA() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) countR++;
  else countR--;
}

// ===== 송신 =====
void sendEncoder() {
  // 인터럽트 잠깐 막고 안전하게 읽기
  noInterrupts();
  long l = countL;
  long r = countR;
  interrupts();

  Serial1.print("E,");
  Serial1.print(l);
  Serial1.print(",");
  Serial1.println(r);
}

void sendImu() {
  if (!imuOk) return;

  float ax, ay, az, gx, gy, gz;
  bool haveA = false, haveG = false;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    haveA = true;
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    haveG = true;
  }

  // 둘 다 새 값이 있을 때만 송신 (없으면 이번 주기 건너뜀)
  if (haveA && haveG) {
    Serial1.print("I,");
    Serial1.print(ax, 4); Serial1.print(",");
    Serial1.print(ay, 4); Serial1.print(",");
    Serial1.print(az, 4); Serial1.print(",");
    Serial1.print(gx, 4); Serial1.print(",");
    Serial1.print(gy, 4); Serial1.print(",");
    Serial1.println(gz, 4);
  }
}

void sendMag() {
  if (!imuOk) return;

  float mx, my, mz;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    Serial1.print("Mag,");
    Serial1.print(mx, 4); Serial1.print(",");
    Serial1.print(my, 4); Serial1.print(",");
    Serial1.println(mz, 4);
  }
}
