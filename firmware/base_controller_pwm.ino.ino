/*
 * base_controller_pwm.ino (UART + PWM 버전)
 * 보드: Arduino Nano 33 BLE Sense (3.3V, 내장 IMU LSM9DS1)
 *
 * UART(Serial1): D1(TX)→Pi GPIO15, D0(RX)→Pi GPIO14, GND 공통
 * PWM: D10→ENA(왼쪽 속도), D11→ENB(오른쪽 속도)
 *
 * 프로토콜 (115200 baud):
 *   [받기]  M,<left>,<right>   (-255~255, 부호=방향/크기=속도)
 *   [보내기] R,ready / E,<l>,<r> / I,ax,ay,az,gx,gy,gz / Mag,mx,my,mz / S,timeout
 */

#include <Arduino_LSM9DS1.h>

// 모터 방향 핀
const int IN1 = 2, IN2 = 3, IN3 = 4, IN4 = 5;
// 모터 속도 PWM 핀
const int ENA = 10, ENB = 11;
// 엔코더 핀
const int ENC_L_A = 6, ENC_L_B = 7, ENC_R_A = 8, ENC_R_B = 9;

// PWM 속도 제한
const int PWM_MAX = 255;
const int PWM_MIN = 100;   // 이하면 모터 안 돎 (검증으로 확인)

volatile long countL = 0, countR = 0;

const unsigned long SEND_INTERVAL = 20;   // 50Hz
const unsigned long MAG_INTERVAL  = 100;  // 10Hz
const unsigned long CMD_TIMEOUT   = 500;  // 0.5초
unsigned long lastSendTime = 0, lastMagTime = 0, lastCmdTime = 0;
bool timedOut = false;

char buf[32];
int bufIdx = 0;
bool imuOk = false;

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial1.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  stopMotors();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, CHANGE);

  if (IMU.begin()) { imuOk = true;  Serial1.println("R,imu_ok"); }
  else             { imuOk = false; Serial1.println("R,imu_not_connected"); }

  lastCmdTime = millis();
  Serial1.println("R,ready");
  Serial.println("[USB] base_controller PWM ready");
}

void loop() {
  readSerial();

  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    if (!timedOut) { stopMotors(); Serial1.println("S,timeout"); timedOut = true; }
  }

  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;
    sendEncoder();
    sendImu();
  }
  if (now - lastMagTime >= MAG_INTERVAL) {
    lastMagTime = now;
    sendMag();
  }
}

void readSerial() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') { buf[bufIdx] = '\0'; parseCommand(buf); bufIdx = 0; }
    else if (bufIdx < (int)sizeof(buf) - 1) { buf[bufIdx++] = c; }
  }
}

void parseCommand(char *s) {
  if (s[0] != 'M') return;
  int l = 0, r = 0;
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

// speed: -255~255 (부호=방향, 크기=속도)
void setMotor(int leftSpeed, int rightSpeed) {
  setOneMotor(leftSpeed, IN1, IN2, ENA);
  setOneMotor(rightSpeed, IN3, IN4, ENB);
}

void setOneMotor(int speed, int inA, int inB, int enPin) {
  if (speed > 0)      { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); }
  else if (speed < 0) { digitalWrite(inA, HIGH); digitalWrite(inB, LOW);  }
  else                { digitalWrite(inA, LOW);  digitalWrite(inB, LOW);  }

  int pwm = abs(speed);
  if (pwm == 0) analogWrite(enPin, 0);
  else          analogWrite(enPin, constrain(pwm, PWM_MIN, PWM_MAX));
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void onLeftA() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) countL++; else countL--;
}
void onRightA() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) countR++; else countR--;
}

void sendEncoder() {
  noInterrupts();
  long l = countL, r = countR;
  interrupts();
  Serial1.print("E,"); Serial1.print(l); Serial1.print(","); Serial1.println(r);
}

void sendImu() {
  if (!imuOk) return;
  float ax, ay, az, gx, gy, gz;
  bool haveA = false, haveG = false;
  if (IMU.accelerationAvailable()) { IMU.readAcceleration(ax, ay, az); haveA = true; }
  if (IMU.gyroscopeAvailable())    { IMU.readGyroscope(gx, gy, gz);    haveG = true; }
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