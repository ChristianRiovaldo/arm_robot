#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t NUM_SERVOS = 6;
const uint8_t FIRST_CHANNEL = 0;
const uint8_t STEP_DELAY_MS = 15;
const uint8_t DEG_PER_STEP = 1;   
const bool ECHO_ALL = false; 
const unsigned long TELEMETRY_MS = 100; 

const uint16_t SERVO_MIN_US[NUM_SERVOS] = { 500,  600,  600,     600,  600,  600};
const uint16_t SERVO_MAX_US[NUM_SERVOS] = {2500, 2400, 2400,    2400, 2400, 2400};

const int16_t SERVO_MIN_DEG[NUM_SERVOS] = {   0,   90,   45,       0,  100,   70};
const int16_t SERVO_MAX_DEG[NUM_SERVOS] = { 180,  180,  150,     180,  180,  110};

const int UNKNOWN_ANGLE = -32768;

int currentAngle[NUM_SERVOS];   
int targetAngle[NUM_SERVOS];     
unsigned long lastStepMs = 0;
unsigned long lastTelemetryMs = 0;
int reportedAngle[NUM_SERVOS];

char lineBuf[64];
uint8_t lineLen = 0;
bool lineOverflow = false;

uint16_t angleToUs(uint8_t idx, int angle) {
  long us = map(angle, 0, 180, SERVO_MIN_US[idx], SERVO_MAX_US[idx]);
  return (uint16_t)constrain(us, 0L, 20000L);
}

void writeServo(uint8_t idx, int angle) {
  pwm.writeMicroseconds(FIRST_CHANNEL + idx, angleToUs(idx, angle));
  currentAngle[idx] = angle;
}

int setTarget(uint8_t idx, long angle) {
  long lo = SERVO_MIN_DEG[idx];
  long hi = SERVO_MAX_DEG[idx];
  long a  = constrain(angle, lo, hi);
  targetAngle[idx] = (int)a;
  return (int)a;
}

void updateServos() {
  unsigned long now = millis();
  if (STEP_DELAY_MS > 0 && (now - lastStepMs) < STEP_DELAY_MS) return;
  lastStepMs = now;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    int tgt = targetAngle[i];
    if (tgt == UNKNOWN_ANGLE) continue;

    int cur = currentAngle[i];

    if (cur == UNKNOWN_ANGLE || STEP_DELAY_MS == 0) {
      writeServo(i, tgt);
      continue;
    }
    if (cur == tgt) continue;

    int diff = tgt - cur;
    int step = diff;
    if (diff > DEG_PER_STEP)       step = DEG_PER_STEP;
    else if (diff < -DEG_PER_STEP) step = -DEG_PER_STEP;
    writeServo(i, cur + step);
  }
}

void reportAngles() {
  if (TELEMETRY_MS == 0) return;
  unsigned long now = millis();
  if ((now - lastTelemetryMs) < TELEMETRY_MS) return;

  bool changed = false;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (currentAngle[i] != reportedAngle[i]) { changed = true; break; }
  }
  if (!changed) return;

  lastTelemetryMs = now;
  Serial.print(F("P"));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    reportedAngle[i] = currentAngle[i];
    Serial.print(F(" "));
    Serial.print(currentAngle[i] == UNKNOWN_ANGLE ? -1 : currentAngle[i]);
  }
  Serial.println();
}

uint8_t parseInts(char *p, long *out, uint8_t maxN) {
  uint8_t n = 0;
  while (true) {
    while (*p == ' ' || *p == ',' || *p == '\t') p++;
    if (*p == '\0') break;
    if (n >= maxN) return 255;
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) return 255;
    out[n++] = v;
    p = end;
  }
  return n;
}

void handleLine(char *s) {
  while (*s == ' ' || *s == '\t') s++;

  char cmd = 'S';
  if (*s == 'A' || *s == 'a') { cmd = 'A'; s++; }
  else if (*s == 'S' || *s == 's') { cmd = 'S'; s++; }
  else if (*s == 'H' || *s == 'h') { cmd = 'H'; s++; }

  long v[NUM_SERVOS];
  uint8_t n = parseInts(s, v, NUM_SERVOS);

  if (cmd == 'H') {
    if (n == 0) {
      for (uint8_t i = 0; i < NUM_SERVOS; i++) targetAngle[i] = currentAngle[i];
      Serial.println(F("OK halt"));
    } else if (n == 1 && v[0] >= 1 && v[0] <= NUM_SERVOS) {
      targetAngle[v[0] - 1] = currentAngle[v[0] - 1];
      Serial.print(F("OK halt servo "));
      Serial.println((int)v[0]);
    } else {
      Serial.println(F("ERR format H: H  atau  H <servo 1-6>"));
    }
    return;
  }

  if (cmd == 'A') {
    if (n != NUM_SERVOS) {
      Serial.println(F("ERR format A: A a1 a2 a3 a4 a5 a6"));
      return;
    }
    for (uint8_t i = 0; i < NUM_SERVOS; i++) setTarget(i, v[i]);
    if (ECHO_ALL) Serial.println(F("OK A"));
    return;
  }

  if (n != 2) {
    Serial.println(F("ERR format S: S <servo 1-6> <sudut>"));
    return;
  }
  if (v[0] < 1 || v[0] > NUM_SERVOS) {
    Serial.println(F("ERR servo harus 1 - 6"));
    return;
  }

  uint8_t idx = (uint8_t)(v[0] - 1);
  int applied = setTarget(idx, v[1]);

  Serial.print(F("OK servo "));
  Serial.print((int)v[0]);
  Serial.print(F(" -> "));
  Serial.print(applied);
  if (applied != v[1]) {
    Serial.print(F(" (dibatasi, batas "));
    Serial.print(SERVO_MIN_DEG[idx]);
    Serial.print(F("-"));
    Serial.print(SERVO_MAX_DEG[idx]);
    Serial.print(F(")"));
  }
  Serial.println();
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (lineOverflow) {
        Serial.println(F("ERR baris terlalu panjang"));
      } else if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i]  = UNKNOWN_ANGLE;
    targetAngle[i]   = UNKNOWN_ANGLE;
    reportedAngle[i] = UNKNOWN_ANGLE;
  }

  Serial.println(F("=== SERVO CONTROL (PCA9685) SIAP ==="));
  Serial.println(F("Perintah: A a1..a6  |  S n sudut  |  H [n]"));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(F("  Servo "));
    Serial.print(i + 1);
    Serial.print(F(" : batas "));
    Serial.print(SERVO_MIN_DEG[i]);
    Serial.print(F(" s/d "));
    Serial.println(SERVO_MAX_DEG[i]);
  }
}

void loop() {
  readSerial();
  updateServos();
  reportAngles();
}