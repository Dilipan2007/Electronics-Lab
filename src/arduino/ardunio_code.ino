#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

// ---- Hardware Pins ----
const uint8_t TILT_PIN = 8;
const uint8_t PAN_PIN  = 9;

// ---- Scan Limits ----
const int PAN_MIN  = 75,  PAN_MAX  = 115, PAN_STEP = 1;
const int TILT_MIN = 60,  TILT_MAX = 155;

// ---- Motion Speeds ----
const float TILT_SPEED_DPS = 25.0; 
const int   ANGLE_DELAY_MS = 25;   
const int   PAN_SETTLE_MS  = 120;  // Dwell time for Pan to complete physical movement

const uint16_t MIN_MM = 30;
const uint16_t MAX_MM = 1500;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo panServo;
Servo tiltServo;

int panPos = PAN_MIN;

inline int degToUs(float deg) {
  return (int)(544.0f + deg * (2400.0f - 544.0f) / 180.0f);
}

bool getReading(uint16_t &mm) {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    mm = measure.RangeMilliMeter;
    return (mm >= MIN_MM && mm <= MAX_MM);
  }
  return false;
}

// Move Pan with Timer1 active, then detach to eliminate jitter
void movePanIsolated(int angle) {
  panPos = angle;
  
  // 1. Re-enable Timer1 channel for Pan
  if (!panServo.attached()) {
    panServo.attach(PAN_PIN);
  }
  
  // 2. Drive to target angle and let mechanical gear train arrive
  panServo.writeMicroseconds(degToUs(angle));
  delay(PAN_SETTLE_MS);

  // 3. Disable PWM pulse generation on Pin 9 (isolates from I2C interrupts)
  panServo.detach();
  digitalWrite(PAN_PIN, LOW); // Force line to stable 0V logic ground
}

// Sweep Tilt continuously (Pan is completely unpowered / non-jittering)
void sweepTilt(float from, float to) {
  float dir = (to > from) ? 1.0f : -1.0f;
  float totalDeg = fabs(to - from);
  unsigned long durMs = (unsigned long)(totalDeg / TILT_SPEED_DPS * 1000.0f);

  tiltServo.writeMicroseconds(degToUs(from));

  unsigned long t0 = millis();
  unsigned long lastServoUpdate = 0;

  while (true) {
    unsigned long el = millis() - t0;
    if (el > durMs + ANGLE_DELAY_MS + 25) break;

    // Refresh Tilt servo pulse every ~4 ms
    if (el - lastServoUpdate >= 4 || el == 0) {
      lastServoUpdate = el;
      unsigned long e = (el < durMs) ? el : durMs;
      float tiltPos = from + dir * (e * TILT_SPEED_DPS / 1000.0f);
      tiltServo.writeMicroseconds(degToUs(tiltPos));
    }

    // Direct I2C read
    uint16_t mm = 0;
    unsigned long sampleTime = millis();
    bool valid = getReading(mm);

    long tm = (long)(sampleTime - t0) - ANGLE_DELAY_MS;
    if (tm >= 0 && tm <= (long)durMs) {
      float ang = from + dir * (tm * TILT_SPEED_DPS / 1000.0f);
      Serial.print(panPos);   Serial.print(',');
      Serial.print(ang, 1);   Serial.print(',');
      Serial.println(valid ? (mm / 10.0f) : -1.0f, 1);
    }
  }

  // Hold Tilt at boundary
  tiltServo.writeMicroseconds(degToUs(to));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400 kHz Fast I2C

  if (!lox.begin()) {
    Serial.println(F("# ERROR: VL53L0X failed to boot over I2C"));
    while (1);
  }

  lox.setMeasurementTimingBudgetMicroSeconds(30000);

  // Attach Tilt permanently (Tilt must continuously fight gravity)
  tiltServo.attach(TILT_PIN);
  tiltServo.writeMicroseconds(degToUs(TILT_MIN));

  // Home Pan and mute it
  movePanIsolated(PAN_MIN);

  Serial.println(F("# Scanner ready"));
}

void loop() {
  Serial.println(F("START"));
  bool up = true;

  for (int pan = PAN_MIN; pan <= PAN_MAX; pan += PAN_STEP) {
    // Step Pan into position, then mute Pin 9
    movePanIsolated(pan);

    // Tilt sweep executes while Pin 9 has 0 pulses
    if (up) sweepTilt(TILT_MIN, TILT_MAX);
    else    sweepTilt(TILT_MAX, TILT_MIN);

    up = !up;
  }

  Serial.println(F("DONE"));
  while (true) {}
}