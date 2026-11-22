#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

// ---- Hardware Pins ----
const uint8_t TILT_PIN = 8;
const uint8_t PAN_PIN  = 9;

// ---- Scan Grid Bounds ----
const int PAN_MIN   = 60,  PAN_MAX  = 120, PAN_STEP  = 1;
const int TILT_MIN  = 50,  TILT_MAX = 120, TILT_STEP = 1;

// ---- Timing & Dwell Configuration ----
const int SERVO_SETTLE_MS       = 20;  // Dwell before each reading for zero motion blur
const int RESET_DWELL_MS        = 200; // Time for Tilt to physically return to bottom
const int CAP_RECHARGE_DWELL_MS = 80;  // Dwell to top up 1500uF cap before Pan takes current

// ---- Sensor Thresholds ----
const uint16_t MIN_MM = 30;
const uint16_t MAX_MM = 1500;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo panServo;
Servo tiltServo;

// Convert degrees to exact microsecond pulses
inline int degToUs(float deg) {
  return (int)(544.0f + deg * (2400.0f - 544.0f) / 180.0f);
}

// 3-Sample Median Filter: Removes random optical outliers and laser speckle
uint16_t getFilteredDistance() {
  uint16_t r[3];
  uint8_t validCount = 0;

  for (uint8_t i = 0; i < 3; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= MIN_MM && measure.RangeMilliMeter <= MAX_MM) {
      r[validCount++] = measure.RangeMilliMeter;
    }
  }

  if (validCount == 0) return 0; // Out of range or no return
  if (validCount == 1) return r[0];
  if (validCount == 2) return (r[0] + r[1]) / 2;

  // Sorting 3 elements to extract median
  if (r[0] > r[1]) { uint16_t t = r[0]; r[0] = r[1]; r[1] = t; }
  if (r[1] > r[2]) { uint16_t t = r[1]; r[1] = r[2]; r[2] = t; }
  if (r[0] > r[1]) { uint16_t t = r[0]; r[0] = r[1]; r[1] = t; }
  return r[1];
}

// BUG FIX: Pre-pulse wakeup + micro-torque kick + synchronized detach
void stepPanIsolated(int angle) {
  int targetUs = degToUs(angle);

  if (!panServo.attached()) {
    // 1. Pre-load target pulse BEFORE attaching so the very first pulse is valid
    panServo.writeMicroseconds(targetUs);
    panServo.attach(PAN_PIN);
    delay(25); // Give MG90S control IC time to wake up and lock onto 50Hz signal
  }

  // 2. Micro-kick (+15 us overshoot for 35ms) to break static friction on small 1-2 deg steps
  panServo.writeMicroseconds(targetUs + 15);
  delay(35);

  // 3. Settle on exact target angle
  panServo.writeMicroseconds(targetUs);
  delay(120);

  // 4. Wait for active HIGH pulse on Pin 9 to naturally finish before detaching
  // Prevents truncated micro-pulses that cause random jumps to 0 degrees
  unsigned long waitStart = micros();
  while (digitalRead(PAN_PIN) == HIGH && (micros() - waitStart < 3000)) {
    // Wait up to 3ms for pulse to end
  }

  panServo.detach();          // Safely cut software PWM output
  digitalWrite(PAN_PIN, LOW); // Hold line ground steady
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400 kHz Fast I2C

  if (!lox.begin()) {
    Serial.println(F("# ERROR: VL53L0X failed to initialize over I2C."));
    Serial.println(F("# Confirm left header wiring (SCL->A5, DA->A4) and PS->GND."));
    while (1);
  }

  // 20 ms timing budget per single shot
  lox.setMeasurementTimingBudgetMicroSeconds(20000);

  // Attach Tilt permanently (Tilt fights gravity)
  tiltServo.attach(TILT_PIN);
  tiltServo.writeMicroseconds(degToUs(TILT_MIN));

  // Home Pan and mute it safely
  stepPanIsolated(PAN_MIN);
  delay(500);

  Serial.println(F("# 3D Scanner Ready (Discrete Mode + Torque Kick Fix)"));
}

void loop() {
  Serial.println(F("START"));

  for (int pan = PAN_MIN; pan <= PAN_MAX; pan += PAN_STEP) {
    // 1. Move Pan to next column with wakeup & static-friction kick
    stepPanIsolated(pan);

    // 2. Unidirectional Tilt sweep (always sweeps bottom to top to eliminate gear backlash)
    for (int tilt = TILT_MIN; tilt <= TILT_MAX; tilt += TILT_STEP) {
      tiltServo.writeMicroseconds(degToUs(tilt));
      delay(SERVO_SETTLE_MS); // Wait until motor fully stops before optical measurement

      uint16_t mm = getFilteredDistance();

      // Output stream format: pan,tilt,distance_cm
      Serial.print(pan);
      Serial.print(',');
      Serial.print(tilt);
      Serial.print(',');
      Serial.println(mm > 0 ? (mm / 10.0f) : -1.0f, 2);
    }

    // 3. Reset Tilt to bottom
    tiltServo.writeMicroseconds(degToUs(TILT_MIN));
    delay(RESET_DWELL_MS);

    // 4. Capacitor recharge & motor back-EMF decay dwell
    // Ensures 1500uF capacitor is fully refreshed to 5.0V before Pan moves
    delay(CAP_RECHARGE_DWELL_MS);
  }

  Serial.println(F("DONE"));
  while (true) {} // Halt when scan finishes
}