#include <Arduino.h>
#include <PS4Controller.h>
#include <ESP32Servo.h>

#define SERVO_PIN 18          // Servo signal wire
#define MIN_ANGLE  2         // Safe mechanical minimum
#define MAX_ANGLE 179         // Safe mechanical maximum
#define OPEN_ANGLE 180        // Fully open preset
#define CLOSE_ANGLE 1        // Fully closed preset

#define SLEW_MAX_DEG_PER_S 180   // Max servo slew rate (°/s)
#define TRIGGER_MAX_DEG_PER_S 220 // Max analog trigger speed (°/s)

Servo gripper;
float currentAngle = (OPEN_ANGLE + CLOSE_ANGLE) / 2.0;
float targetAngle  = currentAngle;
uint32_t lastMs = 0;

// Clamp helper function: ensures current angle never goes below MIN_ANGLE or above MAX_ANGLE
static inline float clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Smooth movement (prevents servo jumping)
void smoothMove(float dt_s, float max_deg_per_s) {
  float maxStep = max_deg_per_s * dt_s;
  float delta   = targetAngle - currentAngle;
  if (fabs(delta) > maxStep) {
    currentAngle += (delta > 0 ? maxStep : -maxStep);
  } else {
    currentAngle = targetAngle;
  }
  gripper.write((int)round(clamp(currentAngle, MIN_ANGLE, MAX_ANGLE)));
}

void setup() {
  Serial.begin(115200);
  PS4.begin();
  Serial.println("Waiting for PS4 controller...");

  gripper.attach(SERVO_PIN);
  gripper.write((int)round(currentAngle));
  lastMs = millis();
}

void loop() {
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  if (!PS4.isConnected()) {
    Serial.println("PS4 not connected...");
    delay(1000);
    return;
  }

  
  if (PS4.R1()) {
    targetAngle = OPEN_ANGLE;
    Serial.println("R1 pressed → OPEN gripper fully");
  }
  if (PS4.L1()) {
    targetAngle = CLOSE_ANGLE;
    Serial.println("L1 pressed → CLOSE gripper fully");
  }

  // --- Proportional trigger control ---
  int r2 = PS4.R2Value(); // open
  int l2 = PS4.L2Value(); // close
  if (r2 > 0 || l2 > 0) {
    float openSpd  = map(r2, 0, 255, 0, TRIGGER_MAX_DEG_PER_S);
    float closeSpd = map(l2, 0, 255, 0, TRIGGER_MAX_DEG_PER_S);
    float net = openSpd - closeSpd; // positive = open, negative = close
    targetAngle = clamp(targetAngle + net * dt, MIN_ANGLE, MAX_ANGLE);
  }

  // --- Smooth motion toward target ---
  smoothMove(dt, SLEW_MAX_DEG_PER_S);

  delay(10);
}
