#include <Arduino.h>
#include <PS4Controller.h>
#include <ESP32Servo.h>

#define SERVOR_PIN    18      // change to your pin if needed (e.g., 25)
#define SERVOL_PIN    21

#define NEUTRAL_US   1530    // stop pulse you calibrated --> (deadzone for servo)
#define MAX_US       2000    // max pulse (your “up” direction) --> max servo speed
#define MIN_US       1000    // min pulse (your “down” direction)
#define DEADZONE     15      // move only if within ±15 is considered neutral
#define INVERT_STICK true    // PS4 Y is negative when UP; 'true' makes UP positive

Servo servoR;
Servo servoL;

void setup() {
  Serial.begin(115200);
  PS4.begin();               // for first pairing you may need PS4.begin("AA:BB:CC:DD:EE:FF")

  // ESP32Servo setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoR.setPeriodHertz(50);   // 50 Hz for hobby servos
  servoL.setPeriodHertz(50); 
           
  servoR.attach(SERVOR_PIN, 1000, 2000); // safe attach range
  servoL.attach(SERVOL_PIN, 1000, 2000); 

  servoR.writeMicroseconds(NEUTRAL_US); // start stopped
  servoL.writeMicroseconds(NEUTRAL_US);

  Serial.println("Ready: Stick UP (>deadzone) -> MAX_US (2000us), else STOP(1530us).");
}

void loop() {
  if (!PS4.isConnected()) {
    servoR.writeMicroseconds(NEUTRAL_US);
    servoL.writeMicroseconds(NEUTRAL_US);
    delay(20);
    return;
  }

  // Read sticks Y ~ [-128..+127]
  int stickL = PS4.LStickY();
  int stickR = PS4.RStickY();

  // Make UP positive on both sticks if requested
  if (INVERT_STICK) {
    stickL = -stickL;
    stickR = -stickR;
  }

  // Only two states per stick: UP -> MAX_US, DOWN -> MIN_US, else STOP
  int pulseR;
  int pulseL;

  // neutral tests must be "within" the deadzone, not equality
  bool L_neutral = (abs(stickL) <= DEADZONE);
  bool R_neutral = (abs(stickR) <= DEADZONE);

  if (!L_neutral && R_neutral && stickL > DEADZONE) {         // left up
    pulseL = MAX_US;     
    pulseR = NEUTRAL_US;
  }
  else if (!L_neutral && R_neutral && stickL < -DEADZONE) {   // left down
    pulseL = MIN_US;   
    pulseR = NEUTRAL_US;       
  }
  else if (!R_neutral && L_neutral && stickR > DEADZONE) {    // right up
    pulseR = MAX_US; 
    pulseL = NEUTRAL_US;      
  }
  else if (!R_neutral && L_neutral && stickR < -DEADZONE) {   // right down
    pulseR = MIN_US; 
    pulseL = NEUTRAL_US;      
  }
  else {
    pulseL = NEUTRAL_US;    // anything else → stop both
    pulseR = NEUTRAL_US;
  }

  // Debug print (both sides for clarity)
  Serial.printf("L: %4d -> %4d us | R: %4d -> %4d us\n", stickL, pulseL, stickR, pulseR);

  servoR.writeMicroseconds(pulseR);
  servoL.writeMicroseconds(pulseL);

  delay(10);
}