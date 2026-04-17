// =============================================================================
// Micromouse.ino — Main sketch for the AREC Micromouse robot
// =============================================================================
//
// This sketch configures hardware pins, sensor thresholds, and motion
// parameters, then instantiates the Micromouse library.  All robot logic
// (driving, turning, sensing, error handling) lives in the library;
// this file is purely configuration + the Arduino setup/loop entry points.
//

// ─── Encoder interrupt optimization ─────────────────────────────────────────
// ENCODER_OPTIMIZE_INTERRUPTS tells the Encoder library to use direct
// port-register reads inside the ISR instead of digitalRead(), which is
// significantly faster (important at high encoder tick rates).
//
// WARNING: This #define disables attachInterrupt() for the pins the
// Encoder library claims.  If you need attachInterrupt() on ANY of the
// encoder pins for something else, comment this line out.
#define ENCODER_OPTIMIZE_INTERRUPTS

#include "Micromouse.h"

// ─── Motor configuration ────────────────────────────────────────────────────
// Number of DC motor channels on the Motoron controller.
// Motoron channels are 1-indexed internally (motor 1, motor 2).
const int numMotors = 2;

// ─── Ultrasonic sensor pins ─────────────────────────────────────────────────
// Each row is {trigPin, echoPin} for one HC-SR04 module.
// Sensor 0 = front-left, sensor 1 = front-center, sensor 2 = front-right
// (adjust labels to match your physical wiring).
const int ultrasonicSensors[][2] = {
  {4, 5},   // sensor 0
  {6, 7},   // sensor 1
  {8, 9},   // sensor 2
};

// ─── IR sensor pins ─────────────────────────────────────────────────────────
// Index 0 is the BASELINE REFERENCE: it reads a "known neutral" surface
// and all other sensors are compared against it via analogRead() deviation.
// Remaining indices are detection sensors.
const int IRSensors[] = {
  A1,  // index 0 — baseline reference (center of chassis, always on neutral)
  A2,  // index 1 — left
  A3,  // index 2 — front
  A4,  // index 3 — right
};

// ─── Encoder pins ───────────────────────────────────────────────────────────
// Quadrature encoders: each row is {channelA, channelB}.
// Both pins in each pair should be interrupt-capable for full resolution:
//   Arduino Mega: 2, 3, 18, 19, 20, 21
//   Arduino Uno:  2, 3
const int enc[2][2] = {
  {2,  3},    // left  encoder
  {18, 19},   // right encoder
};

// ─── Motoron settings ───────────────────────────────────────────────────────
const uint16_t maxAccel        = 140;   // Acceleration limit (Motoron units)
const uint16_t maxDecel        = 800;   // Deceleration limit (higher = snappier stop)
const uint16_t referenceMv     = 3300;  // ADC reference voltage: 3300 for 3.3 V boards,
                                        //   5000 for 5 V boards
const uint16_t minVinVoltageMv = 5000;  // Minimum battery voltage before brownout halt

// ─── Motion & sensor thresholds ─────────────────────────────────────────────
const float degPerCount           = 0.19924;  // Degrees of wheel rotation per encoder tick.
                                              // Derived from: 360° / (encoder CPR × gear ratio)

const int   lineSensitivityOffset = 100;      // Minimum analogRead() deviation from the
                                              // baseline IR sensor to register a line.

const int   encoderSensitivityOffset = 25;    // Maximum allowed absolute difference between
                                              // left and right encoder counts before the
                                              // bang-bang correction activates.

const float minDistance           = 2.5;    // Obstacle detection threshold.
                                              // Intended as 2.5 cm

// ─── Instantiate the library ────────────────────────────────────────────────
// The SENSOR_COUNT() macro uses sizeof to automatically count the number
// of rows in a stack-allocated array.  It only works at the declaration
// site (here in the .ino), not through a pointer.
Micromouse micromouse(
  numMotors,
  SENSOR_COUNT(ultrasonicSensors), ultrasonicSensors,
  SENSOR_COUNT(IRSensors),         IRSensors,
  enc,
  maxAccel, maxDecel,
  referenceMv, minVinVoltageMv,
  degPerCount,
  lineSensitivityOffset,
  encoderSensitivityOffset,
  minDistance
);

// =============================================================================
// Arduino entry points
// =============================================================================

void setup()
{
  Serial.begin(9600);   // Serial monitor for error diagnostics
  Wire.begin();         // Initialize I2C bus (required before Motoron comms)
  micromouse.begin();   // Initialize sensors, Motoron, and wait for motors to stop
}

void loop()
{
  // ── Your maze-solving / line-following logic goes here ─────────────────
  //
  // Example usage:
  //
  //   micromouse.errorCheck();                  // Halt if hardware fault
  //   micromouse.setMotorSpeed(1, 200);         // Set speed for next move
  //   double traveled = micromouse.travelDeg(360);  // Drive forward 1 wheel rev
  //   micromouse.rotate(90);                    // Turn 90° clockwise
  //
  //   if (micromouse.sensorChange()) {
  //     // React to IR line detection or ultrasonic obstacle
  //   }
  //
  //   float frontDist = micromouse.distance(1); // Read center ultrasonic (cm!)
  //   int rawIR = micromouse.IRraw(2);          // Read left IR raw value
  //
}
