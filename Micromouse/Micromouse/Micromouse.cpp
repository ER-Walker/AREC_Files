#include <Arduino.h>
#include <Motoron.h>
#include <motoron_protocol.h>
#include <Encoder.h>
#include "Micromouse.h"

// =============================================================================
// Constructor
// =============================================================================
//
// Deep-copies every pin array onto the heap so the library owns its data.
// Encoder objects are also heap-allocated here.
//
// IMPROVE [MEMORY]: There is no destructor, so these allocations are never
//   freed.  For a single long-running embedded instance this doesn't matter,
//   but adding a destructor (or using fixed-size arrays) is better practice:
//
//     ~Micromouse() {
//       delete _encLeft;  delete _encRight;
//       for (int i = 0; i < _numUltrasonic; i++) delete[] _ultrasonicSensors[i];
//       delete[] _ultrasonicSensors;
//       delete[] _IRSensors;
//       for (int i = 0; i < _senseRows; i++) delete[] _senseArr[i];
//       delete[] _senseArr;
//     }
//

Micromouse::Micromouse(
  int numMotors,
  int numUltrasonic,    int ultrasonicSensors[][2],
  int numIR,            int IRSensors[],
  int enc[2][2],
  uint16_t maxAccel,    uint16_t maxDecel,
  uint16_t referenceMv, uint16_t minVinVoltageMv,
  float degPerCount,
  int lineSensitivityOffset,
  int encoderSensitivityOffset,
  float minDistance
)
{
  // Store scalar configuration
  _numMotors             = numMotors;
  _numUltrasonic         = numUltrasonic;
  _numIR                 = numIR;
  _maxAccel              = maxAccel;
  _maxDecel              = maxDecel;
  _referenceMv           = referenceMv;
  _minVinVoltageMv       = minVinVoltageMv;
  _degPerCount           = degPerCount;
  _lineSensitivityOffset = lineSensitivityOffset;
  _encoderSensitivityOffset = encoderSensitivityOffset;
  _minDistance           = minDistance;
  _motorSpeed            = 0;  // No movement until setMotorSpeed() is called

  // ── Encoders ──────────────────────────────────────────────────────────────
  // Heap-allocate two Encoder objects.  The Encoder library by Paul
  // Stoffregen uses pin-change interrupts internally; the pins passed here
  // MUST be interrupt-capable for full-resolution counting:
  //   Mega: 2, 3, 18, 19, 20, 21
  //   Uno:  2, 3  (only two available)
  // If non-interrupt pins are used, the library falls back to polling,
  // which misses counts at higher RPM.
  _encLeft  = new Encoder(enc[0][0], enc[0][1]);  // left  wheel
  _encRight = new Encoder(enc[1][0], enc[1][1]);  // right wheel

  // ── Ultrasonic sensor pins ────────────────────────────────────────────────
  // Deep-copy the 2-D array: each row is {trigPin, echoPin}.
  _ultrasonicSensors = new int*[numUltrasonic];
  for (int i = 0; i < numUltrasonic; i++)
  {
    _ultrasonicSensors[i]    = new int[2];
    _ultrasonicSensors[i][0] = ultrasonicSensors[i][0]; // trig
    _ultrasonicSensors[i][1] = ultrasonicSensors[i][1]; // echo
  }

  // ── IR sensor pins ────────────────────────────────────────────────────────
  // Flat array; index 0 is the baseline reference sensor that the others
  // are compared against in sensorChange().
  _IRSensors = new int[numIR];
  for (int i = 0; i < numIR; i++)
    _IRSensors[i] = IRSensors[i];

  // _senseArr is deferred to begin() because its row count depends on
  // max(_numUltrasonic, _numIR - 1), and we want begin() to be the
  // single initialization entry point.
  _senseArr  = nullptr;
  _senseRows = 0;
}

// =============================================================================
// Public methods
// =============================================================================

// ─── begin() ─────────────────────────────────────────────────────────────────
// Call once from setup().  Sequence:
//   1. Configure GPIO pin modes for all sensors.
//   2. Allocate the shared sensor-results matrix (_senseArr).
//   3. Initialize the Motoron controller over I2C.
//   4. Wait for any residual motor movement to coast to zero.
//
void Micromouse::begin()
{
  // ── Sensor pin modes ──────────────────────────────────────────────────────
  for (int i = 0; i < _numUltrasonic; i++)
  {
    pinMode(_ultrasonicSensors[i][0], OUTPUT); // trig pin drives the sensor
    pinMode(_ultrasonicSensors[i][1], INPUT);  // echo pin reads the return pulse
  }

  for (int i = 0; i < _numIR; i++)
  {
    pinMode(_IRSensors[i], INPUT);  // analog input (pinMode is optional for
                                    // analogRead, but explicit is clearer)
  }

  // ── Allocate sensor results matrix ────────────────────────────────────────
  // Rows = whichever sensor array is larger (ultrasonic count vs IR count
  // minus the baseline).  Columns = 2 (col 0: IR flag, col 1: ultrasonic).
  // Allocated once and reused every sensorChange() call.
  //
  // IMPROVE [INDEXING]: Because the IR loop starts at i=1 and writes to
  //   _senseArr[i][0], row 0 / col 0 is always zero.  Consider indexing
  //   from 0 and mapping IR sensor i+1 → row i, so no row is wasted.
  _senseRows = max(_numUltrasonic, _numIR - 1);
  _senseArr  = new short*[_senseRows];
  for (int i = 0; i < _senseRows; i++)
    _senseArr[i] = new short[2]{0, 0};

  // ── Motoron I2C initialization ────────────────────────────────────────────
  // reinitialize()  – Sends a full reinitialize command, resetting the
  //                   Motoron to a known state (all speeds 0, default config).
  // clearResetFlag() – Acknowledges the reset so the status register is clean.
  // setErrorResponse(COAST) – On error, motors coast (go slack) rather than
  //                           actively braking.  Safer for a small robot.
  // setErrorMask()   – Tells the Motoron which status flags constitute an
  //                    "error" that triggers the error response.
  _mc.reinitialize();
  _mc.clearResetFlag();
  _mc.setErrorResponse(MOTORON_ERROR_RESPONSE_COAST);
  _mc.setErrorMask(_errorMask);

  // Set per-motor acceleration and deceleration limits.
  // Motoron motor channels are 1-indexed.
  for (int i = 1; i <= _numMotors; i++)
  {
    _mc.setMaxAcceleration(i, _maxAccel);
    _mc.setMaxDeceleration(i, _maxDecel);
  }

  // Block until the Motoron reports that no motor is actively being driven.
  // This guards against stale commands from a previous reset.
  //
  // IMPROVE [SAFETY]: Add a millis() timeout here.  If the flag never
  //   clears (e.g., stuck motor), this loop hangs forever.
  while (_mc.getMotorDrivingFlag());

  _mc.clearMotorFault();
}

// ─── errorCheck() ────────────────────────────────────────────────────────────
// Wrapper around the private checkForProblems().  Intended to be called
// periodically in loop() or at the top of every movement command.
//
void Micromouse::errorCheck()
{
  checkForProblems();
}

// ─── setMotorSpeed() ─────────────────────────────────────────────────────────
// Stores the speed magnitude for subsequent travelDeg() / rotate() calls.
// Does NOT send a speed command to the Motoron immediately.
//
// IMPROVE [BUG]: The `motor` parameter is accepted but completely ignored.
//   A single _motorSpeed is used for every motor.  Two options:
//     (a) Remove the parameter:  void setMotorSpeed(uint16_t speed);
//     (b) Store per-motor speeds:
//           int _motorSpeeds[MAX_MOTORS];
//           void setMotorSpeed(uint8_t motor, uint16_t speed) {
//             _motorSpeeds[motor] = speed;
//           }
//
void Micromouse::setMotorSpeed(uint8_t motor, uint16_t speed)
{
  _motorSpeed = speed;
}

// ─── distance() ──────────────────────────────────────────────────────────────
// Triggers an HC-SR04 ultrasonic sensor and returns the measured distance.
//
// Hardware protocol:
//   1. Drive trigPin LOW for ≥2 µs (ensure clean start).
//   2. Drive trigPin HIGH for exactly 10 µs (trigger pulse).
//   3. Drive trigPin LOW.
//   4. Measure the HIGH-duration echo pulse in microseconds.
//
// Physics:
//   Speed of sound ≈ 343 m/s = 0.0343 cm/µs.
//   Round-trip → divide by 2.
//   Result = echoTime_µs × 0.0343 / 2   [centimeters]
//
//   Passing a timeout:  pulseIn(echoPin, HIGH, 250000)  (≈ 42.875 m max range before error)
//   
//   Future revision: Using the NewPing library for non-blocking readings
//
float Micromouse::distance(int sensorNum)
{
  int trigPin = _ultrasonicSensors[sensorNum][0];
  int echoPin = _ultrasonicSensors[sensorNum][1];

  // Trigger sequence
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo and convert to distance
  return (pulseIn(echoPin, HIGH, 45000) * 0.0343) / 2.0;  // centimeters
}

// ─── sensorChange() ─────────────────────────────────────────────────────────
// Scans all IR and ultrasonic sensors, populates _senseArr, and returns
// true if anything triggered.
//
// _senseArr layout after a call:
//   Row index  │  Col 0 (IR)                          │  Col 1 (Ultrasonic)
//   ───────────┼──────────────────────────────────────┼─────────────────────
//       0      │  always 0 (see note below)           │  sensor 0 trigger
//       1      │  IR sensor 1 deviates from baseline? │  sensor 1 trigger
//       2      │  IR sensor 2 deviates from baseline? │  sensor 2 trigger
//       …      │  …                                   │  …
//
// Note: IR index 0 is the baseline reference and is never compared against
// itself, so _senseArr[0][0] is always 0.
//
// IMPROVE [INDEXING]: Shift IR results down by one so row 0 holds IR
//   sensor 1's result.  This avoids the wasted row and makes the array
//   layout more intuitive.
//
// IMPROVE [PERF]: This function calls distance() for every ultrasonic
//   sensor, each of which blocks on pulseIn().  With 3 sensors and the
//   default 1 s timeout, worst case is ~3 s per call.  Since sensorChange()
//   is called inside the tight _drive() control loop, this starves the
//   encoder correction logic.  Options:
//     - Add a short pulseIn timeout (e.g., 15 ms ≈ 2.5 m max range).
//     - Only poll ultrasonics every Nth iteration of the drive loop.
//     - Use NewPing's non-blocking timer-based measurements.
//
bool Micromouse::sensorChange()
{
  // Reset every slot to 0 before scanning
  for (int i = 0; i < _senseRows; i++)
  {
    _senseArr[i][0] = 0;
    _senseArr[i][1] = 0;
  }

  bool triggered = false;

  // ── IR scan ───────────────────────────────────────────────────────────────
  // Read the baseline reference sensor (index 0) once, then compare every
  // other sensor against it.  If the absolute deviation exceeds the
  // threshold, mark that sensor's row.
  int baseline = analogRead(_IRSensors[0]);
  for (int i = 1; i < _numIR; i++)
  {
    if (abs(analogRead(_IRSensors[i]) - baseline) > _lineSensitivityOffset)
    {
      _senseArr[i][0] = 1;   // Flag: IR sensor i detected a line
      triggered = true;
    }
  }

  // ── Ultrasonic scan ───────────────────────────────────────────────────────
  // Check each sensor's measured distance against the obstacle threshold.
  for (int i = 0; i < _numUltrasonic; i++)
  {
    if (distance(i) < _minDistance)  // ← BUG: unit mismatch (cm vs m)
    {
      _senseArr[i][1] = 1;   // Flag: obstacle within threshold
      triggered = true;
    }
  }

  return triggered;
}

// ─── IRraw() ─────────────────────────────────────────────────────────────────
// Returns the raw 10-bit ADC reading (0–1023) from the specified IR sensor.
// Useful for calibration and debugging outside of the sensorChange() scan.
//
int Micromouse::IRraw(short index)
{
  return analogRead(_IRSensors[index]);
}

// ─── travelDeg() ─────────────────────────────────────────────────────────────
// Drives all motors in the same direction until the average encoder count
// (converted to degrees) reaches targetDeg, or a sensor fires.
//   Positive targetDeg → forward
//   Negative targetDeg → reverse
// Returns the actual degrees traveled (may be less if a sensor triggered).
//
double Micromouse::travelDeg(double targetDeg)
{
  return _drive(targetDeg, false);
}

// ─── rotate() ────────────────────────────────────────────────────────────────
// Rotates the robot in place by spinning odd-numbered motors one way and
// even-numbered motors the opposite way.
//   Positive targetDeg → clockwise  (by convention)
//   Negative targetDeg → counter-clockwise
// Returns the actual degrees rotated.
//
double Micromouse::rotate(double targetDeg)
{
  return _drive(targetDeg, true);
}

// =============================================================================
// Private helpers
// =============================================================================

// ─── _drive() ────────────────────────────────────────────────────────────────
// Shared movement engine for travelDeg() and rotate().
//
//  FLOW:
//    1. Determine direction sign from targetDeg.
//    2. Command all motors at _motorSpeed × sign.
//       • rotate mode: even motors get inverted sign.
//    3. Enter a polling loop:
//       a. Read both encoders.
//       b. If the difference exceeds the encoder sensitivity offset,
//          pause the LEADING side's motor to let the lagging side catch up
//          (bang-bang correction).
//       c. Compute average degrees traveled.
//       d. Break if target reached or sensorChange() triggers.
//    4. Stop all motors and zero encoders.
//
// IMPROVE [SAFETY]:  Add a millis()-based timeout:
//     unsigned long deadline = millis() + DRIVE_TIMEOUT_MS;
//     if (millis() > deadline) break;
//   Without this, a stuck encoder or unreachable target causes an
//   infinite loop.
//
// IMPROVE [SMOOTHNESS]:  The encoder correction is bang-bang: the leading
//   motor is either at full speed or completely stopped.  This produces
//   jerky oscillations (one side lurches, stops, the other catches up,
//   then the roles flip).  A proportional correction is much smoother:
//
//     int error = abs(rightVal) - abs(leftVal);  // positive = right ahead
//     int correction = constrain(error * Kp, 0, _motorSpeed / 2);
//     int speedRight = _motorSpeed - correction;  // slow the leader
//     int speedLeft  = _motorSpeed + correction;  // (or boost the lagger)
//
//   Or use a full PID controller for the encoder-count error.
//
// IMPROVE [CONSISTENCY]:  The encoders are read TWICE per iteration —
//   once for the correction check (rightVal, leftVal) and once for
//   currentDeg (_encRight->read(), _encLeft->read()).  Between those
//   reads the ISR can fire, so the values may differ.  Read once into
//   locals and reuse them:
//
//     long rv = _encRight->read();
//     long lv = _encLeft->read();
//     // use rv, lv for correction AND for currentDeg
//
double Micromouse::_drive(double targetDeg, bool isRotate)
{
  // Edge case: zero target → just make sure we're stopped
  if (targetDeg == 0)
  {
    _stopAndReset();
    return 0;
  }

  // Direction sign: +1 for positive target, -1 for negative
  short sign = (targetDeg > 0) - (targetDeg < 0);

  // ── Initial motor command ─────────────────────────────────────────────────
  // In rotate mode, even-numbered motors spin opposite to odd-numbered
  // motors, creating an in-place turn.
  for (int i = 1; i <= _numMotors; i++)
  {
    short dir = (isRotate && i % 2 == 0) ? -sign : sign;
    _mc.setSpeed(i, _motorSpeed * dir);
  }

  double currentDeg = 0;
  long rightVal, leftVal;

  // ── Main control loop ─────────────────────────────────────────────────────
  while (true)
  {
    // Read encoder counts (signed; sign depends on rotation direction)
    rightVal = _encRight->read();
    leftVal  = _encLeft->read();

    // ── Encoder correction (bang-bang) ───────────────────────────────────────
    // If the absolute counts differ by more than the sensitivity offset,
    // one wheel is ahead of the other.  Determine which, then STOP the
    // leading wheel's motor so the lagging wheel can catch up.
    //
    // Mapping:
    //   Motor 1 (odd)  = left  wheel → _encLeft
    //   Motor 2 (even) = right wheel → _encRight
    //
    // `enc` is true when left is AHEAD (abs(leftVal) > abs(rightVal)).
    // When enc == true:
    //   odd motors (left side) → speed = 0        (pause the leader)
    //   even motors (right side) → speed = normal  (let lagger catch up)
    // When enc == false (right ahead):
    //   odd motors (left side) → speed = normal
    //   even motors (right side) → speed = 0
    //
    if ((abs(rightVal) - abs(leftVal)) > _encoderSensitivityOffset)
    {
      bool enc = abs(leftVal) > abs(rightVal);  // true = left is leading

      for (int i = 1; i <= _numMotors; i++)
      {
        short dir = (isRotate && i % 2 == 0) ? -sign : sign;

        // Zero out the leading side's motor, keep the lagging side running
        //   enc==true  (left  leads): odd motors→0, even motors→normal
        //   enc==false (right leads): odd motors→normal, even motors→0
        dir = (enc == 1 ? (i % 2 == 0 ? dir : 0)
                        : (i % 2 != 0 ? dir : 0));

        _mc.setSpeed(i, _motorSpeed * dir);
      }
    }
    else
    {
      // Encoders are close enough — run both motors at full speed
      for (int i = 1; i <= _numMotors; i++)
      {
        short dir = (isRotate && i % 2 == 0) ? -sign : sign;
        _mc.setSpeed(i, _motorSpeed * dir);
      }
    }

    // ── Compute degrees traveled ────────────────────────────────────────────
    // Average both encoder counts, then convert to degrees.
    //
    // IMPROVE: This re-reads the encoders (ISR could have updated them
    //   since rightVal/leftVal were captured above).  Use the already-read
    //   values instead:
    //     currentDeg = ((rightVal + leftVal) / 2.0) * _degPerCount;
    //
    currentDeg = ((_encRight->read() + _encLeft->read()) / 2.0) * _degPerCount;

    // ── Exit conditions ─────────────────────────────────────────────────────
    // (a) Target reached: depends on direction sign.
    // (b) Sensor triggered: sensorChange() returns true if any IR or
    //     ultrasonic sensor fired.
    bool reachedTarget = (sign > 0) ? (currentDeg >= targetDeg)
                                    : (currentDeg <= targetDeg);

    if (reachedTarget || sensorChange()) break;
  }

  // Stop all motors and reset encoder counts to 0 for the next command
  _stopAndReset();
  return currentDeg;
}

// ─── _stopAndReset() ─────────────────────────────────────────────────────────
// Immediately commands all motors to speed 0 and zeroes both encoder counts.
// Called at the end of every movement command and on targetDeg == 0.
//
void Micromouse::_stopAndReset()
{
  for (int i = 1; i <= _numMotors; i++)
    _mc.setSpeed(i, 0);

  _encLeft->write(0);   // Reset left  encoder count to 0
  _encRight->write(0);  // Reset right encoder count to 0
}

// =============================================================================
// Error checking
// =============================================================================
//
// All three checkers follow the same pattern: if a problem is detected, reset
// the Motoron and enter an infinite loop printing the error to Serial.  This
// is a hard halt — the robot stops permanently until physically reset.
//
// IMPROVE [RECOVERY]: Instead of halting forever, consider:
//   - A retry counter with exponential backoff.
//   - A "safe mode" that coasts motors and blinks an LED.
//   - Returning an error code and letting the caller decide.

void Micromouse::checkCommunicationError(uint8_t errorCode)
{
  if (errorCode)
  {
    while (1)
    {
      _mc.reset();
      Serial.print(F("Communication error: "));
      Serial.println(errorCode);
      delay(1000);
    }
  }
}

void Micromouse::checkControllerError(uint16_t status)
{
  if (status & _errorMask)
  {
    while (1)
    {
      _mc.reset();
      Serial.print(F("Controller error: 0x"));
      Serial.println(status, HEX);
      delay(1000);
    }
  }
}

void Micromouse::checkVinVoltage(uint16_t voltageMv)
{
  if (voltageMv < _minVinVoltageMv)
  {
    while (1)
    {
      _mc.reset();
      Serial.print(F("VIN voltage too low: "));
      Serial.println(voltageMv);
      delay(1000);
    }
  }
}

// ─── checkForProblems() ──────────────────────────────────────────────────────
// Reads Motoron status, checks each error category in order:
//   1. I2C communication (getLastError after getStatusFlags)
//   2. Controller status flags (masked by _errorMask)
//   3. VIN voltage (measured via the Motoron's built-in divider)
//   4. I2C communication again (getLastError after getVinVoltageMv)
//
void Micromouse::checkForProblems()
{
  uint16_t status = _mc.getStatusFlags();
  checkCommunicationError(_mc.getLastError());  // Did the status read fail?
  checkControllerError(status);                  // Any fatal flags set?

  uint32_t voltageMv = _mc.getVinVoltageMv(_referenceMv, _vinType);
  checkCommunicationError(_mc.getLastError());  // Did the voltage read fail?
  checkVinVoltage(voltageMv);                    // Voltage within limits?
}
