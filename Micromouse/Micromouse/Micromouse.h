/*
  Micromouse.h - Library for the AREC Micromouse project.
  Created by Alex C. Chovan, April 9, 2026.

  ─────────────────────────────────────────────────────────────────────────────
  OVERVIEW
  ─────────────────────────────────────────────────────────────────────────────
  This library encapsulates the three core subsystems of a Micromouse robot:

    1. MOTION CONTROL
       Drives two DC motors through a Pololu Motoron I2C motor controller.
       Movement is commanded in encoder-degrees (travelDeg for linear motion,
       rotate for in-place turns).  A bang-bang encoder-matching loop attempts
       to keep both wheels turning the same amount.

    2. SENSING
       Two sensor arrays feed a unified "did anything change?" scan:
         - IR reflectance sensors for line-following (one sensor is a baseline
           reference; the rest are compared against it).
         - HC-SR04 ultrasonic rangefinders for obstacle detection.
       Any triggered sensor can interrupt a movement command mid-drive.

    3. ERROR MONITORING
       Polls the Motoron for communication faults, controller errors, and
       input-voltage brownout.  Halts execution with serial diagnostics if
       any are detected.

  ─────────────────────────────────────────────────────────────────────────────
  KNOWN ISSUES / IMPROVEMENT NOTES  (search "IMPROVE" in source)
  ─────────────────────────────────────────────────────────────────────────────
  [BUG]     setMotorSpeed() accepts a motor number but ignores it — only one
            speed is stored for all motors.
  [SAFETY]  _drive() has no timeout — the robot runs forever if the target
            is unreachable and no sensor fires.
  [PERF]    sensorChange() calls blocking pulseIn() (up to 1 s each) inside
            the tight control loop, starving encoder correction.
  [SMOOTH]  Encoder correction is bang-bang (full speed ↔ zero).  A
            proportional correction would dramatically reduce jerk.
  [STYLE]   No destructor — heap allocations are never freed.
  ─────────────────────────────────────────────────────────────────────────────
*/

#ifndef Micromouse_h
#define Micromouse_h

#include <Arduino.h>
#include <Motoron.h>
#include <motoron_protocol.h>

// Forward-declare Encoder so we don't pull the full header into every
// translation unit that includes Micromouse.h.  The actual #include
// lives in Micromouse.cpp.
class Encoder;

// ─── Helper macro ────────────────────────────────────────────────────────────
// Pass a stack-allocated 2-D or 1-D array declared in the SAME scope and
// this macro computes the number of rows at compile time via sizeof.
// It ONLY works at the declaration site — not through a pointer.
#define SENSOR_COUNT(arr) (sizeof(arr) / sizeof(arr[0]))

// ─── Motoron error mask ──────────────────────────────────────────────────────
// Bitfield of Motoron status flags that should be treated as fatal errors.
// If any of these bits are set in the status register, errorCheck() halts.
#define ERROR_MASK \
    (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR)          | \
    (1 << MOTORON_STATUS_FLAG_CRC_ERROR)               | \
    (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED) | \
    (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED)     | \
    (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED)        | \
    (1 << MOTORON_STATUS_FLAG_UART_ERROR)              | \
    (1 << MOTORON_STATUS_FLAG_RESET)                   | \
    (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT)

// =============================================================================
// Micromouse class
// =============================================================================
class Micromouse
{
  public:
    // ─── Constructor ─────────────────────────────────────────────────────────
    //
    //  numMotors             Number of motors on the Motoron (typically 2).
    //  numUltrasonic         Number of HC-SR04 ultrasonic sensors.
    //  ultrasonicSensors     2-D array [sensor][0=trigPin, 1=echoPin].
    //  numIR                 Total number of IR sensors (including baseline).
    //  IRSensors             1-D array of analog pins; index 0 = baseline ref.
    //  enc                   Encoder pins: enc[0]={leftA,leftB},
    //                                      enc[1]={rightA,rightB}.
    //  maxAccel / maxDecel   Motoron accel/decel limits (Motoron internal units).
    //  referenceMv           ADC reference voltage in mV (5000 or 3300).
    //  minVinVoltageMv       Minimum VIN before brownout error.
    //  degPerCount           Degrees of wheel rotation per single encoder tick.
    //  lineSensitivityOffset IR analog deviation from baseline to flag a line.
    //  encoderSensitivityOffset  Max allowed encoder-count difference before
    //                            the correction logic kicks in.
    //  minDistance            Obstacle threshold.
    //                        *** BUG: currently interpreted as meters in the
    //                        .ino (0.025 = 2.5 cm) but distance() returns
    //                        centimeters.  See IMPROVE note in .cpp. ***
    //
    Micromouse(
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
    );

    // ─── Initialization ──────────────────────────────────────────────────────
    // Call once in setup().  Sets pin modes, allocates internal arrays,
    // configures the Motoron, and waits for motors to coast to a stop.
    void begin();

    // ─── Diagnostics ─────────────────────────────────────────────────────────
    // Checks communication, controller status, and VIN voltage.
    // If any problem is found, enters an infinite loop printing the error
    // to Serial — effectively a hard halt.
    void errorCheck();

    // ─── Motor control ───────────────────────────────────────────────────────
    // Stores the speed used by travelDeg() and rotate().
    // IMPROVE: the `motor` parameter is accepted but ignored — a single
    //          _motorSpeed is stored for ALL motors.  Either remove the
    //          parameter or maintain a per-motor speed array.
    void setMotorSpeed(uint8_t motor, uint16_t speed);

    // Drive forward (positive) or backward (negative) until the average
    // encoder reading reaches targetDeg, or a sensor triggers.
    // Returns the actual degrees traveled.
    double travelDeg(double targetDeg);

    // Rotate in place: odd-numbered motors spin one way, even-numbered
    // motors spin the opposite way.  Positive = clockwise (by convention),
    // negative = counter-clockwise.  Returns degrees actually rotated.
    double rotate(double targetDeg);

    // ─── Sensors ─────────────────────────────────────────────────────────────
    // Returns the distance measured by ultrasonic sensor #sensorNum.
    // *** Currently returns CENTIMETERS despite comments saying meters. ***
    float distance(int sensorNum);

    // Scans all sensors and populates _senseArr:
    //   _senseArr[i][0] = 1 if IR sensor i+1 deviates from baseline
    //   _senseArr[i][1] = 1 if ultrasonic sensor i is within minDistance
    // Returns true if at least one sensor triggered.
    bool sensorChange();

    // Returns the raw 10-bit ADC value from the IR sensor at `index`.
    int IRraw(short index);

  private:
    // ─── Configuration ───────────────────────────────────────────────────────
    int   _numMotors;              // Number of Motoron channels in use
    int   _numUltrasonic;          // Number of ultrasonic sensors
    int** _ultrasonicSensors;      // Heap copy: [sensor][0=trig, 1=echo]
    int   _numIR;                  // Number of IR sensors (including baseline)
    int*  _IRSensors;              // Heap copy: [pin]; index 0 = baseline

    int   _motorSpeed;             // Shared speed magnitude for all motors
    float _degPerCount;            // Encoder ticks → degrees conversion factor
    int   _lineSensitivityOffset;  // IR deviation threshold (analogRead units)
    int   _encoderSensitivityOffset; // Max encoder-count mismatch before correction
    float _minDistance;            // Ultrasonic obstacle threshold

    // ─── Sensor results buffer ───────────────────────────────────────────────
    // Allocated once in begin().  Dimensions:
    //   rows = max(numUltrasonic, numIR - 1)
    //   cols = 2  (col 0 = IR trigger flag, col 1 = ultrasonic trigger flag)
    short** _senseArr;
    int     _senseRows;

    // ─── Encoders ────────────────────────────────────────────────────────────
    Encoder*   _encLeft;           // Left wheel quadrature encoder
    Encoder*   _encRight;          // Right wheel quadrature encoder

    // ─── Motoron ─────────────────────────────────────────────────────────────
    MotoronI2C _mc;                // I2C handle to the Motoron controller
    uint16_t   _referenceMv;       // ADC Vref for VIN measurement
    uint16_t   _minVinVoltageMv;   // Brownout threshold
    uint16_t   _maxAccel;          // Per-motor acceleration limit
    uint16_t   _maxDecel;          // Per-motor deceleration limit

    // Motoron256 VIN sense resistor divider type.
    // Change _referenceMv to 3300 for 3.3 V boards (already done in .ino).
    const static auto _vinType = MotoronVinSenseType::Motoron256;

    // Precomputed error mask from the ERROR_MASK macro
    const uint16_t _errorMask = ERROR_MASK;

    // ─── Internal helpers ────────────────────────────────────────────────────

    // Shared movement engine behind travelDeg() and rotate().
    // isRotate == true → even-numbered motors run opposite to odd-numbered.
    // Returns degrees traveled when target is met or sensor fires.
    // IMPROVE: add a millis()-based timeout to prevent infinite loops.
    double _drive(double targetDeg, bool isRotate);

    // Immediately sets all motor speeds to 0 and zeroes both encoder counts.
    void _stopAndReset();

    // Error-checking sub-functions called by checkForProblems().
    // Each enters an infinite serial-printing loop on failure.
    void checkCommunicationError(uint8_t errorCode);
    void checkControllerError(uint16_t status);
    void checkVinVoltage(uint16_t voltageMv);

    // Top-level error aggregator called by errorCheck().
    void checkForProblems();
};

#endif
