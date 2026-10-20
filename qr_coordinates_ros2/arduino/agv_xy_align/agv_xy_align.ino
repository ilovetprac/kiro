/*
 * agv_xy_align.ino
 * ----------------
 * AGV female-connector X/Y alignment controller.
 *
 * Receives the ArUco marker offset (X, Y in millimeters) from the Raspberry Pi
 * over USB serial and drives TWO stepper motors (via an L293D / HW-130 motor
 * shield, Adafruit Motor Shield v1 clone) that turn two screw slides to move
 * the female connector toward the marker center (X -> 0, Y -> 0).
 *
 * Hardware
 *   - Arduino Uno (clone) + HW-130 L293D motor shield.
 *   - X stepper  -> shield motor ports M1 + M2  (AFMotor port 1)
 *   - Y stepper  -> shield motor ports M3 + M4  (AFMotor port 2)
 *   - NEMA 11, 1.8 deg/step = 200 steps/rev.
 *   - SEPARATE 24V supply to the shield's external power terminal. Remove the
 *     Vin jumper so motor voltage does NOT backfeed the Arduino.
 *
 * Convention (matches the ROS node: +X right, +Y up, origin = camera axis)
 *   - Marker too far RIGHT (+X)  -> move slide so X decreases ("move left").
 *   - Marker too HIGH   (+Y)     -> move slide so Y decreases ("move down").
 *   If a motor moves the wrong way, flip INVERT_X / INVERT_Y below.
 *
 * No end-stops: MANUALLY CENTRE both slides before power-on. Power-on position
 * is treated as (0,0); travel is clamped to +/- HALF_TRAVEL_MM.
 *
 * Serial protocol (from the Pi): one line per update, "X,Y\n" in mm, e.g.
 *     12.3,-4.8
 * Baud: 115200.
 *
 * Requires the Adafruit Motor Shield library v1 ("Adafruit Motor Shield
 * library", AFMotor.h). Install via Arduino IDE Library Manager.
 */

#include <AFMotor.h>

// ---------------- CONFIG — set these for your hardware ----------------

// Motor steps per revolution (NEMA 11, 1.8 deg) = 200.
const int   STEPS_PER_REV = 200;

// Screw lead = mm the slide travels per ONE full screw revolution.
// MEASURE IT: mark the slide, turn the screw exactly 10 turns, measure travel,
// divide by 10. Then STEPS_PER_MM = STEPS_PER_REV / SCREW_LEAD_MM.
// Example: lead 2 mm -> 200 / 2 = 100 steps/mm. (Placeholder below = 2 mm.)
const float SCREW_LEAD_MM = 2.0;
const float STEPS_PER_MM  = STEPS_PER_REV / SCREW_LEAD_MM;

// Alignment deadband: stop moving an axis once |offset| is within this (mm).
const float DEADBAND_MM = 2.0;

// Max travel from the power-on centre, each direction (mm). 150 mm stroke ->
// +/- 75 mm.
const float HALF_TRAVEL_MM = 75.0;

// Flip these if a motor drives the wrong way.
const bool INVERT_X = false;
const bool INVERT_Y = false;

// How many steps to nudge per control update (keeps moves small & responsive).
const int  MAX_STEPS_PER_UPDATE = 20;

// Stepper speed (RPM) and step style.
const int  MOTOR_RPM  = 30;
// FULL step = most torque; DOUBLE = more torque; INTERLEAVE = smoother/slower.
#define STEP_STYLE DOUBLE

// ---------------------------------------------------------------------

// X stepper on shield port 1 (M1/M2), Y on port 2 (M3/M4).
AF_Stepper motorX(STEPS_PER_REV, 1);
AF_Stepper motorY(STEPS_PER_REV, 2);

// Current slide position in steps from the power-on centre.
long posX = 0;
long posY = 0;

const long STEP_LIMIT = (long)(HALF_TRAVEL_MM * STEPS_PER_MM);

String rxBuf = "";

void setup() {
  Serial.begin(115200);
  motorX.setSpeed(MOTOR_RPM);
  motorY.setSpeed(MOTOR_RPM);
  Serial.println("agv_xy_align ready. Send 'X,Y' in mm. Centre slides first.");
}

// Move one axis toward a target offset of 0, by at most MAX_STEPS_PER_UPDATE.
// offset_mm > 0 means the marker is right/up; we move to reduce it.
void driveAxis(AF_Stepper &motor, float offset_mm, long &pos, bool invert) {
  if (fabs(offset_mm) <= DEADBAND_MM) return;   // close enough -> hold

  // Steps needed to cancel the current offset.
  long want = (long)(offset_mm * STEPS_PER_MM);
  // Limit per-update movement for smooth closed-loop behaviour.
  if (want >  MAX_STEPS_PER_UPDATE) want =  MAX_STEPS_PER_UPDATE;
  if (want < -MAX_STEPS_PER_UPDATE) want = -MAX_STEPS_PER_UPDATE;

  // Direction: reduce a positive offset. Optionally invert for wiring.
  long delta = -want;                 // move opposite to the offset
  if (invert) delta = -delta;

  // Clamp to the +/- travel limit (no end-stops).
  long target = pos + delta;
  if (target >  STEP_LIMIT) { delta = STEP_LIMIT - pos; }
  if (target < -STEP_LIMIT) { delta = -STEP_LIMIT - pos; }
  if (delta == 0) return;

  if (delta > 0) motor.step(delta, FORWARD, STEP_STYLE);
  else           motor.step(-delta, BACKWARD, STEP_STYLE);
  pos += delta;
}

void handleLine(const String &line) {
  int comma = line.indexOf(',');
  if (comma < 0) return;                       // malformed
  float x = line.substring(0, comma).toFloat();
  float y = line.substring(comma + 1).toFloat();
  driveAxis(motorX, x, posX, INVERT_X);
  driveAxis(motorY, y, posY, INVERT_Y);
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleLine(rxBuf);
      rxBuf = "";
    } else if (c != '\r') {
      rxBuf += c;
      if (rxBuf.length() > 40) rxBuf = "";     // overflow guard
    }
  }
}
