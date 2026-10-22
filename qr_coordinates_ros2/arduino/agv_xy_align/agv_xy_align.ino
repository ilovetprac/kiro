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
// This rig: 1 mm lead -> STEPS_PER_MM = 200 / 1 = 200 steps/mm.
// If you ever change the screw/motor, re-measure: mark the slide, turn the
// screw exactly 10 turns, measure travel, divide by 10, update this value.
const float SCREW_LEAD_MM = 1.0;
const float STEPS_PER_MM  = STEPS_PER_REV / SCREW_LEAD_MM;  // = 200 steps/mm

// Alignment deadband: stop moving an axis once |offset| is within this (mm).
// 1.0 mm = the required precision.
const float DEADBAND_MM = 1.0;

// Two-speed control for "fast but precise":
//   - When |offset| > FINE_ZONE_MM  -> move FAST (FAST_RPM, big steps).
//   - When |offset| <= FINE_ZONE_MM -> move SLOW (FINE_RPM, tiny steps) so it
//     settles into the 1 mm deadband without overshooting.
const float FINE_ZONE_MM = 5.0;   // distance from target to switch to fine mode
const int   FAST_RPM     = 75;    // coarse approach speed (lower if it stalls)
const int   FINE_RPM     = 25;    // precise approach speed near target
const int   FAST_MAX_STEPS = 200; // max steps/update in fast mode (1.0 mm)
const int   FINE_MAX_STEPS = 10;  // max steps/update in fine mode (0.05 mm)

// Max travel from the power-on centre, each direction (mm). 150 mm stroke ->
// +/- 75 mm.
const float HALF_TRAVEL_MM = 75.0;

// Flip these if a motor drives the wrong way.
// Set TRUE here because the camera + slides are mounted TOGETHER on the AGV
// (camera-on-mover). To cancel an offset, the assembly must move TOWARD the
// marker, which is the opposite of the fixed-camera default. If the bench test
// shows an axis still moving the wrong way, toggle that axis back to false.
// Set FALSE per bench test: with both true the slide drove AWAY from the
// marker (+X moved left, +Y moved down). False makes the assembly chase the
// marker (+X -> move right/toward it, +Y -> up/toward it) so the offset shrinks.
const bool INVERT_X = true;
const bool INVERT_Y = true;

// DEBUG: echo every received "X,Y" back over serial so you can verify the
// Pi -> Arduino link in the Serial Monitor / `arduino-cli monitor`.
const bool DEBUG_ECHO = true;

// SAFETY: set false to TEST THE BRIDGE WITH NO MOTOR MOTION. The Arduino still
// receives and echoes X,Y but does not drive the steppers. Set true only once
// the link is verified AND the slides are centred with 24V connected.
const bool MOTORS_ENABLED = true;

// Step style. FULL = most torque; DOUBLE = more torque; INTERLEAVE = smoother.
// The L293D shield steps in software (no acceleration), so too high an RPM
// makes the stepper STALL/skip instead of going faster (which ruins precision
// via lost steps). If FAST_RPM stalls, lower it until motion is reliable.
#define STEP_STYLE DOUBLE

// ALIGNMENT LATCH: once both X and Y stay within the deadband for this many
// consecutive updates, declare ALIGNED and STOP driving the motors (hold).
const int   ALIGN_CONFIRM = 5;
// Re-arm (resume aligning) only if the marker drifts back out beyond this many
// mm on either axis — prevents motor twitching on small post-align noise.
const float REARM_MM = 6.0;
// Release the motor coils when aligned, so they stop drawing current and the
// L293D cools. Set false to keep the coils energised (holding torque) if the
// slide tends to drift. (Leadscrews are usually self-locking, so release is OK.)
const bool  RELEASE_WHEN_ALIGNED = true;

// ---------------------------------------------------------------------

// X stepper on shield port 1 (M1/M2), Y on port 2 (M3/M4).
AF_Stepper motorX(STEPS_PER_REV, 1);
AF_Stepper motorY(STEPS_PER_REV, 2);

// Current slide position in steps from the power-on centre.
long posX = 0;
long posY = 0;

// LATEST target offset (mm) from the most recent serial reading. The main loop
// continuously chases THIS value, so a newer reading immediately redirects the
// motion — any previously-planned move is abandoned ("follow the latest").
volatile float latestX = 0.0;
volatile float latestY = 0.0;
bool haveTarget = false;   // a valid reading has been received at least once

// Alignment-latch state.
bool aligned = false;      // true once centred -> motors held/stopped
int  inBandCount = 0;      // consecutive updates with both axes in deadband

const long STEP_LIMIT = (long)(HALF_TRAVEL_MM * STEPS_PER_MM);

String rxBuf = "";

void setup() {
  Serial.begin(115200);
  motorX.setSpeed(FAST_RPM);
  motorY.setSpeed(FAST_RPM);
  Serial.println("agv_xy_align ready. Send 'X,Y' in mm. Centre slides first.");
}

// Take ONE small increment toward cancelling offset_mm on this axis. Called
// repeatedly from loop(). Small, non-blocking moves mean serial is re-read
// between increments, so a new target instantly redirects the motion (the old
// planned move is abandoned). Returns true if this axis still needs to move.
bool chaseAxis(AF_Stepper &motor, float offset_mm, long &pos, bool invert) {
  if (fabs(offset_mm) <= DEADBAND_MM) return false;   // within 1 mm -> hold

  // FAST when far, FINE when close (for speed + a precise, no-overshoot finish).
  bool fine = (fabs(offset_mm) <= FINE_ZONE_MM);
  motor.setSpeed(fine ? FINE_RPM : FAST_RPM);
  int cap = fine ? FINE_MAX_STEPS : FAST_MAX_STEPS;

  // Steps to cancel the offset, capped to a small increment per call.
  long want = (long)(offset_mm * STEPS_PER_MM);
  if (want >  cap) want =  cap;
  if (want < -cap) want = -cap;

  long delta = -want;                 // move opposite to the offset
  if (invert) delta = -delta;

  // Clamp to the +/- travel limit (no end-stops).
  long target = pos + delta;
  if (target >  STEP_LIMIT) delta = STEP_LIMIT - pos;
  if (target < -STEP_LIMIT) delta = -STEP_LIMIT - pos;
  if (delta == 0) return false;

  if (delta > 0) motor.step(delta, FORWARD, STEP_STYLE);
  else           motor.step(-delta, BACKWARD, STEP_STYLE);
  pos += delta;
  return true;
}

// Serial line -> just UPDATE the latest target + latch state. No stepping here,
// so the newest reading always wins and the loop chases the latest value.
void handleLine(const String &line) {
  // STOP command: marker lost -> halt motors immediately and drop the target.
  if (line.length() > 0 && (line[0] == 'S' || line[0] == 's')) {
    haveTarget = false;                        // loop() won't step
    inBandCount = 0;
    motorX.release();                          // stop at once (coils off)
    motorY.release();
    if (DEBUG_ECHO) Serial.println("STOP: marker lost, motors halted.");
    return;
  }

  int comma = line.indexOf(',');
  if (comma < 0) return;                       // malformed
  float x = line.substring(0, comma).toFloat();
  float y = line.substring(comma + 1).toFloat();

  latestX = x;                                 // newest target (old plan dropped)
  latestY = y;
  haveTarget = true;                           // marker present -> loop() chases

  bool inBand = (fabs(x) <= DEADBAND_MM && fabs(y) <= DEADBAND_MM);

  // --- Alignment latch state machine ---
  if (aligned) {
    if (fabs(x) > REARM_MM || fabs(y) > REARM_MM) {
      aligned = false;
      inBandCount = 0;
      Serial.println("REARM: marker moved, resuming alignment.");
    }
  } else {
    if (inBand) {
      inBandCount++;
      if (inBandCount >= ALIGN_CONFIRM) {
        aligned = true;
        if (RELEASE_WHEN_ALIGNED) { motorX.release(); motorY.release(); }
        Serial.println("ALIGNED: target centred, motors stopped.");
      }
    } else {
      inBandCount = 0;
    }
  }

  if (DEBUG_ECHO) {
    Serial.print("got X=");  Serial.print(x, 1);
    Serial.print(" Y=");     Serial.print(y, 1);
    if (!MOTORS_ENABLED)   Serial.print("  [motors OFF]");
    else if (aligned)      Serial.print("  [ALIGNED - stopped]");
    else                   Serial.print("  [chasing]");
    Serial.println();
  }
}

void loop() {
  // 1) Drain serial so the LATEST target is always current before we step.
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

  // 2) Take one small increment toward the LATEST target (unless held).
  if (MOTORS_ENABLED && haveTarget && !aligned) {
    chaseAxis(motorX, latestX, posX, INVERT_X);
    chaseAxis(motorY, latestY, posY, INVERT_Y);
  }
}
