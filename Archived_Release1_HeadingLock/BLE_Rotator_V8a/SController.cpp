#include "SController.h"
#include <Arduino.h>
#define DEBUG_BLE_EVENTS 1
// Pull from your sketch (unchanged)
extern bool  goToAngleActive;
extern float targetAngleDeg;
extern const float angleErrorThreshold;
extern const int   maxGotoSpeed;
extern int   currentCommand;

static inline int16_t sgn_i(int x) { return (x < 0) ? -1 : 1; }

void Controller::onEvent(Event e, float v, uint32_t t_ms) {

  #if DEBUG_BLE_EVENTS
    if (e == Event::PEDAL_MOVE) {
      Serial.print("[EVT] PEDAL_MOVE v="); Serial.print((int)v);
      Serial.print(" t_ms="); Serial.println(t_ms);
    }
  #endif  
  switch (e) {
    case Event::PEDAL_MOVE: {
      // Update packet timing on EVERY pedal RX (even zeros)
      if (t_ms) lastCommandTime_ = t_ms;

      // Treat as takeover only when magnitude beats threshold and BLE is up
      // (We use 5 here to match your PEDAL_TAKEOVER_THRESHOLD)
      if (bleOk_ && (abs((int)v) > 5)) {
        lastRealPedalTime_ = t_ms ? t_ms : lastCommandTime_;
        controlInhibit_    = false;
        // Request to drop any locks and return to MANUAL
        dropHLRequested_   = true;
        mode_              = Mode::MANUAL;
      }
    } break;

    case Event::BLE_RECOVER:
      bleOk_ = true;
      // remain inhibited until pedal or explicit cmd clears it
      break;

    case Event::BLE_LOSS:
      bleOk_         = false;
      controlInhibit_= true;
      forceStop_     = true;   // stop motor at next tick
      break;

    case Event::HL_SET_TARGET:
      pendingHLTarget_ = v;
      haveHLTarget_    = true; // will set + switch mode in tick()
      break;

    case Event::HL_OFF:
      dropHLRequested_ = true;
      mode_            = Mode::MANUAL;
      break;

    case Event::AUTOLOCK_ON:  autoLockEnabled_ = true;  break;
    case Event::AUTOLOCK_OFF: autoLockEnabled_ = false; break;

    case Event::GOTO_SET:
      // Keep GOTO inline for now (Step 3 can move it in)
      targetAngleDeg   = v;
      goToAngleActive  = true;
      break;

    case Event::GOTO_CANCEL:
      goToAngleActive  = false;
      break;

    case Event::GOTO_REACHED:
    case Event::TIMEOUT_2S:
    default:
      break;
  }
}

void Controller::tick100ms(uint32_t now, Magnetometer& mag, MotorController& motor, HeadingLock& headingLock) {
  // Honor forced stop (BLE loss) without blocking the app
  if (forceStop_) { motor.drive(0); forceStop_ = false; }

  // 2s timeout → inhibit; hold until pedal/command clears it
  if ((int32_t)(now - lastCommandTime_) > (int32_t)kTimeoutMs) {
    controlInhibit_ = true;
  }

  // Apply deferred "drop HL" from events
  if (dropHLRequested_) {
    headingLock.drop(motor);
    dropHLRequested_ = false;
  }

  // Apply deferred "set HL target" from events
  if (haveHLTarget_) {
    headingLock.setTarget(pendingHLTarget_);
    mode_          = Mode::HEADING_LOCK;
    haveHLTarget_  = false;
  }

  const bool ok = healthy(now);

  // --- Auto-arm (inside controller now) ---
  if (mode_ == Mode::MANUAL && autoLockEnabled_ && ok) {
    if ((int32_t)(now - lastRealPedalTime_) >= (int32_t)kAutoLockDelayMs) {
      if (headingLock.armAtCurrent(mag, motor)) {
        mode_ = Mode::HEADING_LOCK;
      }
    }
  }

  // === Driver dispatch (exactly one owner) ===

  // GOTO stays highest; keep your existing behavior/vars
  if (goToAngleActive) {
    mode_ = Mode::GOTO;
    if (ok) {
      float currentAngle = motor.getAngleDegCumulative();
      float error = targetAngleDeg - currentAngle;
      if (fabsf(error) <= angleErrorThreshold) {
        motor.drive(0);
        goToAngleActive = false;
        Serial.println("Reached target angle.");
      } else {
        const float kP = 14.0f;
        int16_t cmd = (int16_t)(kP * error);
        if (abs(cmd) < 80) cmd = sgn_i(cmd) * 80;
        cmd = constrain(cmd, -maxGotoSpeed, maxGotoSpeed);
        motor.drive(cmd);
      }
    } else {
      motor.drive(0);
    }
    return;
  }

  // HEADING_LOCK
  if (mode_ == Mode::HEADING_LOCK) {
    if (ok) {
      headingLock.tick(mag, motor);
    } else {
      motor.drive(0);
    }
    return;
  }

  // MANUAL
  mode_ = Mode::MANUAL;
  if (ok) {
    motor.drive(currentCommand);
  } else {
    motor.drive(0);
  }
}