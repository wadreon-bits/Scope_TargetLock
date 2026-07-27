#include "SController.h"
#include <Arduino.h>

// Pull from your sketch (unchanged)
extern int currentCommand;

void Controller::onEvent(Event e, float v, uint32_t t_ms) {

  #if DEBUG_BLE_EVENTS
    if (e == Event::PEDAL_MOVE) {
      Serial.print("[EVT] PEDAL_MOVE v="); Serial.print((int)v);
      Serial.print(" t_ms="); Serial.println(t_ms);
    }
  #endif

  switch (e) {
    case Event::PEDAL_MOVE: {
      if (t_ms) lastCommandTime_ = t_ms;

      if (bleOk_ && (abs((int)v) > takeoverThr_)) {
        lastRealPedalTime_ = t_ms ? t_ms : lastCommandTime_;
        controlInhibit_    = false;
        dropHLRequested_   = true;
        mode_              = Mode::MANUAL;
      }
    } break;

    case Event::BLE_RECOVER:
      bleOk_ = true;
      break;

    case Event::BLE_LOSS:
      bleOk_          = false;
      controlInhibit_ = true;
      forceStop_      = true;
      break;

    case Event::HL_SET_TARGET:
      pendingHLTarget_ = v;
      haveHLTarget_    = true;
      break;

    case Event::HL_OFF:
      dropHLRequested_ = true;
      mode_            = Mode::MANUAL;
      break;

    case Event::AUTOLOCK_ON:  autoLockEnabled_ = true;  break;
    case Event::AUTOLOCK_OFF: autoLockEnabled_ = false; break;

    case Event::TIMEOUT_2S:
    default:
      break;
  }
}

void Controller::tick100ms(uint32_t now,
                           Magnetometer& mag,
                           MotorController& motor,
                           HeadingLock& headingLock,
                           GPSManager& gps,
                           TargetLock& targetLock) {
  if (forceStop_) {
    motor.drive(0);
    forceStop_ = false;
  }

  if ((int32_t)(now - lastCommandTime_) > (int32_t)kTimeoutMs_) {
    controlInhibit_ = true;
  }

  if (dropHLRequested_) {
    headingLock.drop(motor);
    dropHLRequested_ = false;
  }

  if (haveHLTarget_) {
    headingLock.setTarget(pendingHLTarget_);
    mode_         = Mode::HEADING_LOCK;
    haveHLTarget_ = false;
  }

  const bool ok       = healthy(now);
  const bool magReady = headingLock.sensorsReady(mag, motor);

  if (!magReady) {
    if (mode_ != Mode::MANUAL) {
      headingLock.drop(motor);
      targetLock.drop();
      mode_ = Mode::MANUAL;
    }
  }

  // --- Auto-arm ---
  if (mode_ == Mode::MANUAL && autoLockEnabled_ && ok && magReady) {
    if ((int32_t)(now - lastRealPedalTime_) >= (int32_t)kAutoLockMs_) {
      // Pass now so armAtCurrent timestamps are consistent with the tick
      if (targetLock.armAtCurrent(now, gps, mag, motor, headingLock)) {
        mode_ = Mode::TARGET_LOCK;
      } else {
        // TL failed (no GPS, or RotVecAvg not ready).
        // Try to establish compass reference for HL — armCompassRef() is a no-op
        // if RotVecAvg is not yet valid, in which case HL arms in game-frame fallback.
        mag.armCompassRef();
        if (headingLock.armAtCurrent(mag, motor)) {
          mode_ = Mode::HEADING_LOCK;
        }
      }
    }
  }

  // TARGET_LOCK
  if (mode_ == Mode::TARGET_LOCK) {
    if (ok && magReady) {
      targetLock.tick(now, gps, mag, motor, headingLock);
      headingLock.tick(mag, motor);

      // Check if TargetLock has dropped itself (GPS timeout or max-distance cancel).
      // State::IDLE means TL gave up — fall back gracefully.
      // State::GPS_LOST means TL is holding — stay in TARGET_LOCK mode so the
      // BNO-only HeadingLock keeps compensating for boat rotation.
      if (targetLock.state() == TargetLock::State::IDLE) {
        // TL dropped — fall to HL if it is still active, otherwise manual
        if (headingLock.active()) {
          mode_ = Mode::HEADING_LOCK;
        } else {
          mode_ = Mode::MANUAL;
        }
      }
      // GPS_LOST: do nothing here — TL manages its own hold timer and will
      // self-transition to IDLE if the timeout expires, caught next tick.
    } else {
      motor.drive(0);
    }
    return;
  }

  // HEADING_LOCK
  if (mode_ == Mode::HEADING_LOCK) {
    if (ok && magReady) {
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
