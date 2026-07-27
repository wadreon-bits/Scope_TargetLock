#pragma once
#include <Arduino.h>
#include "Magnetometer.h"
#include "MotorController.h"
#include "HeadingLock.h"

class Controller {
public:
  enum class Mode  : uint8_t { MANUAL, HEADING_LOCK, GOTO };
  enum class Event : uint8_t {
    PEDAL_MOVE,
    BLE_RECOVER, BLE_LOSS,
    HL_SET_TARGET, HL_OFF,
    AUTOLOCK_ON, AUTOLOCK_OFF,
    GOTO_SET, GOTO_CANCEL, GOTO_REACHED,
    TIMEOUT_2S // internal/synthesized
  };

  // Emit events from your callbacks / parser:
  void onEvent(Event e, float v = 0.0f, uint32_t t_ms = 0);

  // 100 ms policy + dispatch (computes link_ok & healthy internally)
  void tick100ms(uint32_t now, Magnetometer& mag, MotorController& motor, HeadingLock& headingLock);

  Mode mode() const { return mode_; }
  bool inhibited() const { return controlInhibit_; }
  bool bleOk() const { return bleOk_; }

private:
  // --- Tunables (kept local to avoid cross-TU macro issues) ---
  static constexpr uint32_t kLinkRecentMs     = 500;   // packets considered "recent"
  static constexpr uint32_t kTimeoutMs        = 2000;  // link timeout -> inhibit
  static constexpr uint32_t kAutoLockDelayMs  = 1000;  // idle → auto-arm

  // --- State we own now ---
  Mode     mode_             = Mode::MANUAL;
  bool     controlInhibit_   = false;
  bool     autoLockEnabled_  = true;
  bool     bleOk_            = false;
  uint32_t lastCommandTime_  = 0;   // updated on every pedal RX
  uint32_t lastRealPedalTime_= 0;   // updated only when |cmd|>threshold

  // Deferred actions we can't do in onEvent (need motor/HL refs)
  bool     dropHLRequested_  = false;
  bool     forceStop_        = false;
  bool     haveHLTarget_     = false;
  float    pendingHLTarget_  = 0.0f;

  // Helpers
  inline bool healthy(uint32_t now) const {
    const bool link_ok = bleOk_ && ((int32_t)(now - lastCommandTime_) < (int32_t)kLinkRecentMs);
    return link_ok && !controlInhibit_;
  }
};