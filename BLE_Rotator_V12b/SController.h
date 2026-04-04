#pragma once
#include <Arduino.h>
#include "config.h"           // ← added
#include "Magnetometer.h"
#include "MotorController.h"
#include "HeadingLock.h"
#include "TargetLock.h"
#include "GPSManager.h"

//#define DEBUG_BLE_EVENTS 1

class Controller {
public:
  enum class Mode  : uint8_t { MANUAL, HEADING_LOCK, TARGET_LOCK };
  enum class Event : uint8_t {
    PEDAL_MOVE,
    BLE_RECOVER, BLE_LOSS,
    HL_SET_TARGET, HL_OFF,
    AUTOLOCK_ON, AUTOLOCK_OFF,
    TIMEOUT_2S // internal/synthesized
  };

  // Call once in setup(), after cfg is populated
  void init(const Config& cfg) {
    kTimeoutMs_    = cfg.ctrlTimeoutMs;
    kLinkRecentMs_ = cfg.ctrlLinkRecentMs;
    kAutoLockMs_   = cfg.ctrlAutoLockMs;
    takeoverThr_   = cfg.ctrlTakeoverThr;
  }

  // Emit events from your callbacks / parser:
  void onEvent(Event e, float v = 0.0f, uint32_t t_ms = 0);

  // 100 ms policy + dispatch (computes link_ok & healthy internally)
  void tick100ms(uint32_t now,
                 Magnetometer& mag,
                 MotorController& motor,
                 HeadingLock& headingLock,
                 GPSManager& gps,
                 TargetLock& targetLock);

  Mode mode() const { return mode_; }
  bool inhibited() const { return controlInhibit_; }
  bool bleOk() const { return bleOk_; }

private:
  // --- Tunables (driven from cfg.init(), fallback to config.h defaults) ---
  uint32_t kLinkRecentMs_  = CFG_CTRL_LINK_RECENT_MS;
  uint32_t kTimeoutMs_     = CFG_CTRL_TIMEOUT_MS;
  uint32_t kAutoLockMs_    = CFG_CTRL_AUTOLOCK_MS;
  int      takeoverThr_    = CFG_CTRL_TAKEOVER_THR;

  // --- State we own now ---
  Mode     mode_             = Mode::MANUAL;
  bool     controlInhibit_   = false;
  bool     autoLockEnabled_  = true;
  bool     bleOk_            = false;
  uint32_t lastCommandTime_  = 0;
  uint32_t lastRealPedalTime_= 0;

  // Deferred actions we can't do in onEvent (need motor/HL refs)
  bool     dropHLRequested_  = false;
  bool     forceStop_        = false;
  bool     haveHLTarget_     = false;
  float    pendingHLTarget_  = 0.0f;

  // Helpers
  inline bool healthy(uint32_t now) const {
    const bool link_ok = bleOk_ && ((int32_t)(now - lastCommandTime_) < (int32_t)kLinkRecentMs_);
    return link_ok && !controlInhibit_;
  }
};
