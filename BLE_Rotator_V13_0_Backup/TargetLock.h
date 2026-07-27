#pragma once

#include <Arduino.h>
#include <math.h>
#include "GPSManager.h"
#include "Magnetometer.h"
#include "MotorController.h"
#include "HeadingLock.h"

class TargetLock {
public:
  struct Params {
    float    distance_m;          // projection distance at arm time (metres)
    float    maxHAcc_m;           // max GPS horizontal accuracy to allow arming (metres)
    uint32_t maxStale_ms;         // max GPS data age before considering fix stale (ms)
    float    minDrift_m;          // min boat displacement before updating bearing (metres)
    float    bearingAlpha;        // low-pass filter coefficient for bearing delta (0 < x <= 1)
    float    maxDistToTarget_m;   // cancel TL if boat is farther than this from target (metres)
    uint32_t gpsLostTimeoutMs;    // how long to hold TL after GPS lost before dropping (ms)

    Params()
      : distance_m(23.0f),
        maxHAcc_m(2.0f),
        maxStale_ms(2000),
        minDrift_m(1.0f),
        bearingAlpha(0.3f),
        maxDistToTarget_m(125.0f),
        gpsLostTimeoutMs(5000)
    {}
  };

  enum class State : uint8_t {
    IDLE,
    ACTIVE,
    GPS_LOST    // GPS signal lost — holding last bearing, may recover
  };

  TargetLock() : p_() {}
  explicit TargetLock(const Params& p) : p_(p) {}

  bool   active()   const { return state_ == State::ACTIVE; }
  State  state()    const { return state_; }
  float  targetLat() const { return (float)targetLat_deg_; }
  float  targetLon() const { return (float)targetLon_deg_; }

  // Distance from current boat position to the stored target (metres).
  // Useful for status display and the max-distance cancel check.
  float distToTarget_m()  const { return distToTarget_m_; }

  // Distance the boat has moved since the last bearing update (metres).
  // Used internally as the drift gate; exposed here for diagnostics.
  float boatDelta_m()     const { return boatDelta_m_; }

  // Arm at current boat position + rotator heading.
  // now_ms should be millis() from the calling tick — keeps time consistent.
  bool armAtCurrent(uint32_t now_ms,
                    const GPSManager& gps,
                    Magnetometer& mag,
                    MotorController& motor,
                    HeadingLock& headingLock);

  // Drop TL — leaves HeadingLock state unchanged (controller may drop HL too).
  void drop() { state_ = State::IDLE; }

  // Called at 100 ms cadence from controller tick.
  void tick(uint32_t now_ms,
            const GPSManager& gps,
            Magnetometer& mag,
            MotorController& motor,
            HeadingLock& headingLock);

private:
  Params p_;
  State  state_ = State::IDLE;

  // Stored target coordinates (projected at arm time)
  double targetLat_deg_ = 0.0;
  double targetLon_deg_ = 0.0;

  // Last bearing fed to HeadingLock (BNO-relative frame degrees).
  // Used to compute delta for next update — keeps corrections frame-independent.
  float  lastBearing_deg_ = 0.0f;

  // Boat position at the last bearing update — used to gate updates by
  // actual boat displacement rather than distance to target.
  double lastBoatLat_deg_ = 0.0;
  double lastBoatLon_deg_ = 0.0;

  // Running distance metrics (updated each tick, exposed via accessors)
  float  distToTarget_m_ = 0.0f;   // current boat → stored target distance
  float  boatDelta_m_    = 0.0f;   // boat displacement since last bearing update

  // Timestamps
  uint32_t lastGpsUse_ms_  = 0;    // millis() of last successful bearing update
  uint32_t gpsLostTime_ms_ = 0;    // millis() when GPS_LOST state was entered

  // IIR bearing filter state
  float lastFilteredDelta_ = 0.0f;

  // Helpers
  static inline double deg2rad(double d) { return d * (M_PI / 180.0); }
  static inline double rad2deg(double r) { return r * (180.0 / M_PI); }

  static float norm360f(float x) {
    x = fmodf(x, 360.0f);
    if (x < 0.0f) x += 360.0f;
    return x;
  }

  static float wrap180f(float x) {
    x = fmodf(x + 180.0f, 360.0f);
    if (x < 0.0f) x += 360.0f;
    return x - 180.0f;
  }

  // Project a point (lat, lon) by distance_m along bearing_deg
  static void projectPoint(double lat_deg, double lon_deg,
                           float bearing_deg, float distance_m,
                           double& outLat_deg, double& outLon_deg);

  // True-North bearing from (lat1, lon1) to (lat2, lon2), degrees 0–360
  static float bearingDeg(double lat1_deg, double lon1_deg,
                          double lat2_deg, double lon2_deg);

  // Great-circle distance in metres between two lat/lon points
  static float distanceMeters(double lat1_deg, double lon1_deg,
                              double lat2_deg, double lon2_deg);
};
