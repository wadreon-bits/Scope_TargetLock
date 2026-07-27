#include "TargetLock.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Earth radius (metres, WGS-84 mean)
static constexpr double kEarthRadius_m = 6378137.0;

// ---------------------------------------------------------------------------
// distanceMeters — great-circle distance between two lat/lon points (metres)
// ---------------------------------------------------------------------------
float TargetLock::distanceMeters(double lat1_deg, double lon1_deg,
                                 double lat2_deg, double lon2_deg) {
  const double DEG2RAD = M_PI / 180.0;

  double lat1 = lat1_deg * DEG2RAD;
  double lat2 = lat2_deg * DEG2RAD;
  double dLat = (lat2_deg - lat1_deg) * DEG2RAD;
  double dLon = (lon2_deg - lon1_deg) * DEG2RAD;

  double a = sin(dLat * 0.5) * sin(dLat * 0.5) +
             cos(lat1) * cos(lat2) *
             sin(dLon * 0.5) * sin(dLon * 0.5);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return (float)(kEarthRadius_m * c);
}

// ---------------------------------------------------------------------------
// projectPoint — project (lat, lon) by distance_m along bearing_deg
// ---------------------------------------------------------------------------
void TargetLock::projectPoint(double lat_deg, double lon_deg,
                              float bearing_deg, float distance_m,
                              double& outLat_deg, double& outLon_deg) {
  double lat  = deg2rad(lat_deg);
  double lon  = deg2rad(lon_deg);
  double brng = deg2rad(bearing_deg);
  double dr   = distance_m / kEarthRadius_m;

  double sinLat = sin(lat);
  double cosLat = cos(lat);
  double sinDr  = sin(dr);
  double cosDr  = cos(dr);

  double sinLat2 = sinLat * cosDr + cosLat * sinDr * cos(brng);
  double lat2    = asin(sinLat2);

  double y    = sin(brng) * sinDr * cosLat;
  double x    = cosDr - sinLat * sinLat2;
  double lon2 = lon + atan2(y, x);

  outLat_deg = rad2deg(lat2);
  outLon_deg = rad2deg(lon2);
}

// ---------------------------------------------------------------------------
// bearingDeg — true-North bearing from point 1 to point 2 (0–360 degrees)
// ---------------------------------------------------------------------------
float TargetLock::bearingDeg(double lat1_deg, double lon1_deg,
                             double lat2_deg, double lon2_deg) {
  double lat1 = deg2rad(lat1_deg);
  double lat2 = deg2rad(lat2_deg);
  double dLon = deg2rad(lon2_deg - lon1_deg);

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) -
             sin(lat1) * cos(lat2) * cos(dLon);

  double brng = atan2(y, x);   // -pi..pi
  float  deg  = (float)rad2deg(brng);
  return norm360f(deg);
}

// ---------------------------------------------------------------------------
// armAtCurrent — project target from current boat pos + rotator heading,
//                initialise all state, arm HeadingLock to match.
// ---------------------------------------------------------------------------
bool TargetLock::armAtCurrent(uint32_t now_ms,
                              const GPSManager& gps,
                              Magnetometer& mag,
                              MotorController& motor,
                              HeadingLock& headingLock) {
  // GPS must be healthy with a valid fix
  if (!gps.isHealthy()) return false;

  const GPSManager::GPSEpoch& e = gps.getData();
  if (!e.valid)             return false;
  if (e.hAcc > p_.maxHAcc_m) return false;

  // Establish compass reference before computing rotator heading.
  // armCompassRef() must succeed first so that getRotatorHeading() returns
  // true-North compass degrees when projecting the target point.
  // If called before compassArmed_ is set, getRotatorHeading() falls back to
  // game-frame (boot-relative), projecting the target at the wrong geographic
  // direction and causing a large initial heading error after arm.
  // If RotVecAvg is not yet valid, armCompassRef() returns false and TL refuses
  // to arm — controller falls through to headingLock.armAtCurrent() (HL only).
  if (!mag.armCompassRef()) return false;

  // Rotator heading — compass frame is now established; always returns compass degrees.
  float rotHead = mag.getRotatorHeading(motor.getAngleDegCumulative());
  if (rotHead < 0.0f) return false;

  // Project the target point ahead of the boat along the current rotator heading
  projectPoint(e.lat, e.lon,
               rotHead,
               p_.distance_m,
               targetLat_deg_, targetLon_deg_);

  // Compute the true-North bearing to that projected point
  float bearing = bearingDeg(e.lat, e.lon, targetLat_deg_, targetLon_deg_);

  // Store the bearing and the current boat position.
  // lastBoatLat/Lon initialised here so the first tick drift check starts
  // from the arm position — prevents a false large-delta on the first update.
  lastBearing_deg_  = bearing;
  lastBoatLat_deg_  = e.lat;
  lastBoatLon_deg_  = e.lon;
  lastGpsUse_ms_    = now_ms;

  // Initialise distance metrics and filter state
  distToTarget_m_    = p_.distance_m;   // at arm time, boat is exactly distance_m away
  boatDelta_m_       = 0.0f;
  lastFilteredDelta_ = 0.0f;

  headingLock.setTarget(bearing);

  state_ = State::ACTIVE;
  return true;
}

// ---------------------------------------------------------------------------
// tick — called every 100 ms while TL is armed.
//         Responsibilities:
//           1. Detect and handle GPS loss / recovery.
//           2. Check max-distance cancel condition.
//           3. Gate bearing updates on actual boat displacement.
//           4. Apply bearing delta (frame-independent) to HeadingLock.
// ---------------------------------------------------------------------------
void TargetLock::tick(uint32_t now_ms,
                      const GPSManager& gps,
                      Magnetometer& mag,
                      MotorController& motor,
                      HeadingLock& headingLock) {

  if (state_ == State::IDLE) return;

  // ---- GPS health check ------------------------------------------------
  if (!gps.isHealthy()) {
    if (state_ == State::ACTIVE) {
      // Transition to GPS_LOST: record the time, suspend bearing updates.
      // HeadingLock keeps running on its last target using BNO alone —
      // no garbage data is fed; we simply stop updating the target.
      state_          = State::GPS_LOST;
      gpsLostTime_ms_ = now_ms;
    } else {
      // Already GPS_LOST — check if we have exceeded the hold timeout
      if ((uint32_t)(now_ms - gpsLostTime_ms_) > p_.gpsLostTimeoutMs) {
        // Held long enough with no recovery — give up, controller will drop to manual
        state_ = State::IDLE;
      }
      // Either way, nothing more to do this tick while GPS is out
    }
    return;
  }

  // GPS is healthy — if we were in GPS_LOST, recover back to ACTIVE
  if (state_ == State::GPS_LOST) {
    // Resume from here: reset the last-boat-position to current so the
    // drift gate doesn't fire immediately from accumulated position change
    // during the outage.
    const GPSManager::GPSEpoch& er = gps.getData();
    if (er.valid && er.hAcc <= p_.maxHAcc_m) {
      lastBoatLat_deg_ = er.lat;
      lastBoatLon_deg_ = er.lon;
      lastGpsUse_ms_   = now_ms;
      state_           = State::ACTIVE;
    } else {
      // GPS present but not yet a clean fix — keep waiting
      return;
    }
  }

  // ---- From here state_ == ACTIVE and GPS is healthy -------------------

  const GPSManager::GPSEpoch& e = gps.getData();

  // Validate fix quality
  if (!e.valid || e.hAcc > p_.maxHAcc_m) {
    // Degraded fix — treat as GPS_LOST and start the hold timer
    state_          = State::GPS_LOST;
    gpsLostTime_ms_ = now_ms;
    return;
  }

  // Staleness check — if the GPS epoch has not refreshed recently, treat as lost
  if ((uint32_t)(now_ms - lastGpsUse_ms_) > p_.maxStale_ms) {
    state_          = State::GPS_LOST;
    gpsLostTime_ms_ = now_ms;
    return;
  }

  // ---- Distance to target ----------------------------------------------
  // Always update distToTarget for status display and the cancel check,
  // regardless of whether we fire a bearing update this tick.
  distToTarget_m_ = distanceMeters(e.lat, e.lon,
                                   targetLat_deg_, targetLon_deg_);

  // Max-distance cancel: boat has moved too far from target — user has
  // likely repositioned intentionally.  Drop TL; controller falls to manual.
  if (distToTarget_m_ > p_.maxDistToTarget_m) {
    state_ = State::IDLE;
    return;
  }

  // Update lastGpsUse_ms_ whenever fix is valid — prevents false staleness
  // timeout when boat is stationary and drift gate never fires.
  lastGpsUse_ms_ = now_ms;

  // ---- Boat displacement gate ------------------------------------------
  // Measure how far the boat has moved from the position of the last
  // bearing update.  This correctly detects drift in any direction,
  // unlike a distance-to-target comparison which misses perpendicular drift.
  boatDelta_m_ = distanceMeters(e.lat, e.lon,
                                lastBoatLat_deg_, lastBoatLon_deg_);

  if (boatDelta_m_ < p_.minDrift_m) {
    // Boat hasn't moved enough — hold current HeadingLock target, do nothing
    return;
  }

  // ---- Bearing delta update --------------------------------------------
  // Compute the new true-North bearing from current boat position to target.
  float newBearing = bearingDeg(e.lat, e.lon, targetLat_deg_, targetLon_deg_);

  // Compute the change in bearing since last update.
  // Using wrap180 makes the delta always the shortest angular path (-180..+180).
  // This delta is frame-independent — it represents a real angular change
  // regardless of whether the BNO frame matches true North.
  float delta = wrap180f(newBearing - lastBearing_deg_);

  // IIR low-pass filter on bearing delta to smooth GPS position noise.
  // output = alpha * newDelta + (1-alpha) * previousOutput
  // alpha = 1.0 applies full delta immediately; lower values smooth more.
  float filteredDelta = p_.bearingAlpha * delta + (1.0f - p_.bearingAlpha) * lastFilteredDelta_;
  lastFilteredDelta_  = filteredDelta;

  // Apply the filtered delta to the current HeadingLock target.
  // We work in the BNO-relative frame throughout — the delta is what matters,
  // not the absolute bearing value.
  float newTarget = norm360f(headingLock.target() + filteredDelta);
  headingLock.setTarget(newTarget);

  // Update stored state for next tick — only after we have acted on it
  lastBearing_deg_  = newBearing;
  lastBoatLat_deg_  = e.lat;
  lastBoatLon_deg_  = e.lon;
  lastGpsUse_ms_    = now_ms;
}
