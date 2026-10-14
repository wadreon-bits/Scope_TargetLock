#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <Adafruit_BNO08x.h>

// === Heading Source Selection ===
//#define USE_MAG_PRIMARY   // Uncomment for GEOMAG primary (old method)
#define USE_GAME_PRIMARY    // Uncomment for GAME RV primary + mag drift correction

class Magnetometer {
public:
    Magnetometer(float offsetDeg = 0.0f);

    bool begin();
    void update();

    // Returns boat heading (0–360°, compass degrees).
    // If compass has been armed via armCompassRef(), returns RotVec-anchored heading
    // using game vector for smooth rotation tracking from arm time.
    // Falls back to raw game-vector heading if not yet armed.
    float getBoatHeading() const;

    // Returns rotator heading (0–360°, compass degrees) = boat heading + encoder offset.
    // Same armed/fallback behaviour as getBoatHeading().
    float getRotatorHeading(float encoderAngleDeg) const;

    // Establish the compass reference frame at arm time.
    // Stores current game-vector heading and RotVecAvg as the anchor.
    // Returns false (and leaves state unchanged) if RotVecAvg is not yet valid.
    // Call once from SController just before arming TL or HL.
    bool armCompassRef();

    // True after a successful armCompassRef() call.
    bool isCompassArmed() const { return compassArmed_; }

    void setOffset(float offsetDeg) { mag_offset = offsetDeg; }
    float getOffset() const { return mag_offset; }

    // Geomagnetic calibration status from BNO085 (0=uncalibrated, 3=fully calibrated).
    // Watch this on the bench while doing figure-8 motions after boot.
    uint8_t getGeoMagStatus() const { return geoMagStatus_; }

    // Full Rotation Vector (accel + gyro + mag fusion) — most accurate absolute heading.
    // Stored separately from the Game Vector used by the control loop.
    // Use getRotVecAccuracyDeg() to judge trustworthiness — target is < 10.0 degrees.
    // Returns -1.0 if no sample has been received yet.
    float getRotVecHeading()     const { return rotVecHeadingDeg_; }
    float getRotVecAccuracyDeg() const { return rotVecAccuracyDeg_; }

    // Running average of RotVec heading — only samples with accuracy <= 10° are included.
    // Uses circular mean (sin/cos) to handle 0/360 wrap correctly.
    // Returns -1.0 if no valid samples accumulated yet.
    float getRotVecHeadingAvg() const { return rotVecHeadingAvg_; }

private:
    Adafruit_BNO08x   bno;
    sh2_SensorValue_t sensorValue;

    float   mag_offset;      // user-defined fixed offset
    float   headingDeg;      // Game Vector heading (primary, used by control loop)
    bool    headingInit;     // true after first valid Game Vector reading
    bool    spikeRejected_;  // true if last Game RV sample was rejected by spike filter
    uint8_t geoMagStatus_;   // last geomagnetic calibration status (0=none, 3=full)

    // Compass arm-time reference — established once via armCompassRef().
    // armGameHeading_:     raw headingDeg (game vector) at arm time
    // armTrueBoatHeading_: RotVecAvg (true compass degrees) at arm time
    // After arming: boatHeading = armTrueBoatHeading_ - (headingDeg - armGameHeading_)
    bool    compassArmed_       = false;
    float   armGameHeading_     = 0.0f;
    float   armTrueBoatHeading_ = 0.0f;

    // Full Rotation Vector — stored separately, not used in control loop.
    // Observed via ST line for compass accuracy testing.
    // Will be used at arm time once validated as trustworthy.
    float rotVecHeadingDeg_  = -1.0f;   // -1 = no sample received yet
    float rotVecAccuracyDeg_ = 999.0f;  // large value = unknown / untrustworthy

    // Circular mean average of RotVec heading — accuracy-gated (<=10 deg only)
    static constexpr uint8_t RV_AVG_N = 10;
    float    rvSinBuf_[RV_AVG_N] = {};
    float    rvCosBuf_[RV_AVG_N] = {};
    float    rvSinSum_ = 0.0f;
    float    rvCosSum_ = 0.0f;
    uint8_t  rvAvgIdx_ = 0;
    uint8_t  rvAvgFill_ = 0;
    float    rotVecHeadingAvg_ = -1.0f;

    // Wrap helpers
    static float wrap360(float deg);
    static float wrap180(float deg);

    // Quaternion -> yaw in degrees (0–360)
    static float quatToYawDeg(float w, float x, float y, float z);
};

#endif
