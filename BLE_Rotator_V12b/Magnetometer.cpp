#include "Magnetometer.h"
#include <Wire.h>
#include <math.h>

// Maximum legitimate heading change per Game RV update (degrees).
// At 100Hz the BNO085 produces samples every 10ms. Even aggressive physical
// rotation of the boat or rotator cannot exceed this threshold in one sample.
// Any single-sample jump larger than this is a sensor glitch or I2C corruption
// and is rejected — the previous heading is kept instead.
// Consecutive bad samples (genuine fast rotation) are accepted on the second
// occurrence to avoid locking up during real fast movement.
static constexpr float SPIKE_FILTER_DEG = 30.0f;  // was 15.0 at 100Hz, doubled for 50Hz

Magnetometer::Magnetometer(float offsetDeg)
: mag_offset(offsetDeg), headingDeg(0.0f), headingInit(false),
  spikeRejected_(false), geoMagStatus_(0),
  rotVecHeadingDeg_(-1.0f), rotVecAccuracyDeg_(999.0f) {}

float Magnetometer::wrap360(float deg) {
    while (deg >= 360.f) deg -= 360.f;
    while (deg < 0.f) deg += 360.f;
    return deg;
}

float Magnetometer::wrap180(float deg) {
    while (deg > 180.f) deg -= 360.f;
    while (deg < -180.f) deg += 360.f;
    return deg;
}

float Magnetometer::quatToYawDeg(float w, float x, float y, float z) {
    float siny_cosp = 2.f * (w*z + x*y);
    float cosy_cosp = 1.f - 2.f * (y*y + z*z);
    float yaw = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;
    return wrap360(yaw);
}

bool Magnetometer::begin() {
    if (!bno.begin_I2C()) return false;

#ifdef USE_GAME_PRIMARY
    bno.enableReport(SH2_GAME_ROTATION_VECTOR,        20000);  // 50 Hz — primary control (was 100 Hz)
    // bno.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, 50000);  // disabled for testing — was 20 Hz
#endif

#ifdef USE_MAG_PRIMARY
    bno.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, 10000);  // 100 Hz primary
#endif

    // Full Rotation Vector — accel + gyro + mag fusion.
    // Most accurate absolute heading the BNO085 produces.
    // Enabled at 10 Hz — we only need it for compass observation and arm-time reference,
    // not for the fast control loop.  Low rate keeps I2C load minimal.
    bno.enableReport(SH2_ROTATION_VECTOR, 25000);   // 40 Hz (was 10 Hz)

    headingInit    = false;
    spikeRejected_ = false;
    return true;
}

void Magnetometer::update() {
    while (bno.getSensorEvent(&sensorValue)) {

#ifdef USE_GAME_PRIMARY
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float newHeading = quatToYawDeg(
                sensorValue.un.gameRotationVector.real,
                sensorValue.un.gameRotationVector.i,
                sensorValue.un.gameRotationVector.j,
                sensorValue.un.gameRotationVector.k
            );

            if (headingInit) {
                float delta = fabsf(wrap180(newHeading - headingDeg));

                if (delta > SPIKE_FILTER_DEG) {
                    // Large single-sample jump detected.
                    // If the previous sample was also rejected, this is two
                    // consecutive spikes — likely genuine fast rotation, accept it.
                    // If this is the first rejection, discard and keep last good heading.
                    if (spikeRejected_) {
                        // Second consecutive spike — accept as real movement
                        headingDeg     = newHeading;
                        spikeRejected_ = false;
                    } else {
                        // First spike — reject, keep previous heading
                        spikeRejected_ = true;
                    }
                } else {
                    // Normal sample — apply and clear spike flag
                    headingDeg     = newHeading;
                    spikeRejected_ = false;
                }
            } else {
                // First sample ever — always accept to initialise
                headingDeg     = newHeading;
                headingInit    = true;
                spikeRejected_ = false;
            }
        }
        else if (sensorValue.sensorId == SH2_GEOMAGNETIC_ROTATION_VECTOR) {
            uint8_t status = sensorValue.status; // 0–3
            geoMagStatus_ = status;  // always record latest calibration quality
            // Status 3 = high accuracy only — status 2 (low accuracy) can feed
            // noisy corrections that cause visible heading jumps.
            if (headingInit && status >= 3) {
                float magHeading = quatToYawDeg(
                    sensorValue.un.geoMagRotationVector.real,
                    sensorValue.un.geoMagRotationVector.i,
                    sensorValue.un.geoMagRotationVector.j,
                    sensorValue.un.geoMagRotationVector.k
                );
                float err = wrap180(magHeading - headingDeg);
                headingDeg = wrap360(headingDeg + 0.002f * err); // slow drift correction
            }
        }
#endif // USE_GAME_PRIMARY

#ifdef USE_MAG_PRIMARY
        if (sensorValue.sensorId == SH2_GEOMAGNETIC_ROTATION_VECTOR) {
            uint8_t status = sensorValue.status;
            geoMagStatus_ = status;
            if (status >= 2) {
                headingDeg = quatToYawDeg(
                    sensorValue.un.geoMagRotationVector.real,
                    sensorValue.un.geoMagRotationVector.i,
                    sensorValue.un.geoMagRotationVector.j,
                    sensorValue.un.geoMagRotationVector.k
                );
                if (!headingInit) headingInit = true;
            }
        }
#endif // USE_MAG_PRIMARY

        // Full Rotation Vector — accel + gyro + mag fusion.
        // Stored separately from Game Vector — not used in control loop.
        // Accuracy estimate is in radians; convert to degrees for easy reading.
        // This runs regardless of USE_GAME_PRIMARY / USE_MAG_PRIMARY mode.
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            rotVecHeadingDeg_  = wrap360(360.0f - quatToYawDeg(
                sensorValue.un.rotationVector.real,
                sensorValue.un.rotationVector.i,
                sensorValue.un.rotationVector.j,
                sensorValue.un.rotationVector.k
            ));
            // accuracyDeg < 10.0 is the target for a trustworthy North reference
            rotVecAccuracyDeg_ = sensorValue.un.rotationVector.accuracy * 57.2957795f;

            // Running circular mean — only accept samples with accuracy <= 10 deg
            if (rotVecAccuracyDeg_ <= 10.0f) {
                float rad = rotVecHeadingDeg_ * (M_PI / 180.0f);
                float s   = sinf(rad);
                float c   = cosf(rad);

                // Subtract oldest slot from running sums
                rvSinSum_ -= rvSinBuf_[rvAvgIdx_];
                rvCosSum_ -= rvCosBuf_[rvAvgIdx_];

                // Store new sample and add to sums
                rvSinBuf_[rvAvgIdx_] = s;
                rvCosBuf_[rvAvgIdx_] = c;
                rvSinSum_ += s;
                rvCosSum_ += c;

                rvAvgIdx_ = (rvAvgIdx_ + 1) % RV_AVG_N;
                if (rvAvgFill_ < RV_AVG_N) rvAvgFill_++;

                rotVecHeadingAvg_ = wrap360(atan2f(rvSinSum_, rvCosSum_) * (180.0f / M_PI));
            }
            // If accuracy > 10, keep last good average — don't update
        }
    }
}

float Magnetometer::getBoatHeading() const {
    if (!headingInit) return -1.0f;
    return wrap360(360.0F - (headingDeg + mag_offset));
}

float Magnetometer::getRotatorHeading(float encoderAngleDeg) const {
    if (!headingInit) return -1.0f;
    float heading = (360.0f - (headingDeg + mag_offset)) + encoderAngleDeg;
    return wrap360(heading);
}
