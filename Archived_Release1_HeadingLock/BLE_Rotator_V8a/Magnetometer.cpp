#include "Magnetometer.h"
#include <Wire.h>
#include <math.h>

Magnetometer::Magnetometer(float offsetDeg)
: mag_offset(offsetDeg), headingDeg(0.0f), headingInit(false) {}

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
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);        // 100 Hz primary
    bno.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, 50000); // 20 Hz for drift correction
#endif

#ifdef USE_MAG_PRIMARY
    bno.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, 10000); // 100 Hz primary
#endif

    headingInit = false;
    return true;
}

void Magnetometer::update() {
    while (bno.getSensorEvent(&sensorValue)) {

#ifdef USE_GAME_PRIMARY
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            headingDeg = quatToYawDeg(
                sensorValue.un.gameRotationVector.real,
                sensorValue.un.gameRotationVector.i,
                sensorValue.un.gameRotationVector.j,
                sensorValue.un.gameRotationVector.k
            );
            if (!headingInit) headingInit = true;
        }
        else if (sensorValue.sensorId == SH2_GEOMAGNETIC_ROTATION_VECTOR) {
            uint8_t status = sensorValue.status; // 0–3
            if (headingInit && status >= 2) {
                float magHeading = quatToYawDeg(
                    sensorValue.un.geoMagRotationVector.real,
                    sensorValue.un.geoMagRotationVector.i,
                    sensorValue.un.geoMagRotationVector.j,
                    sensorValue.un.geoMagRotationVector.k
                );
                float err = wrap180(magHeading - headingDeg);
                headingDeg = wrap360(headingDeg + 0.002f * err); // slow correction
            }
        }
#endif // USE_GAME_PRIMARY

#ifdef USE_MAG_PRIMARY
        if (sensorValue.sensorId == SH2_GEOMAGNETIC_ROTATION_VECTOR) {
            uint8_t status = sensorValue.status;
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
    }
}

float Magnetometer::getBoatHeading() const {
    if (!headingInit) return -1.0f;
    return wrap360(360.0F - (headingDeg + mag_offset));
}

float Magnetometer::getRotatorHeading(float encoderAngleDeg) const {
    if (!headingInit) return -1.0f;
    //float heading = headingDeg + mag_offset - encoderAngleDeg; //changed 0810 
    float heading = (360.0f - (headingDeg + mag_offset)) + encoderAngleDeg;
    return wrap360(heading);
}