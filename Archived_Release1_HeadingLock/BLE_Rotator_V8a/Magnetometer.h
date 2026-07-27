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

    // Returns boat heading (0–360°)
    float getBoatHeading() const;

    // Returns rotator heading given encoder rotation from 0° reference
    float getRotatorHeading(float encoderAngleDeg) const;

    void setOffset(float offsetDeg) { mag_offset = offsetDeg; }
    float getOffset() const { return mag_offset; }

private:
    Adafruit_BNO08x bno;
    sh2_SensorValue_t sensorValue;

    float mag_offset;      // user-defined fixed offset
    float headingDeg;      // heading from chosen mode
    bool  headingInit;     // true after first valid reading

    // Wrap helpers
    static float wrap360(float deg);
    static float wrap180(float deg);

    // Quaternion -> yaw in degrees (0–360)
    static float quatToYawDeg(float w, float x, float y, float z);
};

#endif