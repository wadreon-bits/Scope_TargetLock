#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <Wire.h>

#define GPS_I2C_ADDR       0x42
#define GPS_I2C_SPEED_HZ   400000
#define GPS_MAX_UBX        128
#define GPS_TIMEOUT_MS     50       // reduced from 200 ms — fast fail if GPS busy
#define GPS_STALE_MS       3000

// Boot detection tunables — used only in begin(), never in update()
#define GPS_BOOT_ATTEMPTS  5        // probe attempts at boot
#define GPS_BOOT_DELAY_MS  400      // ms between boot probe attempts — ZED-F9P needs time to wake

class GPSManager {
public:
    enum HealthState {
        NO_FIX,
        ACQUIRING,
        VALID,
        LOST
    };

    struct GPSEpoch {
        uint32_t iTOW;
        double lat;
        double lon;
        double hMSL;
        float groundSpeed;
        float heading;
        uint8_t fixType;
        uint8_t numSV;
        float hAcc;        // metres
        bool valid;
    };

    GPSManager();

    // Probe for GPS once at boot with retries.
    // Sets available_ flag; if not found update() becomes a no-op forever.
    // No further I2C probing after begin() returns — zero overhead if absent.
    void begin(TwoWire &bus = Wire);

    // Called every loop() pass.
    // Returns immediately if GPS not available at boot.
    // Reads gated by GPS_TIMEOUT_MS — no blocking spin loops.
    void update();

    bool isHealthy()   const;
    bool isAvailable() const { return available_; }  // was GPS found at boot?
    HealthState getHealth() const;
    const GPSEpoch& getData() const { return lastGood_; }

private:
    bool     gpsPresent();    // used only during begin() boot probe
    uint16_t bytesAvailable(); // reads u-blox DDC stream-length registers 0xFD/0xFE
    int      i2cReadN(uint8_t* dst, size_t n);
    int      readOneUBX(uint8_t &cls, uint8_t &id, uint16_t &len, uint8_t* payload);
    void parseNAV_PVT(const uint8_t* p, uint16_t len);
    void parseNAV_VELNED(const uint8_t* p, uint16_t len);
    void evaluateHealth();

    TwoWire*    wire_;
    HealthState state_;
    GPSEpoch    lastGood_;

    bool     available_;   // set once at boot, never changes after begin()
    uint32_t lastFresh_;
    uint32_t lastRead_;
    // lastProbe_ removed — no ongoing probing after boot
};

#endif
