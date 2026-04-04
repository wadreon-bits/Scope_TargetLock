#include "GPSManager.h"

#define UBX_NAV     0x01
#define ID_PVT      0x07
#define ID_VELNED   0x12

GPSManager::GPSManager()
: wire_(nullptr),
  state_(NO_FIX),
  available_(false),
  lastFresh_(0),
  lastRead_(0)
{
    memset(&lastGood_, 0, sizeof(lastGood_));
}

// =============================================================================
// begin()
// Probes for GPS at boot with up to GPS_BOOT_ATTEMPTS retries.
// Sets available_ = true if found, false otherwise.
// After begin() returns, update() checks available_ and returns immediately
// if GPS was not found — zero I2C activity, zero blocking.
// =============================================================================
void GPSManager::begin(TwoWire &bus)
{
    wire_ = &bus;
    wire_->begin();
    wire_->setClock(GPS_I2C_SPEED_HZ);

    // Set a short I2C timeout so failed probes return quickly.
    // On nRF52 Arduino core, setTimeout() is supported on TwoWire.
    wire_->setTimeout(10);   // 10 ms max per transaction — fast fail

    available_ = false;
    state_     = NO_FIX;

    // Probe with retries — GPS module may need a moment to come up after power-on
    for (int attempt = 1; attempt <= GPS_BOOT_ATTEMPTS; ++attempt) {
        if (gpsPresent()) {
            available_ = true;
            state_     = ACQUIRING;
            lastFresh_ = millis();
            break;
        }
        delay(GPS_BOOT_DELAY_MS);
    }
    // available_ is now fixed for the session — no further probing ever
}

// =============================================================================
// gpsPresent()
// Simple I2C ACK probe — used only during begin().
// With wire_->setTimeout(10) this returns in <10 ms if nothing is there.
// =============================================================================
bool GPSManager::gpsPresent()
{
    wire_->beginTransmission(GPS_I2C_ADDR);
    return (wire_->endTransmission() == 0);
}

// =============================================================================
// i2cReadN()
// Reads n bytes from GPS over I2C in 32-byte chunks.
// Uses millis() timeout (GPS_TIMEOUT_MS = 50 ms) — no infinite spin.
// Returns number of bytes actually read.
// =============================================================================
int GPSManager::i2cReadN(uint8_t *dst, size_t n)
{
    size_t got = 0;
    while (got < n) {
        size_t chunk = min((size_t)32, n - got);
        wire_->requestFrom((int)GPS_I2C_ADDR, (int)chunk);

        unsigned long t0 = millis();
        // Wait for bytes with a hard timeout — never spin forever
        while ((size_t)wire_->available() < chunk) {
            if (millis() - t0 >= GPS_TIMEOUT_MS) return (int)got;  // bail early
        }

        while (wire_->available() && got < n) {
            dst[got++] = wire_->read();
        }
    }
    return (int)got;
}

// =============================================================================
// readOneUBX()
// Reads one complete UBX frame from the GPS I2C stream.
// Returns payload length on success, 0 on failure or timeout.
// =============================================================================
int GPSManager::readOneUBX(uint8_t &cls, uint8_t &id, uint16_t &len, uint8_t *payload)
{
    uint8_t b;

    // Search for UBX sync chars 0xB5 0x62
    for (int tries = 0; tries < 128; ++tries) {
        if (i2cReadN(&b, 1) != 1) return 0;
        if (b == 0xB5) {
            if (i2cReadN(&b, 1) != 1) return 0;
            if (b == 0x62) break;
        }
    }

    uint8_t hdr[4];
    if (i2cReadN(hdr, 4) != 4) return 0;

    cls = hdr[0];
    id  = hdr[1];
    len = (uint16_t)(hdr[2] | (hdr[3] << 8));

    if (len > (GPS_MAX_UBX - 8)) return 0;

    if (i2cReadN(payload, len + 2) != (int)(len + 2)) return 0;

    // Verify checksum
    uint8_t calcA = 0, calcB = 0;
    auto add = [&](uint8_t v) { calcA += v; calcB += calcA; };

    add(cls);
    add(id);
    add((uint8_t)(len & 0xFF));
    add((uint8_t)(len >> 8));
    for (uint16_t i = 0; i < len; i++) add(payload[i]);

    if (calcA != payload[len] || calcB != payload[len + 1]) return 0;

    return len;
}

// =============================================================================
// parseNAV_PVT()
// Extracts position, fix type, satellite count, and accuracy from NAV-PVT.
// =============================================================================
void GPSManager::parseNAV_PVT(const uint8_t *p, uint16_t len)
{
    if (len < 92) return;

    lastGood_.iTOW = (uint32_t)(p[0] | p[1]<<8 | p[2]<<16 | p[3]<<24);

    lastGood_.lon  = ((int32_t)(p[24] | p[25]<<8 | p[26]<<16 | p[27]<<24)) * 1e-7;
    lastGood_.lat  = ((int32_t)(p[28] | p[29]<<8 | p[30]<<16 | p[31]<<24)) * 1e-7;
    lastGood_.hMSL = ((int32_t)(p[32] | p[33]<<8 | p[34]<<16 | p[35]<<24)) / 1000.0;

    lastGood_.fixType = p[20];
    lastGood_.numSV   = p[23];
    lastGood_.hAcc    = ((uint32_t)(p[40] | p[41]<<8 | p[42]<<16 | p[43]<<24)) / 1000.0f;  // mm → m

    lastGood_.valid = (lastGood_.fixType >= 3);
}

// =============================================================================
// parseNAV_VELNED()
// Extracts ground speed and heading from NAV-VELNED.
// =============================================================================
void GPSManager::parseNAV_VELNED(const uint8_t *p, uint16_t len)
{
    if (len < 36) return;

    int32_t gSpd = (int32_t)(p[20] | p[21]<<8 | p[22]<<16 | p[23]<<24);
    int32_t head = (int32_t)(p[24] | p[25]<<8 | p[26]<<16 | p[27]<<24);

    lastGood_.groundSpeed = gSpd / 1000.0f;
    lastGood_.heading     = head * 1e-5f;
}

// =============================================================================
// update()
// Called every loop() pass.
// Returns immediately if GPS was not detected at boot — zero I2C activity.
// Reads one UBX frame every 100 ms when GPS is present.
// No blocking probe — available_ is fixed after begin().
// =============================================================================
void GPSManager::update()
{
    // Fast exit — GPS not present at boot, nothing to do
    if (!available_) return;

    unsigned long now = millis();

    if (now - lastRead_ >= 100) {
        lastRead_ = now;

        uint8_t cls, id;
        uint16_t len;
        uint8_t payload[GPS_MAX_UBX];

        int got = readOneUBX(cls, id, len, payload);

        if (got > 0 && cls == UBX_NAV) {
            if (id == ID_PVT) {
                parseNAV_PVT(payload, len);
                lastFresh_ = now;
            } else if (id == ID_VELNED) {
                parseNAV_VELNED(payload, len);
            }
        } else {
            // No valid frame — drain any stale bytes to keep bus clean
            int cleared = 0;
            while (cleared < 64) {
                int n = wire_->requestFrom((int)GPS_I2C_ADDR, 16);
                if (n <= 0) break;
                while (wire_->available()) { (void)wire_->read(); cleared++; }
            }
        }
    }

    evaluateHealth();
}

// =============================================================================
// evaluateHealth()
// Updates state_ based on data freshness and fix validity.
// Only called when available_ is true.
// =============================================================================
void GPSManager::evaluateHealth()
{
    unsigned long now = millis();

    if (now - lastFresh_ > GPS_STALE_MS) {
        state_ = LOST;
        return;
    }

    if (!lastGood_.valid) {
        state_ = ACQUIRING;
        return;
    }

    state_ = VALID;
}

bool GPSManager::isHealthy() const
{
    return (state_ == VALID);
}

GPSManager::HealthState GPSManager::getHealth() const
{
    return state_;
}
