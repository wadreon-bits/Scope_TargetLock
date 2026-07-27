// =============================================================================
// config.h  —  Master tuning & identity configuration
// =============================================================================
// All tuneable parameters live here.  Change this file per unit; do not edit
// individual class files to adjust behaviour.
//
// At runtime a global Config struct (see bottom) is initialised from these
// defaults.  Serial GET/SET commands will update the live struct; values reset
// to these defaults on reboot.
//
// NOTE: Config is a pure-data struct with no class dependencies.
// Build HeadingLock::Params / TargetLock::Params from cfg fields directly
// in the .ino (see global object declarations section).
// =============================================================================

#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Unit identity  (change per physical unit)
// ---------------------------------------------------------------------------
//#define UNIT_ID                 1                       // 1, 2, 3 …
#define BLE_TARGET_PREFIX       "Scope_Pedal_1"         // pedal this rotator pairs with
#define BLE_DEVICE_NAME         "Scope_Rotator_1"       // advertised name of this rotator


// ---------------------------------------------------------------------------
// BLE transport
// ---------------------------------------------------------------------------
#define CFG_BLE_RSSI_FILTER     (-80)                   // dBm  — ignore weaker peers
#define CFG_BLE_TX_POWER        (4)                     // dBm  — nRF52840 Tx power
#define CFG_BLE_ACTIVE_SCAN     true


// ---------------------------------------------------------------------------
// Magnetometer
// ---------------------------------------------------------------------------
// Fixed heading offset (degrees) to correct for BNO085 mounting misalignment.
// Calibrate per installation: rotate rotator to known heading, read reported
// heading, set offset = (reported – known).
#define CFG_MAG_OFFSET_DEG      0.0f

// Slow drift-correction blend rate (Game RV nudged toward GeoMag each update).
// Leave at default unless you observe long-term heading creep.
#define CFG_MAG_DRIFT_ALPHA     0.002f


// ---------------------------------------------------------------------------
// Motor / encoder
// ---------------------------------------------------------------------------
// Encoder counts per full output-shaft revolution.
// Formula: PPR × 4-edges × gearbox1 × gearbox2
// Example: 11 × 4 × 110 × 3.5 = 16940
#define CFG_COUNTS_PER_REV      16940.0f

// Zero the encoder automatically when the pedal connects.
// Allows remote re-zero by power-cycling the pedal while the rotator is
// physically aligned with the bow.  Risk: spurious reconnects will re-zero.
#define CFG_ZERO_ON_PEDAL_CONNECT   true

// Maximum travel in either direction from the encoder zero point (degrees).
// Should be slightly inside the mechanical hard stop.
#define CFG_ANGLE_LIMIT_DEG     270.0f
#define CFG_ANGLE_HYST_DEG      1.0f                    // hysteresis at limit

// PWM floor — minimum PWM value sent to driver to overcome stiction.
#define CFG_MIN_PWM             50

// DRV8874 overcurrent trip via MCP4725 DAC (milliamps).
#define CFG_OVERCURRENT_MA      1300


// ---------------------------------------------------------------------------
// Heading Lock  (proportional controller)
// ---------------------------------------------------------------------------
#define CFG_HL_KP               14.0f                   // proportional gain
#define CFG_HL_DEAD_DEG         2.0f                    // deadband (degrees)
#define CFG_HL_MIN_CMD          80                      // min command to overcome stiction
#define CFG_HL_CAP_CMD          400                     // max |command| sent to motor


// ---------------------------------------------------------------------------
// Target Lock  (GPS bearing tracker)
// ---------------------------------------------------------------------------
// Distance (metres) projected from current boat position along rotator heading
// at arm time.  23 metres ≈ 75 feet — typical Livescope pole-to-target range.
#define CFG_TL_DISTANCE_M       23.0f

// Maximum GPS horizontal accuracy (metres) required to arm Target Lock.
#define CFG_TL_MAX_HACC_M       2.0f

// Maximum GPS data age (ms) before considering a fix stale during active TL.
#define CFG_TL_MAX_STALE_MS     2000

// Minimum boat displacement (metres) from last update position before a new
// bearing update is sent to HeadingLock.  Guards against GPS position jitter
// when the boat is essentially stationary.
#define CFG_TL_MIN_DRIFT_M      3.0f

// Bearing low-pass filter coefficient (0 < alpha <= 1).
// Applied to the GPS bearing delta each tick when the boat is moving.
// Lower = smoother corrections but slower to respond to real drift.
// alpha = 1.0 disables filtering (raw delta applied directly).
// Start around 0.3 and tune on the water — too low causes sluggish response,
// too high allows GPS noise to cause motor jitter.
#define CFG_TL_BEARING_ALPHA    0.3f

// Maximum distance (metres) from stored target before Target Lock is cancelled.
// If the boat moves beyond this distance from the target (e.g. motoring to a
// new spot), TL drops to manual so a fresh target can be acquired.
// 125 metres ≈ 410 feet — generous enough to survive wind/wave drift while
// still detecting intentional repositioning.
#define CFG_TL_MAX_DIST_TO_TARGET_M  125.0f

// How long (ms) Target Lock will hold its last good bearing after GPS is lost,
// before giving up and dropping to manual.  During this window the BNO alone
// compensates for boat rotation; bearing updates from GPS are suspended.
// 5000 ms is a reasonable starting point for brief signal obstructions.
#define CFG_TL_GPS_LOST_TIMEOUT_MS   5000


// ---------------------------------------------------------------------------
// Controller timing
// ---------------------------------------------------------------------------
// Time without a pedal packet before motor output is inhibited (ms).
#define CFG_CTRL_TIMEOUT_MS     2000

// Pedal packet must be this recent to count as a healthy link (ms).
#define CFG_CTRL_LINK_RECENT_MS 500

// Idle time after last real pedal input before auto-arm engages (ms).
// 1000 ms (original) is aggressive; 3000–5000 ms is safer for a physical rotator.
#define CFG_CTRL_AUTOLOCK_MS    2000

// Pedal command magnitude threshold above which manual takeover is triggered.
#define CFG_CTRL_TAKEOVER_THR   5


// =============================================================================
// Config struct  —  live runtime values, initialised from defaults above
// =============================================================================
// Declare one global instance in your .ino:
//     Config cfg;
// Then pass cfg into each subsystem at init.
//
// To build subsystem Params structs, use the cfg fields directly, e.g.:
//     HeadingLock::Params hlp;
//     hlp.Kp = cfg.hlKp;  hlp.dead = cfg.hlDeadDeg;
//     hlp.min = cfg.hlMinCmd;  hlp.cap = cfg.hlCapCmd;
//     HeadingLock headingLock(hlp);
// =============================================================================

struct Config {

    // --- Identity ---
    //uint8_t     unitId              = UNIT_ID;

    // --- Magnetometer ---
    float       magOffsetDeg        = CFG_MAG_OFFSET_DEG;
    float       magDriftAlpha       = CFG_MAG_DRIFT_ALPHA;

    // --- Motor ---
    float       countsPerRev        = CFG_COUNTS_PER_REV;
    float       angleLimitDeg       = CFG_ANGLE_LIMIT_DEG;
    float       angleHystDeg        = CFG_ANGLE_HYST_DEG;
    uint16_t    minPwm              = CFG_MIN_PWM;
    uint16_t    overcurrentMa       = CFG_OVERCURRENT_MA;

    // --- Heading Lock ---
    float       hlKp                = CFG_HL_KP;
    float       hlDeadDeg           = CFG_HL_DEAD_DEG;
    int16_t     hlMinCmd            = CFG_HL_MIN_CMD;
    int16_t     hlCapCmd            = CFG_HL_CAP_CMD;

    // --- Target Lock ---
    float       tlDistanceM         = CFG_TL_DISTANCE_M;
    float       tlMaxHAccM          = CFG_TL_MAX_HACC_M;
    uint32_t    tlMaxStaleMs        = CFG_TL_MAX_STALE_MS;
    float       tlMinDriftM         = CFG_TL_MIN_DRIFT_M;
    float       tlBearingAlpha      = CFG_TL_BEARING_ALPHA;
    float       tlMaxDistToTargetM  = CFG_TL_MAX_DIST_TO_TARGET_M;
    uint32_t    tlGpsLostTimeoutMs  = CFG_TL_GPS_LOST_TIMEOUT_MS;

    // --- Controller ---
    uint32_t    ctrlTimeoutMs       = CFG_CTRL_TIMEOUT_MS;
    uint32_t    ctrlLinkRecentMs    = CFG_CTRL_LINK_RECENT_MS;
    uint32_t    ctrlAutoLockMs      = CFG_CTRL_AUTOLOCK_MS;
    int         ctrlTakeoverThr     = CFG_CTRL_TAKEOVER_THR;
};
