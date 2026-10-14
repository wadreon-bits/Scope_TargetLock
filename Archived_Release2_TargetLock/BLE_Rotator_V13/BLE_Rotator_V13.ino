// === BLE_RECEIVER_ROTATOR.ino ===
// BLE Receiver Rotator for Adafruit nRF52840 (Feather or Itsybitsy)
// Connects to BLE controller pedals named "BLE_Controller_Pedal_<#>"
// Uses: DRV_IN1_PIN (D5), DRV_IN2_PIN (D12), DRV_SLP_PIN (D9)
//V2 Added Current Measurement, Overcurrent Adjustment - Working tested with Pedal_V1
//V3 Added Encoder read, full quadrature, added angle resolution, added goto angle serial cmnd, added angle limits  (Pedal_V1)
//V4 Beginning to add in targeting features - starting with BNO085 magnetometer - Geomagnetic vector
//V5 Have mag working with sign issues -  Changed to using Game Rotation Vector.
//V6 Working heading tracking - Slow control loop
//V6a - Working updated heading target Kp to 14. Motor control not class yet
// NOTE!! Changed Pedal BLE advertised name to 'Scope_Pedal_1'.
//V6b working on class for MotorController.
//V7a created BLE class- migrated and tested working so far, logic Errors. Logic cleaned in 7A
//V7b heading lock class added, testing working so far. Next is Main state machine, and command handler.
//     Seems to be working well. Learned about heading frame- relative position when waking up. Rotator and Mag zero
//      At boot- thus tracking movement from that point on. Slowly corrected for magnetic north with filter.
//      Should still work as the eventual target math will be made relative to rotator
//V8  Adding controller class- Working with guards in main loop. Minor Testing working Pedal V1
//V8A Updated Controller Class with Step 2
//V8B 11/13/25 Ok, after hiatus, back at it. Completed step 3 from GPT 'Code for steps 1-3'. Actually decided to just remove GOTO angle command- likely never used.
//      So all state variables etc are all moved properly into the controller class.
//      Still using Scope_Pedal_1
//V8C 11/18/25 Rebuilt using V8A as base. Chatgpt stripped a bunch of code and comments that I wanted back. V8C has steps 1-3 complete, so all
//      goto angle has been removed. Tested briefly working with pedal 1. Some oddities with neopixel needs addressed. (neopixel fixed- was map function wrap)
//V9a 12/10/25 V8C (GPS Installed) with GPS manager reporting basic info in the main loop. Working!  Note i2c slows main loop if no sensor on bus- retries
//V9b 12/11/25 (Basic Target lock Installed) Added TL class and header. rebuilt controller class to manage new target lock mode. Working basically, no major testing.
//            controller manages tiered mode controller. Downgrades to only features available.  TL>HL>MAN
//            Added deadband to drift, min drift setup in TL class. currently set to 1.0meters.
//            Added Status debug print line
//            ST,0:now_ms,1:mode,2:gpsState,3:gps_ok,4:hAcc_m,5:boatHead_deg,6:rotHead_deg,7:TL_on,8:distToTarget_m,9:HL_target_deg,10:motorCmd
//V9c 12/20/25 Basically same as b, however was screwing around with the dotstar to reflect current mode and value, Was getting confused with Kapton
//             tape hiding the violet color, made me dig into a bug that didnt exist. Not sure how 'Refined' of a solution V9c is? Must retest
//V10  03/26 Config refactor: all tunables moved to config.h / Config struct. First use of CLAUDE!!!!! 
//           SController: constexprs replaced with cfg-driven member vars.
//           Fixed false "Magnetometer ready" print on failed init.
//           Removed dead code: goToAngleActive, COUNTS_PER_REV local, mag_offset local.
//           config.h is now a pure-data header (no class deps); params structs
//           for HeadingLock and TargetLock are built inline from cfg fields.
//V10.1 03/26 Serial command parser rewrite.
//           Non-blocking char accumulator replaces readStringUntil() blocking call.
//           Unified L/l for lock enable/disable (replaces H/h/T/u/U).
//           Added VER, INFO, STATUS ON/OFF, ? help commands.
//           statusEnabled flag gates 4 Hz ST stream.
//V11  Targeting logic overhaul:
//           Fixed sensor frame mismatch: TL now feeds HeadingLock bearing *delta*
//             (frame-independent) instead of raw true-North GPS bearing.
//           Fixed drift gate: now measures actual boat displacement from last
//             update position (any direction), replacing incorrect boat-to-target
//             distance comparison that missed perpendicular drift.
//           Added bearing low-pass filter (bearingAlpha) to smooth GPS corrections.
//           Added GPS_LOST hold: TL survives brief GPS outages (5 s default),
//             BNO-only rotation compensation continues during hold window.
//           Added max-distance cancel: TL drops to manual if boat moves >125 m
//             from target, detecting intentional repositioning to a new spot.
//           Corrected CFG_TL_DISTANCE_M from 75.0 to 23.0 (75 ft in metres).
//           armAtCurrent() now accepts now_ms for consistent timestamping.
//           Renamed lastDrift_m_ -> boatDelta_m_; added distToTarget_m_.
//           Status line field 8 now reports distToTarget_m (boat to target).
//V12  BLE config interface + SET/GET tuning commands:
//           Added BLEConfigService — Nordic UART Service peripheral role.
//             nRF52840 runs Central (pedal) + Peripheral (phone config) simultaneously.
//             Phone connects with nRF Toolbox, Serial Bluetooth Terminal, or LightBlue.
//             All serial commands including SET/GET work identically over BLE.
//           Added SET/GET serial commands for live tuning of all cfg parameters.
//             Changes take effect immediately; reset to config.h defaults on reboot.
//             Lets you tune on the water via phone, hardcode values when home.
//           BLEManager: Bluefruit.begin(0,1) -> begin(1,1) to enable peripheral slot.
//           Fixed L/l commands — L enables auto-lock permanently, l disables permanently.
//V12.1 Removed continuous ST stream over BLE — was causing motor sluggishness.
//           Phone BLE connection is now config terminal only (SET/GET/INFO/commands).
//           ST stream remains on serial at 4 Hz unchanged.
//           No continuous uart_.write() calls — BLE stack only active when command
//             response is sent, eliminating interference with pedal central role.
//V12.2 Fixed GPS I2C blocking — root cause of BNO glitches and loop sluggishness.
//           GPS now detected once at boot with retries, never probed again.
//           update() returns immediately if GPS not found at boot — zero I2C overhead.
//           GPS_TIMEOUT_MS reduced 200->50 ms; wire setTimeout(10 ms) added.
//           Removed blocking gpsPresent() probe from update() — was stalling
//             main loop and corrupting BNO I2C reads when GPS acquiring.
//           GPS and Magnetometer both fully optional — system boots and runs
//             cleanly regardless of which sensors are connected.
//           INFO command now reports GPS and mag availability.
//           BLEConfigService advertising interval reduced 100ms->500ms to reduce
//             SoftDevice interrupt load on main loop and BNO I2C reads.
//V12.3 BNO085 Game Rotation Vector spike filter added to Magnetometer.cpp.
//           Diagnosed: BNO085 produces occasional single bad Game RV quaternion
//             during thermal warmup causing 15-20 degree heading spikes.
//           Fix: reject any Game RV sample that jumps more than 15 degrees from
//             previous heading in one update — physically impossible for real motion.
//           Consecutive rejections accepted (genuine fast rotation allowed through).
//           GeoMag drift correction kept at status>=3 (high accuracy only).
//           All debug prints removed — clean production build.
//V13.0 Compass-referenced heading architecture (full rework):
//           Magnetometer: armCompassRef() stores RotVecAvg + game-vector heading
//             at arm time as the compass anchor. After arm, getBoatHeading() and
//             getRotatorHeading() return true-North compass degrees using:
//               boatHeading = armTrueBoatHeading - (gameHeading - armGameHeading)
//             Game vector contributes only its delta from arm — no dependency on
//             boot orientation; gyro drift only accumulates from arm time (~0.25 deg/30min).
//             Falls back to raw game-frame if RotVecAvg not valid at arm time.
//           TargetLock: armAtCurrent() calls armCompassRef() — returns false if
//             RotVecAvg not ready, controller demotes to HL. HL target set directly
//             as true-North GPS bearing, no frame translation needed.
//           TL tick: bearing delta corrections remain frame-independent (unchanged).
//             Radial drift (toward/away from target) produces zero bearing delta —
//             naturally ignored. Only lateral drift updates HL target.
//           SController: armCompassRef() also attempted before standalone HL arm —
//             HL gets compass-referenced heading when RotVecAvg is available.
//           CFG_TL_MIN_DRIFT_M default changed 1.0 -> 3.0 m.
//           BNO085 report rates tuned: Game RV 100->50 Hz, RotVec 10->40 Hz,
//             GeoMag disabled — reduces I2C load, improves RotVec settling time.
//           Spike filter threshold doubled 15->30 deg to match 50 Hz sample rate.
//V13.1 Bug fixes + MAX_MODE command:
//           Added SET/GET MAX_MODE — configures the highest targeting mode the system
//             will use: TL (2, default), HL (1), or MANUAL (0, no targeting).
//             Accepts token (TL/HL/MANUAL) or numeric (2/1/0). Session-only; reboot
//             resets to CFG_MAX_MODE in config.h.
//           Removed L/l single-char commands — redundant with SET MAX_MODE.
//V13.1 Bug fixes only — no behavior changes:
//           Bug A (TargetLock.cpp armAtCurrent): armCompassRef() moved before
//             getRotatorHeading(). Previously rotHead was computed in game-frame
//             fallback when compassArmed_ was false (TL arming before HL had ever
//             armed), projecting the GPS target at the wrong geographic direction.
//             After armCompassRef set the compass anchor the tracking frame changed
//             but the target bearing was derived from the wrong projection — rotator
//             drove to an incorrect heading on first TL arm when GPS and RotVecAvg
//             became available simultaneously.
//             Fix: armCompassRef() first; rotHead always computed in compass frame.
//           Bug B (GPSManager.cpp update): bytesAvailable() helper added — reads
//             u-blox DDC stream-length registers 0xFD/0xFE before calling readOneUBX.
//             Previously the 128-iteration sync-byte search ran whenever GPS had no
//             valid frame, blocking the shared I2C bus for up to ~1 s worst case and
//             starving BNO085 reads — caused occasional full freezes and progressive
//             blocky HL correction during extended GPS-acquiring periods indoors.
//             Fix: skip read entirely when GPS buffer reports 0 bytes available;
//             sync search and drain loop now only run when data is confirmed present.


#include <bluefruit.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "NRF52TimerInterrupt.h"
#include <Adafruit_DotStar.h>
#include <Adafruit_BNO08x.h>
#include "config.h"           // ← all tunables live here now (pure data, no class deps)
#include "Magnetometer.h"
#include "MotorController.h"
#include "BLEManager.h"
#include "BLEConfigService.h" // ← V12: phone config interface
#include "HeadingLock.h"
#include "SController.h"
#include "GPSManager.h"
#include "TargetLock.h"


#define APP_VERSION     "13.0"
#define HW_REV            3
//#define DEBUG_PRINTS
//#define PRINT_HEADINGS
#define DEBUG_BLE_EVENTS 0
//#define PRINT_BLE_CMDS
#define ENABLE_DOTSTAR

// === Pin Definitions ===
#define DRV_IN1_PIN       13
#define DRV_IN2_PIN       12
#define ENC_A_PIN         11
#define ENC_B_PIN         10
#define DRV_SLP_PIN       9
#define DRV_FLT_PIN       7
#define LED_STATUS_PIN    3
#define LED_PWRBTN_PIN    2
#define DOTSTAR_DATA_PIN  8
#define DOTSTAR_CLK_PIN   6

#if HW_REV == 2
  #define IPROPI_PIN        A0 //Changed on REV3 hardware->  A5
#elif HW_REV == 3

  #define IPROPI_PIN        A5
  #define VSENSOR_EN        A0
#else
  #error "MUST DEFINE HW REV"
#endif




// === CONSTANTS ===
#define HW_TIMER_INTERVAL_US      100000  // 100 ms timer tick


// ======================================================================
// SYSTEM LED ABSTRACTION LAYER
// Change ONLY these 4 lines when switching LED hardware.
// ======================================================================
#define SYSLED_BEGIN()              sysLED.begin()
#define SYSLED_SET_BRIGHT(x)        sysLED.setBrightness(x)
#define SYSLED_SET_PIXEL(r,g,b)     sysLED.setPixelColor(0, (r), (g), (b))
#define SYSLED_SHOW()               sysLED.show()
// To switch to NeoPixel later, replace the sysLED declaration below and
// update these 4 macros. No other code changes required.
// ======================================================================


// --- System State ---
bool     flag_100ms    = false;
int      bleEnabled    = true;
int      currentCommand = 0;       // Last received motor command

// Sensor availability — set once at boot, never change.
// Controls whether sensor-dependent features are offered.
static bool magAvailable = false;  // BNO085 found and initialised at boot
// gpsAvailable is tracked inside GPSManager via gps.isAvailable()

// Status print timing
static uint32_t lastStatusPrint_ms = 0;
const  uint32_t STATUS_PERIOD_MS   = 250;  // 4 Hz
static bool     statusEnabled      = true; // toggled by STATUS ON/OFF command


// ======================================================================
// GLOBAL OBJECT DECLARATIONS
// Config must be first — other constructors read from it.
// HeadingLock and TargetLock params are built from cfg fields directly
// here, so config.h stays free of class dependencies.
// ======================================================================
Config cfg;   // ← all tunables, initialised from config.h defaults

NRF52Timer ITimer(NRF_TIMER_3);
Adafruit_DotStar sysLED(1, DOTSTAR_DATA_PIN, DOTSTAR_CLK_PIN, DOTSTAR_BRG);

Magnetometer mag(cfg.magOffsetDeg);

MotorController::Pins motorPins {
  DRV_IN1_PIN, DRV_IN2_PIN, DRV_SLP_PIN,
  DRV_FLT_PIN, IPROPI_PIN,
  ENC_A_PIN, ENC_B_PIN
};
MotorController motor(motorPins, cfg.countsPerRev);

BLEManager      ble;
BLEConfigService bleConfig;   // V12: phone config peripheral

// Build HeadingLock params inline from cfg — no helper method needed
static HeadingLock::Params makeHLParams() {
  HeadingLock::Params p;
  p.Kp   = cfg.hlKp;
  p.dead = cfg.hlDeadDeg;
  p.min  = cfg.hlMinCmd;
  p.cap  = cfg.hlCapCmd;
  return p;
}
HeadingLock headingLock(makeHLParams());

// Build TargetLock params inline from cfg — no helper method needed
static TargetLock::Params makeTLParams() {
  TargetLock::Params p;
  p.distance_m         = cfg.tlDistanceM;
  p.maxHAcc_m          = cfg.tlMaxHAccM;
  p.maxStale_ms        = cfg.tlMaxStaleMs;
  p.minDrift_m         = cfg.tlMinDriftM;
  p.bearingAlpha       = cfg.tlBearingAlpha;
  p.maxDistToTarget_m  = cfg.tlMaxDistToTargetM;
  p.gpsLostTimeoutMs   = cfg.tlGpsLostTimeoutMs;
  return p;
}
TargetLock  targetLock(makeTLParams());

Controller  controller;
GPSManager  gps;


// --- Small utilities ---
static inline float wrap180(float x) {
  x = fmodf(x + 180.f, 360.f);
  if (x < 0) x += 360.f;
  return x - 180.f;
}

static inline void updatePowerLed(bool connected) {
  static uint8_t cnt = 0;
  const uint8_t onTicks = connected ? 29 : 3;
  const uint8_t period  = connected ? 30 : 6;
  digitalWrite(LED_PWRBTN_PIN, (cnt < onTicks) ? LOW : HIGH);
  cnt = (cnt + 1 >= period) ? 0 : cnt + 1;
}


// === STATUS HELPERS ===
static const char* modeToStr(Controller::Mode m) {
  switch (m) {
    case Controller::Mode::MANUAL:       return "MAN";
    case Controller::Mode::HEADING_LOCK: return "HL";
    case Controller::Mode::TARGET_LOCK:  return "TL";
    default:                             return "UNK";
  }
}

static const char* gpsHealthToStr(GPSManager::HealthState s) {
  switch (s) {
    case GPSManager::NO_FIX:    return "NO_FIX";
    case GPSManager::ACQUIRING: return "ACQ";
    case GPSManager::VALID:     return "VALID";
    case GPSManager::LOST:      return "LOST";
    default:                    return "UNK";
  }
}

// ST stream field legend:
//   0:now_ms  1:mode  2:gpsState  3:gps_ok  4:hAcc_m
//   5:boatHead_deg  6:rotHead_deg  7:TL_on
//   8:distToTarget_m  9:HL_target_deg  10:motorCmd
//   11:geoMagStatus(0-3)  12:rotVecHeading_deg  13:rotVecAccuracy_deg
//   14:rotVecHeadingAvg_deg  (accuracy-gated 10-sample circular mean)
// Fields 11-14 are for compass testing — compare 12/14 to a real compass,
// trust the reading only when 13 is below 10.0 degrees.
static void printStatusLine(uint32_t now_ms) {
  bool tl_on = targetLock.active();
  const char* modeStr = tl_on ? "TL" : modeToStr(controller.mode());

  GPSManager::HealthState ghs = gps.getHealth();
  const GPSManager::GPSEpoch& ge = gps.getData();
  bool gps_ok = gps.isHealthy() && ge.valid;

  float boatHead      = mag.getBoatHeading();
  float rotHead       = mag.getRotatorHeading(motor.getAngleDegCumulative());
  float distToTarget  = targetLock.distToTarget_m();
  float hlTarget      = headingLock.target();
  int16_t motorCmd    = motor.state();
  uint8_t geoMagSt    = mag.getGeoMagStatus();
  float rotVecHead    = mag.getRotVecHeading();
  float rotVecAcc     = mag.getRotVecAccuracyDeg();
  float rotVecAvg     = mag.getRotVecHeadingAvg();

  // Build the ST line and send to serial only.
  // BLE phone connection is config terminal only — no continuous streaming
  // to avoid interference with pedal central role responsiveness.
  char buf[180];
  snprintf(buf, sizeof(buf),
    "ST,%lu,%s,%s,%d,%.2f,%.1f,%.1f,%d,%.2f,%.1f,%d,%d,%.1f,%.1f,%.1f",
    (unsigned long)now_ms,
    modeStr,
    gpsHealthToStr(ghs),
    gps_ok ? 1 : 0,
    ge.hAcc,
    boatHead,
    rotHead,
    tl_on ? 1 : 0,
    distToTarget,
    hlTarget,
    (int)motorCmd,
    (int)geoMagSt,
    rotVecHead,
    rotVecAcc,
    rotVecAvg
  );

  Serial.println(buf);
}


void TimerHandler() {
  flag_100ms = true;
}


// ======================================================================
// SETUP
// ======================================================================
void setup() {
  Serial.begin(115200);

  #ifdef DEBUG_PRINTS
    Serial.println("Waiting for Serial...");
    unsigned long serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart < 3000)) {
      delay(10);
    }
  #endif

  #if HW_REV == 3
    pinMode(VSENSOR_EN, OUTPUT);
    digitalWrite(VSENSOR_EN, HIGH); //enable power supply for external sensor array
  #endif
  
  Serial.print("Version: ");
  Serial.println(APP_VERSION);
  delay(500);
  Serial.println("BLE Receiver Rotator Starting");
  checkDisableBLE();


  

  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(LED_PWRBTN_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);
  digitalWrite(LED_PWRBTN_PIN, LOW);



  analogReadResolution(12);
  analogWriteResolution(10);

  // --- Controller: load timing tunables from cfg ---
  controller.init(cfg);

  // --- BLE ---
  // Note: BLEManager::begin() now calls Bluefruit.begin(1, 1) to enable
  // one peripheral slot alongside the central role for the config service.
  if (bleEnabled) {
    ble.setTargetNamePrefix(BLE_TARGET_PREFIX);   // from config.h
    ble.setDeviceName(BLE_DEVICE_NAME);
    ble.setRssiFilter(CFG_BLE_RSSI_FILTER);
    ble.setTxPower(CFG_BLE_TX_POWER);
    ble.setActiveScan(CFG_BLE_ACTIVE_SCAN);

    ble.setCommandCallback([](int16_t cmd, uint32_t t_ms) {
      currentCommand = cmd;
      controller.onEvent(Controller::Event::PEDAL_MOVE, (float)cmd, t_ms);
      #if DEBUG_BLE_EVENTS
        static uint32_t seq = 0;
        Serial.print("[CMD] #"); Serial.print(++seq);
        Serial.print(" t_ms="); Serial.print(t_ms);
        Serial.print(" cmd="); Serial.println(cmd);
      #endif
    });

    ble.setStatusCallback([](bool ok) {
      if (!ok) { motor.drive(0); }
      #if CFG_ZERO_ON_PEDAL_CONNECT
      if (ok)  { motor.resetEncoder(); }
      #endif
      controller.onEvent(ok ? Controller::Event::BLE_RECOVER
                            : Controller::Event::BLE_LOSS);
      digitalWrite(LED_STATUS_PIN, ok ? HIGH : LOW);
    });

    ble.begin();

    // V12: start config peripheral after ble.begin() has initialised Bluefruit
    bleConfig.begin();
  }

  Wire.begin();
  delay(5);

  SYSLED_BEGIN();
  SYSLED_SHOW();

  // --- Motor: apply cfg tunables via setters ---
  motor.begin(0x60);
  motor.setAngleLimits(cfg.angleLimitDeg, cfg.angleHystDeg);
  motor.setMinPwm(cfg.minPwm);
  motor.setOvercurrentTrip(cfg.overcurrentMa);

  // --- Timer ---
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler)) {
    Serial.print(F("Starting ITimer OK, millis() = "));
    Serial.println(millis());
  }

  // --- Magnetometer init with retries ---
  // Fully optional — if not found system runs in MANUAL only.
  // BNO085 needs up to 500ms after power-on before responding — wait first.
  Serial.println("Initializing magnetometer...");
  delay(500);
  for (int attempt = 1; attempt <= 5; ++attempt) {
    if (mag.begin()) {
      magAvailable = true;
      break;
    }
    Serial.print("  Mag init failed (try ");
    Serial.print(attempt);
    Serial.println(") - retrying...");
    delay(200);
  }

  if (magAvailable) {
    mag.setOffset(cfg.magOffsetDeg);
    Serial.println("Magnetometer: OK");
  } else {
    Serial.println("Magnetometer: NOT FOUND — heading/target lock unavailable");
  }

  // --- GPS init — probes once at boot with retries, never again ---
  // gps.begin() handles all retry logic internally (GPS_BOOT_ATTEMPTS x GPS_BOOT_DELAY_MS).
  // ZED-F9P is slower to wake than BNO — retries spaced 400ms apart (see GPSManager.h).
  // If GPS not found, gps.update() becomes a no-op — zero I2C overhead.
  Serial.println("Initializing GPS...");
  gps.begin();
  if (gps.isAvailable()) {
    Serial.println("GPS: OK — acquiring fix");
  } else {
    Serial.println("GPS: NOT FOUND — target lock unavailable");
  }
}


// ======================================================================
// MAIN LOOP
// ======================================================================
void loop() {
  const uint32_t now = millis();

  ble.update();
  bleConfig.update();       // V12: drain BLE config RX, dispatch any commands
  processSerialCommand();
  gps.update();

  if (flag_100ms) {
    flag_100ms = false;

    // Only update sensors that were found at boot
    if (magAvailable) mag.update();
    updatePowerLed(bleEnabled && ble.isConnected());

    // Main control state machine
    controller.tick100ms(now, mag, motor, headingLock, gps, targetLock);

    #ifdef PRINT_HEADINGS
      float boat_heading = mag.getBoatHeading();
      if (boat_heading >= 0.0f) {
        Serial.print("Boat Heading: "); Serial.println(boat_heading, 1);
      } else {
        Serial.println("No heading available.");
      }
      float rotator_heading = mag.getRotatorHeading(motor.getAngleDegCumulative());
      Serial.print("Rotator Heading: "); Serial.println(rotator_heading, 1);
    #endif

    if (motor.isRunning()) {
      digitalWrite(LED_PWRBTN_PIN, HIGH);
      motor.updateCurrent();
    }
    #ifdef ENABLE_DOTSTAR
      updateSystemLED(motor.state(), controller.mode(), now);
    #endif

  } // end 100ms tick

  if (statusEnabled && (now - lastStatusPrint_ms >= STATUS_PERIOD_MS)) {
    printStatusLine(now);
    lastStatusPrint_ms = now;
  }

} // END OF MAIN LOOP


// ======================================================================
// SYSTEM LED
// ======================================================================
void updateSystemLED(int16_t command, Controller::Mode mode, uint32_t now_ms) {

  if (command == 0) {
    // Idle: show mode color
    uint8_t r = 0, g = 0, b = 0;
    switch (mode) {
      case Controller::Mode::MANUAL:       r = g = b = 40; break;  // dim white
      case Controller::Mode::HEADING_LOCK: b = 255;        break;  // blue
      case Controller::Mode::TARGET_LOCK:  r = 30; b = 255; break; // blue-violet
      default:                             break;
    }
    SYSLED_SET_BRIGHT(150);
    SYSLED_SET_PIXEL(g, r, b);
    SYSLED_SHOW();
    return;
  }

  // Active: brightness = speed, color = direction
  int c = constrain(abs(command), 0, 512);
  uint8_t brightness = map(c, 0, 512, 10, 255);
  uint8_t red   = (command > 0) ? 255 : 0;
  uint8_t green = (command < 0) ? 255 : 0;
  SYSLED_SET_BRIGHT(brightness);
  SYSLED_SET_PIXEL(red, green, 0);
  SYSLED_SHOW();
}


// ======================================================================
// BLE DISABLE CHECK  (runs once at boot, before BLE init)
// ======================================================================
void checkDisableBLE() {
  Serial.println("Press 'x' within 1.5 seconds to disable BLE...");
  unsigned long startTime = millis();
  while (millis() - startTime < 1500) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        bleEnabled = false;
        Serial.println("BLE disabled.");
        return;
      }
    }
    delay(100);
  }
}


// ======================================================================
// SERIAL COMMAND HANDLER
// Non-blocking line accumulator — no readStringUntil() blocking.
// Lines are trimmed and split into verb + optional argument before dispatch.
//
// Command reference:
//   A / a          — overcurrent trip +/- 25 mA
//   Z / z          — reset encoder to 0
//   L              — enable lock (TL if GPS valid+accurate, else HL)
//   l              — disable lock → manual
//   ? / H / h      — help
//   VER            — firmware version
//   INFO           — system snapshot
//   STATUS ON/OFF  — enable / suppress 4 Hz ST stream
//   GET ALL        — print all tunable cfg parameters
//   GET <param>    — print one parameter
//   SET <param> <value>  — update live cfg parameter (resets on reboot)
//
// SET/GET parameter tokens:
//   HL_KP          HeadingLock proportional gain
//   HL_DEAD        HeadingLock deadband (deg)
//   HL_MIN         HeadingLock min command
//   HL_CAP         HeadingLock max command
//   TL_DIST        TargetLock projection distance (m)
//   TL_HACC        TargetLock max GPS hAcc to arm (m)
//   TL_DRIFT       TargetLock min boat displacement before bearing update (m)
//   TL_ALPHA       TargetLock bearing low-pass filter coefficient
//   TL_MAXDIST     TargetLock max distance before cancel (m)
//   TL_GPSLOST     TargetLock GPS lost hold timeout (ms)
//   AUTOLOCK_MS    Controller idle time before auto-arm (ms)
//   TAKEOVER_THR   Controller pedal threshold to override lock
//   OVERCURRENT    Motor overcurrent trip (mA)
//   MAG_OFFSET     Magnetometer heading offset (deg)
// ======================================================================


// ---- Output helper -------------------------------------------------------
// cmdPrintln() sends a complete line to both serial and phone.
// Use in all command handlers so responses are mirrored consistently.

// Send a complete response line to both serial and phone.
static void cmdPrintln(const char* s) {
  Serial.println(s);
  bleConfig.sendLine(s);
}

// Convenience: print a String
static void cmdPrintln(const String& s) {
  cmdPrintln(s.c_str());
}

// ---- MAX_MODE helpers ----------------------------------------------------
static const char* maxModeStr(uint8_t m) {
  if (m == 0) return "MANUAL";
  if (m == 1) return "HL";
  return "TL";
}

// ---- GET handler ---------------------------------------------------------
// Prints one or all tunable cfg parameters.
// Called from dispatchLine() when verb == "GET".

static void handleGet(const String& token) {
  // Helper lambdas to print a single parameter line to both outputs.
  // snprintf %.*f handles float formatting — -u _printf_float is already
  // in the nRF52 build flags so this works without dtostrf.
  auto pf = [](const char* name, float val, int dp = 2) {
    char out[48];  snprintf(out, sizeof(out), "  %-14s %.*f", name, dp, val);
    Serial.println(out);
    bleConfig.sendLine(out);
  };
  auto pu = [](const char* name, uint32_t val) {
    char out[48]; snprintf(out, sizeof(out), "  %-14s %lu", name, (unsigned long)val);
    Serial.println(out);
    bleConfig.sendLine(out);
  };
  auto pi = [](const char* name, int val) {
    char out[48]; snprintf(out, sizeof(out), "  %-14s %d", name, val);
    Serial.println(out);
    bleConfig.sendLine(out);
  };

  bool all = (token == "ALL" || token.length() == 0);

  if (all) { cmdPrintln("--- CFG ---"); }

  if (all || token == "HL_KP")       pf("HL_KP",       cfg.hlKp,               2);
  if (all || token == "HL_DEAD")     pf("HL_DEAD",      cfg.hlDeadDeg,          2);
  if (all || token == "HL_MIN")      pi("HL_MIN",       cfg.hlMinCmd);
  if (all || token == "HL_CAP")      pi("HL_CAP",       cfg.hlCapCmd);
  if (all || token == "TL_DIST")     pf("TL_DIST",      cfg.tlDistanceM,        2);
  if (all || token == "TL_HACC")     pf("TL_HACC",      cfg.tlMaxHAccM,         2);
  if (all || token == "TL_DRIFT")    pf("TL_DRIFT",     cfg.tlMinDriftM,        2);
  if (all || token == "TL_ALPHA")    pf("TL_ALPHA",     cfg.tlBearingAlpha,     3);
  if (all || token == "TL_MAXDIST")  pf("TL_MAXDIST",   cfg.tlMaxDistToTargetM, 1);
  if (all || token == "TL_GPSLOST")  pu("TL_GPSLOST",   cfg.tlGpsLostTimeoutMs);
  if (all || token == "AUTOLOCK_MS") pu("AUTOLOCK_MS",  cfg.ctrlAutoLockMs);
  if (all || token == "TAKEOVER_THR")pi("TAKEOVER_THR", cfg.ctrlTakeoverThr);
  if (all || token == "OVERCURRENT") pi("OVERCURRENT",  (int)cfg.overcurrentMa);
  if (all || token == "MAG_OFFSET")  pf("MAG_OFFSET",   cfg.magOffsetDeg,       2);
  if (all || token == "MAX_MODE") {
    char out[48]; snprintf(out, sizeof(out), "  %-14s %s (%u)", "MAX_MODE", maxModeStr(cfg.maxMode), cfg.maxMode);
    Serial.println(out); bleConfig.sendLine(out);
  }

  // If a specific token was requested but matched nothing, say so
  if (!all) {
    // Re-check by running again — if nothing printed, it was unknown.
    // Simpler: just flag unknown here directly.
    bool known =
      token == "HL_KP"       || token == "HL_DEAD"     ||
      token == "HL_MIN"      || token == "HL_CAP"      ||
      token == "TL_DIST"     || token == "TL_HACC"     ||
      token == "TL_DRIFT"    || token == "TL_ALPHA"    ||
      token == "TL_MAXDIST"  || token == "TL_GPSLOST"  ||
      token == "AUTOLOCK_MS" || token == "TAKEOVER_THR"||
      token == "OVERCURRENT" || token == "MAG_OFFSET"  ||
      token == "MAX_MODE";
    if (!known) {
      String msg = F("Unknown param: "); msg += token;
      cmdPrintln(msg);
    }
  }
}

// ---- SET handler ---------------------------------------------------------
// Updates one live cfg parameter and applies it immediately to the running
// subsystem where applicable.  Changes are NOT persisted — reboot resets
// to config.h defaults.  Note the working values when tuning on the water
// and hardcode them in config.h when you get home.

static void handleSet(const String& token, const String& valStr) {
  if (token.length() == 0 || valStr.length() == 0) {
    cmdPrintln("Usage: SET <param> <value>");
    return;
  }

  float fval = valStr.toFloat();
  long  lval = valStr.toInt();

  bool applied = true;

  // HeadingLock params — update cfg then rebuild HL params live
  if (token == "HL_KP") {
    cfg.hlKp = fval;
    // Rebuild HeadingLock with updated params — takes effect next tick()
    HeadingLock::Params p = makeHLParams();
    headingLock = HeadingLock(p);

  } else if (token == "HL_DEAD") {
    cfg.hlDeadDeg = fval;
    HeadingLock::Params p = makeHLParams();
    headingLock = HeadingLock(p);

  } else if (token == "HL_MIN") {
    cfg.hlMinCmd = (int16_t)lval;
    HeadingLock::Params p = makeHLParams();
    headingLock = HeadingLock(p);

  } else if (token == "HL_CAP") {
    cfg.hlCapCmd = (int16_t)lval;
    HeadingLock::Params p = makeHLParams();
    headingLock = HeadingLock(p);

  // TargetLock params — update cfg then rebuild TL params live
  } else if (token == "TL_DIST") {
    cfg.tlDistanceM = fval;
    TargetLock::Params p = makeTLParams();
    targetLock = TargetLock(p);

  } else if (token == "TL_HACC") {
    cfg.tlMaxHAccM = fval;
    TargetLock::Params p = makeTLParams();
    targetLock = TargetLock(p);

  } else if (token == "TL_DRIFT") {
    cfg.tlMinDriftM = fval;
    TargetLock::Params p = makeTLParams();
    targetLock = TargetLock(p);

  } else if (token == "TL_ALPHA") {
    cfg.tlBearingAlpha = fval;
    TargetLock::Params p = makeTLParams();
    targetLock = TargetLock(p);

  } else if (token == "TL_MAXDIST") {
    cfg.tlMaxDistToTargetM = fval;
    TargetLock::Params p = makeTLParams();
    targetLock = TargetLock(p);

  } else if (token == "TL_GPSLOST") {
    cfg.tlGpsLostTimeoutMs = (uint32_t)lval;
    TargetLock::Params p = makeTLParams();
    targetLock = TargetLock(p);

  // Controller timing — update cfg then re-init controller live
  } else if (token == "AUTOLOCK_MS") {
    cfg.ctrlAutoLockMs = (uint32_t)lval;
    controller.init(cfg);   // re-applies all controller tunables

  } else if (token == "TAKEOVER_THR") {
    cfg.ctrlTakeoverThr = (int)lval;
    controller.init(cfg);

  // Motor overcurrent — applied immediately via setter
  } else if (token == "OVERCURRENT") {
    cfg.overcurrentMa = (uint16_t)lval;
    motor.setOvercurrentTrip(cfg.overcurrentMa);

  // Magnetometer offset — applied immediately via setter
  } else if (token == "MAG_OFFSET") {
    cfg.magOffsetDeg = fval;
    mag.setOffset(cfg.magOffsetDeg);

  // Mode ceiling — applied immediately to controller
  } else if (token == "MAX_MODE") {
    String v = valStr; v.toUpperCase(); v.trim();
    uint8_t m;
    if      (v == "MANUAL" || v == "0") m = 0;
    else if (v == "HL"     || v == "1") m = 1;
    else if (v == "TL"     || v == "2") m = 2;
    else {
      cmdPrintln("MAX_MODE: use MANUAL, HL, or TL  (or 0, 1, 2)");
      return;
    }
    cfg.maxMode = m;
    controller.setMaxMode(m);

  } else {
    applied = false;
    String msg = F("Unknown param: "); msg += token;
    cmdPrintln(msg);
    cmdPrintln("Type GET ALL for valid params");
  }

  if (applied) {
    // Echo the new value back so you can confirm it was accepted
    handleGet(token);
  }
}


// ---- printHelp -----------------------------------------------------------
static void printHelp() {
  // Send to both serial and phone
  const char* lines[] = {
    "--- Commands ---",
    "  A / a            overcurrent +/- 25 mA",
    "  Z / z            reset encoder",
    "  VER              firmware version",
    "  INFO             system snapshot",
    "  STATUS ON/OFF    enable/suppress 4 Hz status stream",
    "  GET ALL          show all tunable parameters",
    "  GET <param>      show one parameter",
    "  SET <param> <v>  update live parameter (resets on reboot)",
    "  SET MAX_MODE     TL | HL | MANUAL  (or 2|1|0) — mode ceiling",
    "  ? / H / h        this help"
  };
  for (const char* l : lines) { cmdPrintln(l); }
}


// ---- printInfo -----------------------------------------------------------
static void printInfo() {
  uint32_t now_ms = millis();
  const GPSManager::GPSEpoch& ge = gps.getData();
  float boatHead = mag.getBoatHeading();
  float rotHead  = mag.getRotatorHeading(motor.getAngleDegCumulative());

  char buf[64];

  cmdPrintln("--- INFO ---");

  snprintf(buf, sizeof(buf), "  Version       : %s", APP_VERSION);  cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Uptime ms     : %lu", (unsigned long)now_ms); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Mag avail     : %s", magAvailable ? "yes" : "no"); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  GPS avail     : %s", gps.isAvailable() ? "yes" : "no"); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Mode          : %s", modeToStr(controller.mode())); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  BLE ok        : %s", controller.bleOk() ? "yes" : "no"); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  GPS state     : %s", gpsHealthToStr(gps.getHealth())); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  GPS hAcc m    : %.2f", ge.hAcc);   cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  GPS numSV     : %d",  ge.numSV);   cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Boat head     : %.1f", boatHead);  cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Rot head      : %.1f", rotHead);   cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  HL target     : %.1f", headingLock.target()); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  TL active     : %s", targetLock.active() ? "yes" : "no"); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  TL dist tgt m : %.2f", targetLock.distToTarget_m()); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  TL boat delta : %.2f", targetLock.boatDelta_m()); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Overcurr mA   : %d",  motor.getOvercurrentTrip_mA()); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Motor cmd     : %d",  motor.state()); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Encoder deg   : %.1f", motor.getAngleDegCumulative()); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Status strm   : %s", statusEnabled ? "ON" : "OFF"); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  Phone conn    : %s", bleConfig.isConnected() ? "yes" : "no"); cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  GeoMag status : %d",   mag.getGeoMagStatus());           cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  RotVec head   : %.1f", mag.getRotVecHeading());            cmdPrintln(buf);
  snprintf(buf, sizeof(buf), "  RotVec acc deg: %.1f", mag.getRotVecAccuracyDeg());        cmdPrintln(buf);
}


// ---- dispatchLine --------------------------------------------------------
// Handles a fully-assembled trimmed command line from either serial or
// the BLE config service.  Both paths call this function — no duplication.

void dispatchLine(String& line) {
  line.trim();
  if (line.length() == 0) return;

  // Split into verb (first token) and the rest
  int spaceIdx = line.indexOf(' ');
  String verb = (spaceIdx < 0) ? line : line.substring(0, spaceIdx);
  String arg  = (spaceIdx < 0) ? String("") : line.substring(spaceIdx + 1);
  arg.trim();
  verb.toUpperCase();   // case-insensitive verbs

  // --- Single-char legacy commands (preserve original case for A/a Z/z L/l) ---
  if (verb.length() == 1) {
    char c = line.charAt(0);
    switch (c) {
      case 'A':
        motor.setOvercurrentTrip(min<uint16_t>(motor.getOvercurrentTrip_mA() + 25, 3000));
        { char buf[32]; snprintf(buf, sizeof(buf), "Overcurrent: %d mA", motor.getOvercurrentTrip_mA()); cmdPrintln(buf); }
        return;

      case 'a': {
        uint16_t cur = motor.getOvercurrentTrip_mA();
        motor.setOvercurrentTrip((cur >= 25) ? cur - 25 : 0);
        char buf[32]; snprintf(buf, sizeof(buf), "Overcurrent: %d mA", motor.getOvercurrentTrip_mA()); cmdPrintln(buf);
      } return;

      case 'Z': case 'z':
        motor.resetEncoder();
        cmdPrintln("Encoder reset to 0");
        return;

      case '?': case 'H': case 'h':
        printHelp();
        return;

      default:
        { String msg = F("Unknown command: "); msg += line; cmdPrintln(msg); }
        return;
    }
  }

  // --- Word commands ---
  if (verb == "VER") {
    char buf[24]; snprintf(buf, sizeof(buf), "Version: %s", APP_VERSION);
    cmdPrintln(buf);

  } else if (verb == "INFO") {
    printInfo();

  } else if (verb == "STATUS") {
    arg.toUpperCase();
    if (arg == "ON") {
      statusEnabled = true;
      cmdPrintln("Status stream ON");
    } else if (arg == "OFF") {
      statusEnabled = false;
      cmdPrintln("Status stream OFF");
    } else {
      cmdPrintln("Usage: STATUS ON | STATUS OFF");
    }

  } else if (verb == "GET") {
    // arg is the parameter token, or "ALL"
    arg.toUpperCase();
    handleGet(arg);

  } else if (verb == "SET") {
    // arg is "<param> <value>" — split once more
    arg.toUpperCase();
    int sp2 = arg.indexOf(' ');
    if (sp2 < 0) {
      cmdPrintln("Usage: SET <param> <value>");
    } else {
      String param = arg.substring(0, sp2);
      String val   = arg.substring(sp2 + 1);
      val.trim();
      handleSet(param, val);
    }

  } else {
    String msg = F("Unknown command: "); msg += line;
    cmdPrintln(msg);
    cmdPrintln("Type ? for help");
  }
}


// ---- processSerialCommand ------------------------------------------------
void processSerialCommand() {
  // Non-blocking accumulator: collect chars until '\n', then dispatch
  static char lineBuf[64];
  static uint8_t lineLen = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;            // ignore CR (handles \r\n terminals)
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      String line(lineBuf);
      lineLen = 0;
      dispatchLine(line);
      return;                           // one line per loop() pass
    }
    if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
    // silently drop overflow chars — keeps buffer clean
  }
}
