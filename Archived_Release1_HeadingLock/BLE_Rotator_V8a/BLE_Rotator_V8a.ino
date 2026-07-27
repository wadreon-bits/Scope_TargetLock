// === BLE_RECEIVER_ROTATOR.ino ===
// BLE Receiver Rotator for Adafruit nRF52840 (Feather or Itsybitsy)
// Connects to BLE controller pedals named "BLE_Controller_Pedal_<#>"
// Uses: DRV_IN1_PIN (D5), DRV_IN2_PIN (D12), DRV_SLP_PIN (D9)
//V2 Added Current Measurement, Overcurrent Adjustment - Working tested with Pedal_V1
//V3 Added Encoder read, full quadrature, added angle resolution, added goto angle serial cmnd, added angle limits  (Pedal_V1)
//V4 Beginning to add in targeting features - starting with BNO085 magnetometer - Geomagnetic vector
//V5 Have mag working with sign issues -  Changed to using Game Rotation Vector. 
//V6 Working heading tracking -Slow control loop
//V6a - Working updated heading target Kp to 14. Motor control not class yet
//V6b working on class for MotorController. 
//V7a created BLE class- migrated and tested working so far, logic Errors. Logic cleaned in 7A 
//V7b heading lock class added, testing working so far. Next is Main state machine, and command handler. 
//     Seems to be working well. Learned about heading frame- relative position when waking up. Rotator and Mag zero 
//      At boot- thus tracking movement from that point on. Slowly corrected for magnetic north with filter. 
//      Should still work as the eventual target math will be made relative to rotator 
//V8  Adding controller class- Working with gaurds in main loop. Minor Testing working Pedal V1
//V8A Updated Controller Class with Step 2 
// NOTE!! Changed Pedal BLE advertised name to 'Scope_Pedal_1'.  

#include <bluefruit.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h> // DAC Library
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "NRF52TimerInterrupt.h"
#include <Adafruit_DotStar.h>
#include <Adafruit_BNO08x.h>
#include "Magnetometer.h"
#include "MotorController.h"
#include "BLEManager.h"
#include "HeadingLock.h"
#include "SController.h"

#define APP_VERSION     "8.0" //build into ble read values
//#define DEBUG_PRINTS
#define PRINT_HEADINGS
#define DEBUG_BLE_EVENTS 1
//#define PRINT_BLE_CMDS
#define ENABLE_DOTSTAR  
// === Pin Definitions ===
#define DRV_IN1_PIN       13   //PIN
#define DRV_IN2_PIN       12  //PIN
#define ENC_A_PIN         11  //might not match pcb silk reversed in code to clean up sign issues
#define ENC_B_PIN         10  //might not match pcb silk reversed to clean up sign issues
//#define DRV_CS_PIN      A0
#define DRV_SLP_PIN       9   //PIN 
#define DRV_FLT_PIN       7   //PIN
#define LED_STATUS_PIN    3   //PIN Onboard LED for BLE status
#define LED_PWRBTN_PIN    2   //PIN
#define IPROPI_PIN        A0

#define DOTSTAR_DATA_PIN  8
#define DOTSTAR_CLK_PIN   6

// === CONSTANTS ===
#define HW_TIMER_INTERVAL_US      100000  // timer interval in microseconds

// --- System State ---
bool goToAngleActive = false;         // move to state vars
bool flag_100ms = false;
int bleEnabled = true;                //can disable via serial interface
int currentCommand = 0;               // Last received motor command

//** GEARMOTOR ENCODER VARIABLES **//
const float COUNTS_PER_REV      = 16940.0f;  // 11 PPR × 4 edges × 110:1 x 3.5:1  gear ROUGHLY A 17k counts/rev encoder

//** GOTO ANGLE VARS **//
float targetAngleDeg = 0;
extern const float angleErrorThreshold = 0.1;  // Acceptable error margin in degrees
extern const int maxGotoSpeed = 400;          // Adjust speed for seeking

//** HEADING LOCK - MAGNETOMETER VARS **//
float mag_offset = 0.0; 

//** INSTANTIATE CLASSES **//
NRF52Timer ITimer(NRF_TIMER_3); // Init NRF52 timer NRF_TIMER1
Adafruit_DotStar dotstar(1, DOTSTAR_DATA_PIN, DOTSTAR_CLK_PIN, DOTSTAR_BGR); // Init DotStar
Magnetometer mag(mag_offset); // Init Magnetometer
MotorController::Pins motorPins{ DRV_IN1_PIN, DRV_IN2_PIN, DRV_SLP_PIN, DRV_FLT_PIN, IPROPI_PIN, ENC_A_PIN, ENC_B_PIN };
MotorController motor(motorPins, COUNTS_PER_REV);
BLEManager ble;
HeadingLock headingLock;
Controller controller;


// --- Small utilities ---
static inline float wrap180(float x){ x = fmodf(x+180.f, 360.f); if(x<0)x+=360.f; return x-180.f; }
// LED patterns
static inline void updatePowerLed(bool connected){
  // 100ms tick assumed
  static uint8_t cnt = 0;
  const uint8_t onTicks  = connected ? 29 : 3;   // tune as you had
  const uint8_t period   = connected ? 30 : 6;

  digitalWrite(LED_PWRBTN_PIN, (cnt < onTicks) ? LOW : HIGH);
  cnt = (cnt+1 >= period) ? 0 : cnt+1;
}

void TimerHandler(){
  flag_100ms = true;
}

// === Setup ===
void setup() {
  //Bluefruit.configClearStorage();
  Serial.begin(115200);

  #ifdef DEBUG_PRINTS
    // Optional: tell the user we’re waiting
    Serial.println("Waiting for Serial...");

    unsigned long serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart < 3000)) {
      delay(10);  // avoid tight spinning
    }
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

  analogReadResolution(12);     // 12-bit ADC (0–4095)
  analogWriteResolution(10);  // Set resolution to 10 bits (0–1023)

  if (bleEnabled) {
    ble.setTargetNamePrefix("Scope_Pedal_1");
    ble.setRssiFilter(-80);
    ble.setActiveScan(true);
    ble.setTxPower(4);
    ble.setDeviceName("Scope_Rotator");

    ble.setCommandCallback([](int16_t cmd, uint32_t t_ms) {
      currentCommand  = cmd;
      // NEW: always notify controller; it decides takeover vs idle
      controller.onEvent(Controller::Event::PEDAL_MOVE, (float)cmd, t_ms);
      #if DEBUG_BLE_EVENTS
        static uint32_t seq = 0;
        Serial.print("[CMD] #"); Serial.print(++seq);
        Serial.print(" t_ms="); Serial.print(t_ms);
        Serial.print(" cmd="); Serial.println(cmd);   // <- should show zeros too
      #endif
    });

    ble.setStatusCallback([](bool ok) {
      if (!ok) { motor.drive(0); } // keep this hard failsafe
      controller.onEvent(ok ? Controller::Event::BLE_RECOVER
                            : Controller::Event::BLE_LOSS);
      digitalWrite(LED_STATUS_PIN, ok ? HIGH : LOW);
    });

    ble.begin();
  }
  Wire.begin();
  delay(5);
  dotstar.begin();      // Initialize DotStar
  dotstar.show();       // Turn off initially
  motor.begin(0x60);
  motor.setAngleLimits(270.0f, 1.0f);
  motor.setMinPwm(50);
  motor.setOvercurrentTrip(1300); // same default you had
  // Interval in microsecs
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = "));
    Serial.println(millis());
  }
  // --- Magnetometer init with retries ---
  bool magReady = false;
  Serial.println("Initializing magnetometer...");
  for (int attempt = 1; attempt <= 5; ++attempt) {
    if (mag.begin()) {
      Serial.println("✅ Magnetometer ready");
      magReady = true;
      break;
    }
    Serial.print("❌ Mag init failed (try ");
    Serial.print(attempt);
    Serial.println(") – retrying...");
    delay(200); // short pause before retry
  }

  if (!magReady) {
    Serial.println("⚠️ Magnetometer not ready – continuing without it");
    // We’ll skip heading lock until mag.sensorsReady() returns true
  }
  delay(500);
  Serial.println("✅ Magnetometer ready");
  //install enable switch to power line and toggle here before blocking if no mag
}

// === Main Loop ===
void loop() {
  ble.update();
  processSerialCommand();

  const uint32_t now = millis();
  const bool ble_link  = bleEnabled && ble.isConnected();
  //const bool rx_recent = ((int32_t)(now - lastCommandTime) < 500);   // saw pedal lately

  if (flag_100ms) {
    flag_100ms = false;

    mag.update();
    updatePowerLed(bleEnabled && ble.isConnected());
    //MAIN CONTROL STATE MACHINE CALL 
    controller.tick100ms(now, mag, motor, headingLock);

    #ifdef PRINT_HEADINGS
      float boat_heading = mag.getBoatHeading();
      if (boat_heading >= 0.0f) {
        Serial.print("Boat Heading: "); Serial.println(boat_heading, 1);
      } else {
        Serial.println("⚠️ No heading available.");
      }
      float rotator_heading = mag.getRotatorHeading(motor.getAngleDegCumulative());
      Serial.print("Rotator Heading: "); Serial.println(rotator_heading, 1);
    #endif

    if (motor.isRunning()) {
      digitalWrite(LED_PWRBTN_PIN, HIGH);
      motor.updateCurrent();
      #ifdef ENABLE_DOTSTAR
        updateDotStar(motor.state());
      #endif
    } else {
      #ifdef ENABLE_DOTSTAR
        updateDotStar(0);
      #endif
    }
  } // 100ms tick end
} //!!!!!!!!!!!!!!!!!!!!!!!END OF MAIN LOOP!!!!!!!!!!!!!!!!!!!!!!!!!!

void updateDotStar(int16_t command) {
  if (command == 0) {
    dotstar.setBrightness(0);  // Force off
    dotstar.setPixelColor(0, 0, 0, 0);
  } else {
    uint8_t brightness = map(abs(command), 0, 512, 10, 255); // Scale brightness
    dotstar.setBrightness(brightness);
    uint8_t red = (command > 0) ? 255 : 0;
    uint8_t green = (command < 0) ? 255 : 0;
    dotstar.setPixelColor(0, red, green, 0);
  }
  dotstar.show();
}

void checkDisableBLE() {
  Serial.println("Press 'x' within 1.5 seconds to disable BLE...");
  unsigned long startTime = millis();
  while (millis() - startTime < 1500) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        bleEnabled = false;
        Serial.println("BLE disabled by user.");
        return;
      }
    }
    delay(100);
  }
}

void processSerialCommand() {
  if (!Serial.available()) return;

  char command = Serial.read();
  String argument = Serial.readStringUntil('\n');  // Read until end of line
  argument.trim();  // Clean whitespace

  switch (command) {
    case 'A':
      motor.setOvercurrentTrip(min<uint16_t>(motor.getOvercurrentTrip_mA() + 25, 3000));
      Serial.print("Overcurrent setpoint updated: ");
      Serial.print(motor.getOvercurrentTrip_mA());
      Serial.println(" mA");
      break;
    case 'a':
      {
        uint16_t cur = motor.getOvercurrentTrip_mA();
        motor.setOvercurrentTrip((cur >= 25) ? cur - 25 : 0);
        Serial.print("Overcurrent setpoint updated: ");
        Serial.print(motor.getOvercurrentTrip_mA());
        Serial.println(" mA");
      }
      break;

    case 'z':  // Reset encoder count
    case 'Z':
      noInterrupts();
      motor.resetEncoder();
      interrupts();
      Serial.println("Encoder angle reset to 0°");
      break;

    case 'g':
    case 'G':  // Go to absolute angle
      targetAngleDeg = argument.toFloat();
      //goToAngleActive = true;
      controller.onEvent(Controller::Event::GOTO_SET, targetAngleDeg);
      Serial.print("Go to angle: ");
      Serial.println(targetAngleDeg, 2);
      break;

    // case 'h':
    //   targetAngleDeg = 0.0;
    //   goToAngleActive = true;
    //   Serial.print("Homing");
    //   break;
    case 'H': {  // Enable heading lock to explicit target
      float tgt = argument.toFloat();
      if (!(tgt >= 0.0f && tgt <= 360.0f)) {
        Serial.println("⚠️ Invalid heading. Must be 0–360.");
      } else {
        controller.onEvent(Controller::Event::HL_SET_TARGET, tgt);;
        Serial.print("🔒 Heading lock enabled to ");
        Serial.print(headingLock.target(), 1);
        Serial.println("°");
      }
    } break;

    case 'h':  // Disable heading lock
      controller.onEvent(Controller::Event::HL_OFF);
      Serial.println("🟢 Heading lock disabled");
      break;

    case 'u': controller.onEvent(Controller::Event::AUTOLOCK_OFF); Serial.println("Auto-arm OFF"); break;
    case 'U': controller.onEvent(Controller::Event::AUTOLOCK_ON);  Serial.println("Auto-arm ON");  break;

    default:
      Serial.print("Unknown command: ");
      Serial.println(command);
      break;
  }
}






