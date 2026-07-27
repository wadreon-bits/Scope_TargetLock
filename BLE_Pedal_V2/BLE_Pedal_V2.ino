// === BLE_CONTROLLER_PEDAL.ino ===
// Controller Side - Feather nRF52840 / Seeed XIAO nRF52840
// Sends direction/speed commands over BLE to receiver based on pedal input
// Features: moving average filter, BLE UART, BLE config (deadband, scale, calibration), low power mode, nonlinear scaling response, serial debugging output
//0.3 for a super-flat center   0.6 for a snappier feel   1.0 for linear
//1000 = 1.0 (linear) 500 = 0.5 (smoother curve)  1500 = 1.5 (more aggressive curve)
//V1 - Main release in 2025 / early 2026
//V2 - Added improved sleep profile. When not connected -10s scan interval. 120s of fast scan before slow scan. Still Scope_Pedal_1
//V2 - Added multi-board support: Adafruit Feather nRF52840 and Seeed XIAO nRF52840
//
// ============================================================
// CRITICAL: BOARD PACKAGE VERSION REQUIREMENTS
// ============================================================
// Adafruit Feather nRF52840:
//   - Use Adafruit nRF52 board package (any current version)
//   - Bluefruit library is bundled — no issues
//
// Seeed XIAO nRF52840:
//   - MUST use Seeed nRF52 board package version 1.0.0 ONLY
//   - DO NOT update the Seeed package beyond 1.0.0
//   - Versions 1.1.3 and above removed Bluefruit library support
//     and will cause compile errors (missing bluefruit.h)
//   - Arduino IDE will prompt you to update — IGNORE IT
//   - In Boards Manager: Seeeduino → Seeed nRF52 Boards → 1.0.0
//
// Also note: Seeed package 1.0.0 does not handle paths with spaces
// correctly. If your sketch is in a path with spaces (e.g. OneDrive),
// create a junction to a no-spaces path:
//   mklink /J C:\Arduino\BLE_Pedal_V2 "C:\path with spaces\BLE_Pedal_V2"
// Then open and compile via the junction path for the Seeed board.
// ============================================================


#include <bluefruit.h>
#include <math.h>  // Required for nonlinear response curve


#define APP_VERSION         "2.0"     //build into ble read values
#define HOUSING_REVD
// --- Board-specific pin and LED definitions ---
// To revert: replace this block with:
//   #define PEDAL_PIN      A0
//   #define LED_STATUS_PIN 3
// And replace LED_ON->HIGH and LED_OFF->LOW in all LED digitalWrite calls
#if defined(ARDUINO_NRF52840_FEATHER)
  #define PEDAL_PIN         A0
  #define LED_STATUS_PIN    3           // pin 3, unchanged from original
  // LED_CONN already defined as pin 4 by Adafruit board package
  #define LED_ON            HIGH
  #define LED_OFF           LOW
#elif defined(ARDUINO_Seeed_XIAO_nRF52840)
  #define PEDAL_PIN         A5          // D5 / AIN3
  #define LED_STATUS_PIN    LED_RED     // pin 11
  #define LED_CONN          LED_BLUE    // pin 12 — not defined in Seeed package
  #define LED_ON            LOW         // Seeed LEDs are active LOW
  #define LED_OFF           HIGH
#else
  #error "Unsupported board — add pin definitions for this target"
#endif
#define SAMPLES             10      // Number of analog samples for moving average
#define UPDATE_INTERVAL_MS  50      // BLE update interval when active
#define LOWPOWER_INTERVAL_MS 1000   // BLE update interval in low power mode
#define IDLE_TIMEOUT_MS     600000  // Time (10 min) before entering low power mode
#define FAST_ADV_DURATION_MS 120000UL // Must match setFastTimeout(120) in ms

// --- BLE Services and Characteristics ---
BLEUart bleuart;                                 // BLE UART communication
BLEService configService = BLEService(0x1234);   // Custom BLE service for config parameters

// Configuration BLE characteristics (using uint16_t for compatibility with nRF Connect)
BLECharacteristic charDeadband = BLECharacteristic("6E400010-B5A3-F393-E0A9-E50E24DCCA99");
BLECharacteristic charScale = BLECharacteristic("6E400011-B5A3-F393-E0A9-E50E24DCCA99");
BLECharacteristic charResponseCurve = BLECharacteristic("6E400012-B5A3-F393-E0A9-E50E24DCCA99");
BLECharacteristic charPedalMin = BLECharacteristic("6E400013-B5A3-F393-E0A9-E50E24DCCA99");
BLECharacteristic charPedalNeutral = BLECharacteristic("6E400014-B5A3-F393-E0A9-E50E24DCCA99");
BLECharacteristic charPedalMax = BLECharacteristic("6E400015-B5A3-F393-E0A9-E50E24DCCA99");

// --- State Variables ---
int samples[SAMPLES];           // Circular buffer for moving average
int sampleIndex = 0;            // Index for circular buffer
unsigned long lastUpdate = 0;        // Timestamp of last BLE update
unsigned long lastActive = 0;        // Timestamp of last non-neutral activity
unsigned long advertisingStartTime = 0;  // Timestamp when advertising last started
bool inLowPower = false;        // True if controller is in low power mode
bool connected = false;         // BLE connection status

// --- Configurable Parameters (internal variables) ---
int deadband = 7;               // ADC deviation range for neutral zone
int scale = 1000;               // Sensitivity scale in thousandths (e.g., 1000 = 1.0)
uint16_t responseCurveX1000 = 500;  // default = 0.5

#ifdef HOUSING_REVB
int pedalMin = 387;             // ADC value at full reverse pedal
int pedalNeutral = 434;         // ADC value when pedal is centered
int pedalMax = 481;             // ADC value at full forward pedal
#endif
#ifdef HOUSING_REVD
int pedalMin = 440;             // ADC value at full reverse pedal
int pedalNeutral = 486;         // ADC value when pedal is centered
int pedalMax = 528;             // ADC value at full forward pedal
#endif


void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.print("Version: ");
  Serial.println(APP_VERSION);
  //delay(100);
  pinMode(PEDAL_PIN, INPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);

#if defined(ARDUINO_Seeed_XIAO_nRF52840)
  pinMode(22, OUTPUT);      //SET CHARGE CURRENT TO 100mA
  digitalWrite(22, LOW);
#endif


  // --- Initialize BLE ---
  Bluefruit.begin();
  Bluefruit.autoConnLed(false);       // Take manual control of the blue BLE LED
  pinMode(LED_CONN, OUTPUT);
  digitalWrite(LED_CONN, LED_OFF);
  Bluefruit.setTxPower(+4); // Set transmit power
  Bluefruit.setName("Scope_Pedal_1");
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addName(); 
  //Bluefruit.Periph.setConnectable(true);
  bleuart.begin();

  // --- Configuration Service & Characteristics ---
  configService.begin();

  // Deadband characteristic (uint16_t)
  charDeadband.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  charDeadband.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  charDeadband.setFixedLen(sizeof(uint16_t));
  charDeadband.setWriteCallback([](uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
    if (len == sizeof(uint16_t)) {
      uint16_t temp;
      memcpy(&temp, data, sizeof(uint16_t));
      deadband = (int)temp;
    }
  });
  charDeadband.setUserDescriptor("Deadband");
  charDeadband.begin();
  uint16_t deadbandInit = deadband;
  charDeadband.write((uint8_t*)&deadbandInit, sizeof(uint16_t));

  // Scale characteristic (uint16_t x1000)
  charScale.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  charScale.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  charScale.setFixedLen(sizeof(uint16_t));
  charScale.setWriteCallback([](uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
    if (len == sizeof(uint16_t)) {
      uint16_t temp;
      memcpy(&temp, data, sizeof(uint16_t));
      scale = (int)temp;
    }
  });
  charScale.setUserDescriptor("Scale x1000");
  charScale.begin();
  uint16_t scaleInit = scale;
  charScale.write((uint8_t*)&scaleInit, sizeof(uint16_t));

  //ResponseCurve (uint16_t x1000)
  charResponseCurve.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  charResponseCurve.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  charResponseCurve.setFixedLen(sizeof(uint16_t));
  charResponseCurve.setWriteCallback([](uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
    if (len == sizeof(uint16_t)) {
      uint16_t temp;
      memcpy(&temp, data, sizeof(uint16_t));
      if (temp >= 100 && temp <= 5000) {  // valid range: 0.1 to 5.0
        responseCurveX1000 = temp;
      }
    }
  });
  charResponseCurve.setUserDescriptor("Response Curve x1000");
  charResponseCurve.begin();
  charResponseCurve.write((uint8_t*)&responseCurveX1000, sizeof(uint16_t));



  // Pedal Min characteristic (uint16_t)
  charPedalMin.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  charPedalMin.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  charPedalMin.setFixedLen(sizeof(uint16_t));
  charPedalMin.setWriteCallback([](uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
    if (len == sizeof(uint16_t)) {
      uint16_t temp;
      memcpy(&temp, data, sizeof(uint16_t));
      pedalMin = (int)temp;
    }
  });
  charPedalMin.setUserDescriptor("Pedal Min");
  charPedalMin.begin();
  uint16_t pedalMinInit = pedalMin;
  charPedalMin.write((uint8_t*)&pedalMinInit, sizeof(uint16_t));

  // Pedal Neutral characteristic (uint16_t)
  charPedalNeutral.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  charPedalNeutral.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  charPedalNeutral.setFixedLen(sizeof(uint16_t));
  charPedalNeutral.setWriteCallback([](uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
    if (len == sizeof(uint16_t)) {
      uint16_t temp;
      memcpy(&temp, data, sizeof(uint16_t));
      pedalNeutral = (int)temp;
    }
  });
  charPedalNeutral.setUserDescriptor("Pedal Neutral");
  charPedalNeutral.begin();
  uint16_t pedalNeutralInit = pedalNeutral;
  charPedalNeutral.write((uint8_t*)&pedalNeutralInit, sizeof(uint16_t));

  // Pedal Max characteristic (uint16_t)
  charPedalMax.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  charPedalMax.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  charPedalMax.setFixedLen(sizeof(uint16_t));
  charPedalMax.setWriteCallback([](uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
    if (len == sizeof(uint16_t)) {
      uint16_t temp;
      memcpy(&temp, data, sizeof(uint16_t));
      pedalMax = (int)temp;
    }
  });
  charPedalMax.setUserDescriptor("Pedal Max");
  charPedalMax.begin();
  uint16_t pedalMaxInit = pedalMax;
  charPedalMax.write((uint8_t*)&pedalMaxInit, sizeof(uint16_t));

  // BLE connection event handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Add services to BLE advertising
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addService(configService);

  // Fast advertising for 2 min after boot, then slow (10s interval) to save power
  Bluefruit.Advertising.setInterval(32, 16000);  // fast=20ms, slow=10s (units of 0.625ms)
  Bluefruit.Advertising.setFastTimeout(120);      // seconds before switching to slow interval
  Bluefruit.Advertising.start();
  advertisingStartTime = millis();

  // Initialize moving average buffer
  memset(samples, 0, sizeof(samples));
  lastActive = millis();
}

void loop() {
  unsigned long now = millis();

  // Sample and compute moving average
  int raw = analogRead(PEDAL_PIN);
  samples[sampleIndex++] = raw;
  if (sampleIndex >= SAMPLES) sampleIndex = 0;

  int avg = 0;
  for (int i = 0; i < SAMPLES; i++) avg += samples[i];
  avg /= SAMPLES;

  // Track activity
  int deviation = abs(avg - pedalNeutral);
  if (deviation > deadband) lastActive = now;

  // Determine if a BLE update should be sent
  bool shouldUpdate = (!inLowPower && now - lastUpdate >= UPDATE_INTERVAL_MS) ||
                      (inLowPower && now - lastUpdate >= LOWPOWER_INTERVAL_MS);

  if (shouldUpdate && connected) {
    int16_t command = 0;

    // Map analog input to motor command range with calibration
    int16_t linearOutput = 0;
    if (avg < pedalNeutral - deadband) {
      linearOutput = map(avg, pedalMin, pedalNeutral - deadband, -512, 0);
    } else if (avg > pedalNeutral + deadband) {
      linearOutput = map(avg, pedalNeutral + deadband, pedalMax, 0, 512);
    } else {
      linearOutput = 0;
    }

    
      // Normalize to [-1.0, 1.0]
    float normalized = linearOutput / 512.0;
    float responseExp = responseCurveX1000 / 1000.0;

    //float curved = normalized * pow(fabs(normalized), 0.5);  // less sharp than x³
    float curved = normalized * pow(fabs(normalized), responseExp);  // less sharp than x³
    if (normalized < 0) curved = -fabs(curved);  // preserve sign

    // 3. Optional: smooth output with exponential filter
    static float filtered = 0;
    float alpha = 0.2;
    // If we're in deadband, instantly zero the filtered output
    if (linearOutput == 0) {
      filtered = 0;
    } else {
      filtered = alpha * curved + (1.0 - alpha) * filtered;
    }


    // 4. Scale final command
    command = (int16_t)(filtered * 512.0 * scale / 1000.0);

    // Send over BLE UART
    bleuart.write((uint8_t*)&command, sizeof(command));

    // Output to serial for debugging
    Serial.print("Raw ADC: ");
    Serial.print(raw);
    Serial.print(" | Avg: ");
    Serial.print(avg);
    Serial.print(" | Command: ");
    Serial.println(command);

    lastUpdate = now;
  }

  // Handle transition to/from low power mode
  if (!inLowPower && (now - lastActive > IDLE_TIMEOUT_MS)) {
    inLowPower = true;
  } else if (inLowPower && (now - lastActive <= IDLE_TIMEOUT_MS)) {
    inLowPower = false;
  }

  // --- Blue BLE LED (LED_CONN) ---
  // Connected: solid on (set in callback)
  // Fast advertising (first 2 min): blink every 1s
  // Slow advertising (after 2 min): brief flash every 10s
  if (!connected) {
    static unsigned long lastConnBlink = 0;
    unsigned long advAge = now - advertisingStartTime;
    if (advAge < FAST_ADV_DURATION_MS) {
      if (now - lastConnBlink >= 1000) {
        digitalWrite(LED_CONN, LED_ON);
        delay(100);
        digitalWrite(LED_CONN, LED_OFF);
        lastConnBlink = now;
      }
    } else {
      if (now - lastConnBlink >= 10000) {
        digitalWrite(LED_CONN, LED_ON);
        delay(50);
        digitalWrite(LED_CONN, LED_OFF);
        lastConnBlink = now;
      }
    }
  }

  // LED blink every 3s when connected
  if (connected) {
    static unsigned long lastBlink = 0;
    if (now - lastBlink > 3000) {
      digitalWrite(LED_STATUS_PIN, LED_ON);
      delay(250);
      digitalWrite(LED_STATUS_PIN, LED_OFF);
      lastBlink = now;
    }
  } else {
    digitalWrite(LED_STATUS_PIN, LED_OFF);
    delay(500);  // CPU sleep via FreeRTOS WFI when not connected — saves power
  }
}

// --- BLE Connect/Disconnect Handlers ---
void connect_callback(uint16_t conn_handle) {
  connected = true;
  digitalWrite(LED_CONN, LED_ON);  // solid on when connected
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  connected = false;
  digitalWrite(LED_CONN, LED_OFF);
  Bluefruit.Advertising.start();
  advertisingStartTime = millis();  // reset fast adv window on each disconnect
}
