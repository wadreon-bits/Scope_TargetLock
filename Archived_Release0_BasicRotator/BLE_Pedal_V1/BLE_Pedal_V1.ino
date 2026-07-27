// === BLE_CONTROLLER_PEDAL.ino ===
// Controller Side - Feather nRF52840
// Sends direction/speed commands over BLE to receiver based on pedal input
// Features: moving average filter, BLE UART, BLE config (deadband, scale, calibration), low power mode, nonlinear scaling response, serial debugging output
//0.3 for a super-flat center   0.6 for a snappier feel   1.0 for linear
//1000 = 1.0 (linear) 500 = 0.5 (smoother curve)  1500 = 1.5 (more aggressive curve)


#include <bluefruit.h>
#include <math.h>  // Required for nonlinear response curve


#define APP_VERSION         "1.0"     //build into ble read values
// --- Pin and Timing Definitions ---
#define PEDAL_PIN           A0      // Potentiometer analog input
#define LED_STATUS_PIN      3      // LED pin used to indicate BLE connection status
#define SAMPLES             10      // Number of analog samples for moving average
#define UPDATE_INTERVAL_MS  50      // BLE update interval when active
#define LOWPOWER_INTERVAL_MS 1000   // BLE update interval in low power mode
#define IDLE_TIMEOUT_MS     600000  // Time (10 min) before entering low power mode

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
unsigned long lastUpdate = 0;   // Timestamp of last BLE update
unsigned long lastActive = 0;   // Timestamp of last non-neutral activity
bool inLowPower = false;        // True if controller is in low power mode
bool connected = false;         // BLE connection status

// --- Configurable Parameters (internal variables) ---
int deadband = 7;               // ADC deviation range for neutral zone
int scale = 1000;               // Sensitivity scale in thousandths (e.g., 1000 = 1.0)
uint16_t responseCurveX1000 = 500;  // default = 0.5
int pedalMin = 387;             // ADC value at full reverse pedal
int pedalNeutral = 434;         // ADC value when pedal is centered
int pedalMax = 481;             // ADC value at full forward pedal

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.print("Version: ");
  Serial.println(APP_VERSION);
  //delay(100);
  pinMode(PEDAL_PIN, INPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);

  // --- Initialize BLE ---
  Bluefruit.begin();
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
  Bluefruit.Advertising.start();

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

  // LED blink every 3s when connected
  if (connected) {
    static unsigned long lastBlink = 0;
    if (now - lastBlink > 3000) {
      digitalWrite(LED_STATUS_PIN, HIGH);
      delay(250);
      digitalWrite(LED_STATUS_PIN, LOW);
      lastBlink = now;
    }
  } else {
    digitalWrite(LED_STATUS_PIN, LOW);
  }
}

// --- BLE Connect/Disconnect Handlers ---
void connect_callback(uint16_t conn_handle) {
  connected = true;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  connected = false;
  Bluefruit.Advertising.start();
}
