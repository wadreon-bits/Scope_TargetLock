// === BLE_RECEIVER_ROTATOR.ino ===
// BLE Receiver Rotator for Adafruit nRF52840 (Feather or Itsybitsy)
// Connects to BLE controller pedals named "BLE_Controller_Pedal_<#>"
// Uses: DRV_IN1_PIN (D5), DRV_IN2_PIN (D12), DRV_SLP_PIN (D9)
//V2 Added Current Measurement, Overcurrent Adjustment 
// USE FOR NON ENCODED MOTORS!!!!!!
#include <bluefruit.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h> // DAC Library
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "NRF52TimerInterrupt.h"
#include <Adafruit_DotStar.h>

#define APP_VERSION     "2.0" //build into ble read values
//#define DEBUG_PRINTS
//#define ENABLE_DOTSTAR
// === Pin Definitions ===
#define DRV_IN1_PIN     13   //PIN
#define DRV_IN2_PIN     12  //PIN
#define ENC_B_PIN       11
#define ENC_A_PIN       10
//#define DRV_CS_PIN      A0
#define DRV_SLP_PIN     9   //PIN 
#define DRV_FLT_PIN     7   //PIN
#define LED_STATUS_PIN  3   //PIN Onboard LED for BLE status
#define LED_PWRBTN_PIN  2   //PIN
#define IPROPI_PIN      A0

#define DOTSTAR_DATA_PIN  8
#define DOTSTAR_CLK_PIN   6

// === CONSTANTS ===
#define MIN_PWM         50                // Minimum effective PWM (adjust as needed)
#define HW_TIMER_INTERVAL_US      100000  // timer interval in microseconds


// === Function Declarations ===
void scan_callback(ble_gap_evt_adv_report_t* report);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
BLEClientUart clientUart;

//BLEClientService uart_service(0x6E400001B5A3F393E0A9E50E24DCCA9E); // Nordic UART
// --- System State ---
bool targetLockEnabled = false;       // Target lock mode status
bool connected = false;               // True if any pedal connected
bool timeout_once = false;            // bit lock to avoid re-entry into timeout test
bool flag_100ms = false;
int currentCommand = 0;               // Last received motor command
uint32_t lastCommandTime = 0;         // Timestamp of last valid motor command
int motor_state = 0;                  // current motor state- 0 = off. Value equals commanded speed and dir

volatile unsigned int pwrled_cnt = 0; 

const float ADC_VREF = 3.3;           // VDD is used as ADC reference
const int ADC_MAX = 4095;             // 12-bit ADC resolution
const float MV_PER_AMP = 1100.0;       // Pololu DRV8874 board: 500 mV per 1 A

float motor_current;
#define CURRENT_AVG_WINDOW 10
float current_samples[CURRENT_AVG_WINDOW];
uint8_t current_index = 0;

//uint16_t overcurrent_dac_value = 600; // 750mA default (750/1000 = 0.75A)
uint16_t overcurrent_trip_mA = 1300;  // Default trip = 750 mA
// Constants for IPROPI trip calculation
const float IPROPI_GAIN = 0.000455;     // 455 µA per A
const float RIPROPI_OHMS = 2350.0;      // Your actual resistor
const float DAC_VDD = 3.3;      

// Init NRF52 timer NRF_TIMER1
NRF52Timer ITimer(NRF_TIMER_3);
// Init DAC
Adafruit_MCP4725 dac;
// Init DotStar
//Adafruit_DotStar dotstar(1, DOTSTAR_DATA_PIN, DOTSTAR_CLK_PIN, DOTSTAR_BRG);
Adafruit_DotStar dotstar(1, DOTSTAR_DATA_PIN, DOTSTAR_CLK_PIN, DOTSTAR_BGR);

void TimerHandler()
{
  flag_100ms = true;
}

// === Setup ===
void setup() {
  //Bluefruit.configClearStorage();
  Serial.begin(115200);
  delay(100);
 // while (!Serial); delay(500);
  Serial.print("Version: ");
  Serial.println(APP_VERSION);
  delay(500);
  Serial.println("BLE Receiver Rotator Starting");

  // Motor control pin setup
  pinMode(DRV_IN1_PIN, OUTPUT);
  pinMode(DRV_IN2_PIN, OUTPUT);
  pinMode(DRV_SLP_PIN, OUTPUT);
  
  pinMode(DRV_FLT_PIN, INPUT);
  pinMode(IPROPI_PIN, INPUT);
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(LED_PWRBTN_PIN, OUTPUT);

  digitalWrite(DRV_IN1_PIN, LOW);
  digitalWrite(DRV_IN2_PIN, LOW);
  digitalWrite(DRV_SLP_PIN, LOW);
  digitalWrite(LED_STATUS_PIN, LOW);
  digitalWrite(LED_PWRBTN_PIN, LOW);

  analogReadResolution(12);     // 12-bit ADC (0–4095)

  // BLE init
  Bluefruit.begin(0,1);
  //Bluefruit.Periph.begin();  // Required even if not used
  Bluefruit.setTxPower(4);   // +4 dBm
  Bluefruit.setName("BLE_Receiver_Rotator");

  analogWriteResolution(10);  // Set resolution to 10 bits (0–1023)

  clientUart.begin();

  clientUart.setRxCallback(bleuart_rx_callback);

  delay(2000); // Let the pedal boot and start advertising

  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.begin();

  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-80);
  Bluefruit.Scanner.start(0); // Scan continuously

  Serial.println("Scanning for BLE_Controller_Pedal_<#>...");

  Wire.begin();
  dac.begin(0x60); // Default I2C address
  delay(5);
  updateOvercurrentTrip();
  dotstar.begin();      // Initialize DotStar
  dotstar.show();       // Turn off initially
  // Interval in microsecs
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = "));
    Serial.println(millis());
  }
}

// === Main Loop ===
void loop() {
  
  // Blink LED every 3s when connected to at least one pedal
  // static uint32_t lastBlink = 0;
  // if (connected && (millis() - lastBlink > 1000)) {
  //   digitalWrite(LED_STATUS_PIN, HIGH);
  //   delay(250);
  //   digitalWrite(LED_STATUS_PIN, LOW);
  //   lastBlink = millis();
  // }

  if(flag_100ms){
    flag_100ms = false;


    if(connected){   //pulse when powered and connected
      if(pwrled_cnt < 29) //2.7sec
        digitalWrite(LED_PWRBTN_PIN, LOW);
      else
        digitalWrite(LED_PWRBTN_PIN, HIGH);
      if(pwrled_cnt++ >=30)//300msec
        pwrled_cnt = 0;
    }
    else {  //blink when not connected but powered
      if(pwrled_cnt < 3) //2.7sec
        digitalWrite(LED_PWRBTN_PIN, LOW);
      else
        digitalWrite(LED_PWRBTN_PIN, HIGH);
      if(pwrled_cnt++ >=6)//300msec
        pwrled_cnt = 0;

    }
    

    //process motor current
    if(motor_state != 0){
      int adcVal = analogRead(IPROPI_PIN);
      float voltage = (adcVal * ADC_VREF) / ADC_MAX;
      //float instant_current = voltage / 1.1;
      float instant_current = voltage / (RIPROPI_OHMS * IPROPI_GAIN);
      current_samples[current_index] = instant_current;
      current_index = (current_index + 1) % CURRENT_AVG_WINDOW;

      float sum = 0;
      for (int i = 0; i < CURRENT_AVG_WINDOW; i++) sum += current_samples[i];
      motor_current = sum / CURRENT_AVG_WINDOW;
      #ifdef DEBUG_PRINTS
      // Print current values
      Serial.print("ADC: ");
      Serial.print(adcVal);
      Serial.print(current, 3);
      Serial.println(" A");
      #endif
    }
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'A') {
        overcurrent_trip_mA += 25;
        if (overcurrent_trip_mA > 3300) overcurrent_trip_mA = 3300;
      } else if (c == 'a') {
        overcurrent_trip_mA -= 25;
        if (overcurrent_trip_mA < 0) overcurrent_trip_mA = 0;
      }
      dac.setVoltage((uint16_t)(overcurrent_trip_mA * 4095 / 3300), false);
      Serial.print("Overcurrent setpoint updated: ");
      Serial.print(overcurrent_trip_mA);
      Serial.println(" mA");
    }

  }
//The intermittent Zero bug solution had to do with a unsigned overflow issue. Last command time, on occasion, could be larger than Millis, due to an 
// ble callback happening at immediatly after the millis() call. this would overflow to very large number. 
// IN CASE NOT FIXED. 
  if (((int32_t)(millis() - lastCommandTime) > 2000) && !timeout_once) {
    timeout_once = true;
    Serial.print("Timeout triggered");
    drive_motor(0);
  }
} //!!!!!!!!!!!!!!!!!!!!!!!END OF MAIN LOOP!!!!!!!!!!!!!!!!!!!!!!!!!!

// === Scan Callback (Manual parsing compatible with Bluefruit v0.21.0) ===
void scan_callback(ble_gap_evt_adv_report_t* report) {
  const uint8_t* advData = report->data.p_data;
  uint8_t index = 0;
  bool nameFound = false;

  while (index < report->data.len) {
    uint8_t len = advData[index];
    if (len == 0) break;

    uint8_t type = advData[index + 1];

    // Look for complete or short local name
    if (type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME || type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME) {
      char devName[32] = {0};
      uint8_t nameLen = min(len - 1, (uint8_t)(sizeof(devName) - 1));
      memcpy(devName, &advData[index + 2], nameLen);

      Serial.print("Found device name: ");
      Serial.println(devName);
      nameFound = true;

      // Look for target prefix
      if (strncmp(devName, "Scope_Pedal_1", 13) == 0) {
        Serial.println("Matching pedal found, attempting connection...");

      // Save a copy of the peer address
      ble_gap_addr_t peer_addr = report->peer_addr;


      Serial.println("Connecting...");

      Serial.print("Attempting to connect to address: ");
      for (int i = 5; i >= 0; i--) {
        Serial.print(report->peer_addr.addr[i], HEX);
        if (i > 0) Serial.print(":");
      }
      Serial.println();

      bool success = Bluefruit.Central.connect(report);
       
      if (success) {
        Serial.println("Connected to pedal!");
        digitalWrite(LED_STATUS_PIN, HIGH);
        Bluefruit.Scanner.stop();
      } 
      else {
        Serial.println("Connection failed. Resuming scan...");
        delay(500);
        //Bluefruit.Central.disconnectAll();
        Bluefruit.Central.clearBonds();
        delay(100);
        //Bluefruit.Scanner.start(0);
        //Bluefruit.Scanner.resume(); 
        Bluefruit.Scanner.stop();  // Ensure not already stuck
        delay(100);
        Bluefruit.Scanner.start(0);
      }  
      return; // Exit after one match
      }
    }

    index += len + 1;
  }
  Bluefruit.Scanner.resume(); 
}

// === Disconnect Callback ===
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.print("Disconnected, reason: 0x");
  Serial.println(reason, HEX);
  digitalWrite(LED_STATUS_PIN, LOW);
  drive_motor(0); //stop motor if pedal lost. 
  Serial.println("Motor Stopped!");
  connected = false;
  delay(100);
  Serial.println("Restarting scan...");
  Bluefruit.Scanner.start(0);
}

void connect_callback(uint16_t conn_handle){
  Serial.println("Connected"); 
  Serial.print("Dicovering Device Information ... ");
  if(clientUart.discover(conn_handle)){
    Serial.print("found UART");
    clientUart.enableTXD();
    
  }
  else{
    Serial.print("found NONE");
    Bluefruit.disconnect(conn_handle);
    digitalWrite(LED_STATUS_PIN, HIGH);
  }
  connected = true;

}

void bleuart_rx_callback(BLEClientUart &uart_service) {
  while (uart_service.available() >= sizeof(int16_t)) {
    int16_t command = 0;
    size_t bytesRead = uart_service.read((uint8_t*)&command, sizeof(command));

    if (bytesRead == sizeof(command)) {
#ifdef DEBUG_PRINTS 
      Serial.print("[RX] Command: ");
      Serial.println(command);
#endif
      parse_command(command); // <- This updates lastCommandTime
    } else {
      Serial.println("⚠️ Partial packet received, ignoring");
    }
  }
}

void parse_command(int16_t cmd){
  
 // if (cmd < -525 || cmd > 525) return;  // skip invalid

  lastCommandTime = millis();   //valid command. save time off 
  timeout_once = false;
  drive_motor(cmd);

}

void drive_motor(int16_t command) {
  bool direction;

  motor_state = command; 
  updateDotStar(command);
  if (command == 0) {
    // Disable motor when command is neutral
    
    digitalWrite(DRV_SLP_PIN, LOW); // Inverted logic: HIGH = DRV_SLP_PIN
    analogWrite(DRV_IN1_PIN, 0); // No PWM
    return;
  }

  // Wake up motor driver
  digitalWrite(DRV_SLP_PIN, HIGH); // LOW = awake

  // Determine direction
  if (command > 0) {
    digitalWrite(DRV_IN2_PIN, HIGH); // One direction
    direction = true;
  } else {
    direction = false;
    digitalWrite(DRV_IN2_PIN, LOW);  // Opposite direction
    command = -command;            // Make magnitude positive
  }


  //10bit analog value clamped and written
  uint16_t pwm = min(command, 512) * 1023 / 512;
  if (pwm < MIN_PWM) pwm = MIN_PWM;
  
  analogWrite(DRV_IN1_PIN, pwm);
  
  // Serial.print("Drive PWM: ");
  // Serial.print(pwm);
  Serial.print("Cmnd:");
  Serial.print(command);
  Serial.print(" | Direction: ");
  //Serial.println(digitalRead(DRV_IN2_PIN) ? "FWD" : "REV");
  Serial.print(digitalRead(DRV_IN2_PIN) ? "CW" : "CCW"); //final arrow rotation direction
  Serial.print(" | A:");
  Serial.println(motor_current);
}

void updateOvercurrentTrip() {
  float trip_current_A = overcurrent_trip_mA / 1000.0;  // convert to amps
  float vref = trip_current_A * RIPROPI_OHMS * IPROPI_GAIN;  // V = I × R × G
  uint16_t dac_value = (uint16_t)((vref / DAC_VDD) * 4095.0);

  dac.setVoltage(dac_value, false);

#ifdef DEBUG_PRINTS
  Serial.print("Trip current: ");
  Serial.print(overcurrent_trip_mA);
  Serial.print(" mA | VREF: ");
  Serial.print(vref, 3);
  Serial.print(" V | DAC = ");
  Serial.println(dac_value);
#endif
}

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
