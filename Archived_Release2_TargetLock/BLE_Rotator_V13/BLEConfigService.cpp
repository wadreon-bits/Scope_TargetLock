#include "BLEConfigService.h"

BLEConfigService* BLEConfigService::self_ = nullptr;

// =============================================================================
// begin()
// Sets up the NUS peripheral service and starts advertising.
// Must be called after Bluefruit.begin(1, 1) in BLEManager::begin().
// =============================================================================
void BLEConfigService::begin() {
  self_ = this;

  // Configure peripheral device name — shown during phone BLE scan
  Bluefruit.Periph.setConnectCallback(connect_cb);
  Bluefruit.Periph.setDisconnectCallback(disconnect_cb);

  // Initialise the Nordic UART Service in peripheral (server) mode
  uart_.begin();

  // Build and start advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(uart_);             // advertise NUS UUID
  Bluefruit.Advertising.setStopCallback(nullptr);      // advertise indefinitely
  Bluefruit.Advertising.restartOnDisconnect(true);     // auto-restart after phone disconnects

  // Scan response carries the full device name (adv packet is too small)
  Bluefruit.ScanResponse.addName();
  // Override the name for this peripheral role — Central role keeps its own name
  Bluefruit.setName(device_name_);

  // Advertising interval: 500 ms — reduced from 100 ms to minimize SoftDevice
  // interrupt load on the main loop and BNO I2C reads.
  // Phone will still find the device within ~500 ms when scanning.
  // Once phone connects, advertising stops entirely — no ongoing overhead.
  // Units are 0.625 ms per unit: 800 * 0.625 = 500 ms
  Bluefruit.Advertising.setInterval(800, 800);
  Bluefruit.Advertising.setFastTimeout(30);  // 30 s fast adv, then slow
  Bluefruit.Advertising.start(0);            // 0 = advertise indefinitely
}

// =============================================================================
// update()
// Non-blocking pump.  Drains bytes from the NUS RX FIFO into the line
// accumulator.  When a complete '\n'-terminated line is ready, it is
// dispatched to dispatchLine() — the same parser used by processSerialCommand().
// Call once per loop() pass.
// =============================================================================
void BLEConfigService::update() {
  if (!connected_) return;   // fast exit — no phone connected

  // Drain all available bytes this pass
  while (uart_.available()) {
    char c = (char)uart_.read();

    if (c == '\r') continue;   // ignore CR (handles \r\n line endings)

    if (c == '\n') {
      // Complete line received — dispatch it
      line_buf_[line_len_] = '\0';
      String line(line_buf_);
      line_len_ = 0;
      dispatchLine(line);      // reuse full serial command parser
      return;                  // one line per loop() pass (matches serial handler)
    }

    // Accumulate — silently drop overflow chars to keep buffer clean
    if (line_len_ < LINE_BUF_SZ - 1) {
      line_buf_[line_len_++] = c;
    }
  }
}

// =============================================================================
// sendLine()
// Sends a string + newline to the connected phone.
// Returns immediately if no phone is connected — no queuing, no blocking.
// Safe to call unconditionally from printStatusLine() and dispatchLine().
// =============================================================================
void BLEConfigService::sendLine(const char* s) {
  if (!connected_ || !s) return;

  // Write string then newline — phone terminal apps expect \r\n
  uart_.write((const uint8_t*)s, strlen(s));
  uart_.write((const uint8_t*)"\r\n", 2);
}

// =============================================================================
// Static callback thunks — forward to instance methods via self_
// =============================================================================
void BLEConfigService::connect_cb(uint16_t conn_handle) {
  if (self_) self_->onConnect(conn_handle);
}

void BLEConfigService::disconnect_cb(uint16_t conn_handle, uint8_t reason) {
  if (self_) self_->onDisconnect(conn_handle, reason);
}

// =============================================================================
// onConnect / onDisconnect
// =============================================================================
void BLEConfigService::onConnect(uint16_t conn_handle) {
  connected_   = true;
  conn_handle_ = conn_handle;
  // Notify serial monitor that a phone has connected — useful during bench work
  Serial.println(F("[CFG] Phone connected"));
}

void BLEConfigService::onDisconnect(uint16_t conn_handle, uint8_t /*reason*/) {
  connected_   = false;
  conn_handle_ = BLE_CONN_HANDLE_INVALID;
  line_len_    = 0;   // discard any partial line from previous session
  Serial.println(F("[CFG] Phone disconnected"));
}
