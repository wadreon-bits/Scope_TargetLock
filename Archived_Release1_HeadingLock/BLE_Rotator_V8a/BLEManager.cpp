#include "BLEManager.h"

BLEManager* BLEManager::self_ = nullptr;

BLEManager::BLEManager() : clientUart_() {}

void BLEManager::setTargetNamePrefix(const char* pfx)   { target_prefix_ = pfx ? pfx : target_prefix_; }
void BLEManager::setRssiFilter(int8_t minRssi)          { min_rssi_ = minRssi; }
void BLEManager::setActiveScan(bool on)                 { active_scan_ = on; }
void BLEManager::setTxPower(int8_t dbm)                 { tx_power_ = dbm; }
void BLEManager::setDeviceName(const char* name)        { device_name_ = name ? name : device_name_; }

void BLEManager::begin() {
  self_ = this;

  Bluefruit.begin(0, 1);            // central only
  Bluefruit.setTxPower(tx_power_);
  Bluefruit.setName(device_name_);

  clientUart_.begin();
  clientUart_.setRxCallback(uart_rx_cb);

  Bluefruit.Central.setConnectCallback(connect_cb);
  Bluefruit.Central.setDisconnectCallback(disconnect_cb);
  Bluefruit.Central.begin();

  Bluefruit.Scanner.setRxCallback(scan_cb);
  Bluefruit.Scanner.useActiveScan(active_scan_);
  Bluefruit.Scanner.filterRssi(min_rssi_);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.start(0);       // continuous
}

void BLEManager::update() {
  // Drain RX ring into (little-endian) int16 samples.
  // Minimal parsing: every 2 bytes -> one sample.
  while (rb_available() >= 2) {
    uint8_t b0, b1;
    rb_pop(b0);
    rb_pop(b1);
    int16_t v = static_cast<int16_t>(static_cast<uint16_t>(b0) | (static_cast<uint16_t>(b1) << 8));
    if (cmd_cb_) cmd_cb_(v, millis());
  }
}

void BLEManager::scan_cb(ble_gap_evt_adv_report_t* report) {
  if (self_) self_->handleScanReport(report);
  Bluefruit.Scanner.resume(); // keep scanning
}

void BLEManager::handleScanReport(ble_gap_evt_adv_report_t* report) {
  // RSSI check
  last_rssi_ = report->rssi;
  if (report->rssi < min_rssi_) return;

  // Name prefix match
  if (!advNameHasPrefix(report->data.p_data, report->data.len, target_prefix_)) return;

  // Try connect
  Bluefruit.Scanner.stop();
  if (!Bluefruit.Central.connect(report)) {
    // Failed: resume scanning
    Bluefruit.Scanner.start(0);
  }
}

void BLEManager::connect_cb(uint16_t conn_handle) {
  if (self_) self_->onConnect(conn_handle);
}

void BLEManager::onConnect(uint16_t conn_handle) {
  conn_handle_ = conn_handle;
  connected_   = false; // set true after UART discover

  if (clientUart_.discover(conn_handle)) {
    clientUart_.enableTXD(); // enable notifications from peripheral
    connected_ = true;
    if (status_cb_) status_cb_(true);
  } else {
    // No UART service; drop and resume scan
    Bluefruit.disconnect(conn_handle);
  }
}

void BLEManager::disconnect_cb(uint16_t conn_handle, uint8_t reason) {
  if (self_) self_->onDisconnect(conn_handle, reason);
}

void BLEManager::onDisconnect(uint16_t /*conn_handle*/, uint8_t /*reason*/) {
  connected_   = false;
  conn_handle_ = BLE_CONN_HANDLE_INVALID;

  // reset RX ring
  rx_head_ = rx_tail_ = 0;

  if (status_cb_) status_cb_(false);

  // Backoff is handled by Bluefruit; simply restart scan
  Bluefruit.Scanner.start(0);
}

void BLEManager::uart_rx_cb(BLEClientUart& /*uart*/) {
  if (self_) self_->onUartRx();
}

void BLEManager::onUartRx() {
  // Pull all available bytes into the ring buffer quickly.
  while (clientUart_.available()) {
    int c = clientUart_.read();
    if (c < 0) break;
    rb_push(static_cast<uint8_t>(c));
  }
}

// ---- Ring buffer helpers ----
inline bool BLEManager::rb_push(uint8_t b) {
  size_t next = (rx_head_ + 1) % RX_BUF_SZ;
  if (next == rx_tail_) {
    // overflow: drop oldest byte (single-byte drop keeps stream moving)
    rx_tail_ = (rx_tail_ + 1) % RX_BUF_SZ;
  }
  rx_buf_[rx_head_] = b;
  rx_head_ = next;
  return true;
}
inline bool BLEManager::rb_pop(uint8_t& b) {
  if (rx_head_ == rx_tail_) return false;
  b = rx_buf_[rx_tail_];
  rx_tail_ = (rx_tail_ + 1) % RX_BUF_SZ;
  return true;
}
inline size_t BLEManager::rb_available() const {
  if (rx_head_ >= rx_tail_) return rx_head_ - rx_tail_;
  return RX_BUF_SZ - (rx_tail_ - rx_head_);
}

// ---- Advertising name prefix match ----
bool BLEManager::advNameHasPrefix(const uint8_t* data, uint8_t len, const char* prefix) {
  if (!data || !prefix) return false;
  const size_t pfxLen = strlen(prefix);

  uint8_t idx = 0;
  while (idx < len) {
    uint8_t fieldLen = data[idx];
    if (fieldLen == 0) break;
    if (idx + fieldLen >= len) break;

    uint8_t type = data[idx + 1];
    if (type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME || type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME) {
      const uint8_t* name = &data[idx + 2];
      uint8_t nameLen = fieldLen - 1;
      if (nameLen >= pfxLen && memcmp(name, prefix, pfxLen) == 0) {
        return true;
      }
    }
    idx += fieldLen + 1;
  }
  return false;
}