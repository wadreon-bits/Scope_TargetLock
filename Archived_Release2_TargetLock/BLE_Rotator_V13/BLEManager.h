#pragma once
#include <Arduino.h>
#include <bluefruit.h>

// Transport-only BLE manager for Bluefruit Central + UART client.
// Emits raw little-endian int16 "drive" samples via callback.
// No control policy, no Serial prints inside callbacks.

class BLEManager {
public:
  using CommandCb = void (*)(int16_t cmd, uint32_t t_ms);
  using StatusCb  = void (*)(bool connected);

  BLEManager();

  // ---- Configuration (call before begin()) ----
  void setTargetNamePrefix(const char* pfx); // default "Scope_Pedal_1"
  void setRssiFilter(int8_t minRssi);        // default -80 dBm
  void setActiveScan(bool on);               // default true
  void setTxPower(int8_t dbm);               // default +4 dBm
  void setDeviceName(const char* name);      // default "BLE_Receiver_Rotator"

  // ---- Lifecycle ----
  void begin();
  void update();                 // non-blocking pump
  bool isConnected() const { return connected_; }
  int8_t getRssi() const { return last_rssi_; }

  // ---- Callbacks ----
  void setCommandCallback(CommandCb cb) { cmd_cb_ = cb; }
  void setStatusCallback(StatusCb cb)  { status_cb_ = cb; }

private:
  // --- Bluefruit objects ---
  BLEClientUart clientUart_;

  // --- Config ---
  const char* target_prefix_ = "Scope_Pedal_1";
  const char* device_name_   = "BLE_Receiver_Rotator";
  int8_t  min_rssi_   = -80;
  int8_t  tx_power_   = 4;
  bool    active_scan_ = true;

  // --- State ---
  bool    connected_  = false;
  int8_t  last_rssi_  = INT8_MIN;
  uint16_t conn_handle_ = BLE_CONN_HANDLE_INVALID;

  // --- RX ring buffer (small, lock-free in callback) ---
  static constexpr size_t RX_BUF_SZ = 256;
  uint8_t rx_buf_[RX_BUF_SZ];
  volatile size_t rx_head_ = 0;
  volatile size_t rx_tail_ = 0;

  // --- Callbacks ---
  CommandCb cmd_cb_   = nullptr;
  StatusCb  status_cb_ = nullptr;

  // --- Internals ---
  static BLEManager* self_;  // singleton thunk target

  static void scan_cb(ble_gap_evt_adv_report_t* report);
  static void connect_cb(uint16_t conn_handle);
  static void disconnect_cb(uint16_t conn_handle, uint8_t reason);
  static void uart_rx_cb(BLEClientUart& uart);

  void handleScanReport(ble_gap_evt_adv_report_t* report);
  void onConnect(uint16_t conn_handle);
  void onDisconnect(uint16_t conn_handle, uint8_t reason);
  void onUartRx();

  // ring helpers
  inline bool rb_push(uint8_t b);
  inline bool rb_pop(uint8_t& b);
  inline size_t rb_available() const;

  // adv name parse
  static bool advNameHasPrefix(const uint8_t* data, uint8_t len, const char* prefix);
};