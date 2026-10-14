#pragma once
#include <Arduino.h>
#include <bluefruit.h>

// =============================================================================
// BLEConfigService  —  Nordic UART Service (NUS) peripheral role
// =============================================================================
// Runs alongside BLEManager's central role on the nRF52840.
// Requires Bluefruit.begin(1, 1) instead of begin(0, 1) — one number changed
// in BLEManager::begin().  Everything else in BLEManager is untouched.
//
// Purpose:
//   - Accepts text command lines from a paired phone (nRF Toolbox, Serial
//     Bluetooth Terminal, LightBlue, etc.) and routes them to the existing
//     dispatchLine() handler — all commands including SET/GET work over BLE
//     exactly as they do over serial.
//   - Forwards any text sent via sendLine() to the phone when connected.
//     Used to mirror the 4 Hz ST stream and command responses to the phone.
//     sendLine() is a no-op when no phone is connected — zero overhead.
//
// Typical phone apps (free, no account needed):
//   iOS / Android:  "Serial Bluetooth Terminal" by Kai Morich
//   iOS:            "nRF Toolbox" by Nordic Semiconductor  (UART tab)
//   iOS / Android:  "LightBlue" by Punch Through
//
// Usage in .ino:
//   BLEConfigService bleConfig;
//
//   setup():
//     bleConfig.begin();          // after ble.begin()
//
//   loop():
//     bleConfig.update();         // non-blocking, alongside ble.update()
//
//   printStatusLine():
//     Serial.println(line);
//     bleConfig.sendLine(line);   // no-op if not connected
// =============================================================================

// dispatchLine() is defined in the .ino and handles all command parsing.
// Declared extern here so BLEConfigService can call it directly, reusing
// the full command table without duplicating any logic.
extern void dispatchLine(String& line);

class BLEConfigService {
public:

  BLEConfigService() {}

  // ---- Configuration (call before begin()) --------------------------------

  // Advertised name the phone sees during a BLE scan.
  // Default is deliberately distinct from pedal names.
  void setDeviceName(const char* name) { device_name_ = name ? name : device_name_; }

  // ---- Lifecycle ----------------------------------------------------------

  // Initialise NUS peripheral and start advertising.
  // Must be called after Bluefruit.begin(1, 1) — i.e. after ble.begin().
  void begin();

  // Non-blocking pump: drains RX ring into complete lines, dispatches each
  // to dispatchLine().  Call once per loop() pass.
  void update();

  // ---- Output to phone ----------------------------------------------------

  // Send a null-terminated string + newline to the phone.
  // Returns immediately (no-op) if no phone is connected.
  // Safe to call unconditionally — the isConnected() guard is inside.
  void sendLine(const char* s);

  // Convenience overload for Arduino String
  void sendLine(const String& s) { sendLine(s.c_str()); }

  // ---- Status -------------------------------------------------------------
  bool isConnected() const { return connected_; }

private:
  // --- Bluefruit NUS peripheral objects ---
  BLEUart uart_;                          // Nordic UART Service (peripheral side)

  // --- Config ---
  const char* device_name_ = "Scope_Cfg"; // short name fits adv packet cleanly

  // --- Connection state ---
  bool     connected_   = false;
  uint16_t conn_handle_ = BLE_CONN_HANDLE_INVALID;

  // --- RX line accumulator -------------------------------------------------
  // Collects incoming bytes into complete '\n'-terminated lines before
  // dispatching.  Mirrors the non-blocking pattern in processSerialCommand().
  static constexpr uint8_t LINE_BUF_SZ = 64;
  char    line_buf_[LINE_BUF_SZ];
  uint8_t line_len_ = 0;

  // --- Static callback thunks ----------------------------------------------
  // Bluefruit callbacks must be static; they forward to instance methods
  // via the singleton pointer below.
  static BLEConfigService* self_;

  static void connect_cb(uint16_t conn_handle);
  static void disconnect_cb(uint16_t conn_handle, uint8_t reason);

  // --- Instance handlers ---
  void onConnect(uint16_t conn_handle);
  void onDisconnect(uint16_t conn_handle, uint8_t reason);
};
