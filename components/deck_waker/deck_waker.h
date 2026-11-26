#pragma once

#include "esphome/core/component.h"
#include "esphome/components/esp32_ble/ble.h"

#include <atomic>
#include <string>

#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_event.h>
#include <esp_hidd.h>

namespace esphome::deck_waker {

enum class AdvState : uint8_t {
  IDLE,        // not advertising
  UNDIRECTED,  // connectable advertising, only while pairing
  DIRECTED,    // high-duty directed "knock" at the bonded host
  STOPPING,    // waiting for ADV_STOP_COMPLETE
};

/// What to send once the host is connected (and encrypted)
enum class HostAction : uint8_t {
  NONE,
  KEY,          // keyboard key (HID usage in pending key)
  SYSTEM_SLEEP, // Generic Desktop "System Sleep" -> KEY_SLEEP
  SYSTEM_POWER, // Generic Desktop "System Power Down" -> KEY_POWER
};

class DeckWaker : public Component {
 public:
  void set_parent(esp32_ble::ESP32BLE *parent) { this->parent_ = parent; }
  void set_name(const char *name) { this->name_ = name; }
  /// Number of high-duty directed advertising bursts (~1.28 s each) per wake/connect request (1..20).
  void set_direct_attempts(uint8_t attempts) { this->direct_attempts_cfg_ = attempts < 1 ? 1 : (attempts > 20 ? 20 : attempts); }
  uint8_t get_direct_attempts() const { return this->direct_attempts_cfg_; }
  void set_pause_scanner(bool pause) { this->pause_scanner_ = pause; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; }

  // Called from the ESPHome main loop by esp32_ble (deferred GAP events)
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

  /// Wake the host: high-duty directed advertising until it connects. Sends nothing.
  void wake();
  /// Send one keyboard key (HID usage id). Connects first (knock) if needed.
  void send_key(uint8_t key);
  /// Send the HID "System Sleep" key (Linux KEY_SLEEP). Connects first if needed.
  void sleep_host();
  /// Send the HID "System Power Down" key (Linux KEY_POWER). Connects first if needed.
  void power_key();
  /// Remove all BLE bonds (use before re-pairing the Steam Deck).
  void clear_bonds();
  /// Advertise (connectable, undirected) for `seconds` so a host can pair.
  void start_pairing_mode(uint32_t seconds = 120);
  bool in_pairing_mode() const;

  bool is_connected() const { return this->connected_.load(); }
  bool has_bond();
  std::string bonded_peer_str();

  // internal, used by the HID event callback / report task
  void on_hid_event_(int32_t id);
  esp_hidd_dev_t *hid_dev_{nullptr};
  uint8_t report_id_{0};
  uint8_t report_len_{0};
  uint8_t report_buf_[8]{};
  std::atomic<bool> report_busy_{false};

 protected:
  void init_hid_();
  void configure_adv_data_();
  void start_undirected_();
  bool start_directed_();
  void stop_adv_();
  bool get_peer_(esp_bd_addr_t addr, esp_ble_addr_type_t *type);
  void request_(HostAction action, uint8_t key);
  void begin_knock_();
  void end_knock_();
  void perform_(HostAction action, uint8_t key);
  void send_report_(uint8_t report_id, const uint8_t *data, uint8_t len);
  void scanner_pause_(bool pause);

  esp32_ble::ESP32BLE *parent_{nullptr};
  const char *name_{"Steam Deck Waker"};
  uint8_t direct_attempts_cfg_{8};
  bool pause_scanner_{true};

  bool initialized_{false};
  bool init_failed_{false};
  bool adv_data_set_{false};
  bool adv_data_pending_{false};
  AdvState adv_state_{AdvState::IDLE};
  uint32_t adv_state_since_{0};
  uint32_t adv_retry_after_{0};

  std::atomic<bool> connected_{false};
  bool was_connected_{false};

  // knock (directed advertising) sequence
  bool knock_active_{false};
  bool direct_requested_{false};
  uint8_t direct_attempts_left_{0};
  uint32_t knock_deadline_{0};
  bool coex_bt_{false};
  bool scanner_paused_{false};

  // action to run once connected
  HostAction pending_action_{HostAction::NONE};
  uint8_t pending_key_{0};
  uint32_t action_at_{0};
  bool action_scheduled_{false};

  uint32_t pairing_until_{0};
  bool bond_cached_{false};
  uint32_t bond_checked_at_{0};
  bool bond_check_valid_{false};
};

}  // namespace esphome::deck_waker
