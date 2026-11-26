#include "deck_waker.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef DECK_WAKER_PAUSE_SCAN
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#endif

#include <cstring>
#include <vector>

#include <esp_bt_defs.h>
#include <esp_coexist.h>
#include <esp_hidd_gatts.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome::deck_waker {

static const char *const TAG = "deck_waker";

static DeckWaker *g_instance = nullptr;  // NOLINT

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Give Bluetooth the shared 2.4 GHz radio while knocking (Wi-Fi and BT share the antenna on the ESP32).
static void set_coex_prefer_bt(bool bt) { esp_coex_preference_set(bt ? ESP_COEX_PREFER_BT : ESP_COEX_PREFER_BALANCE); }
#pragma GCC diagnostic pop

static constexpr uint8_t REPORT_ID_KEYBOARD = 1;
static constexpr uint8_t REPORT_ID_SYSTEM = 2;
static constexpr uint8_t SYSTEM_POWER_DOWN = 0x01;  // bit 0: usage 0x81
static constexpr uint8_t SYSTEM_SLEEP = 0x02;       // bit 1: usage 0x82

static const uint8_t REPORT_MAP[] = {
    // ---- Keyboard, report id 1: modifiers, reserved, 6 keys (+ LED output) ----
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x06,  // Usage (Keyboard)
    0xA1, 0x01,  // Collection (Application)
    0x85, 0x01,  //   Report ID (1)
    0x05, 0x07,  //   Usage Page (Key Codes)
    0x19, 0xE0,  //   Usage Minimum (224)
    0x29, 0xE7,  //   Usage Maximum (231)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x01,  //   Logical Maximum (1)
    0x75, 0x01,  //   Report Size (1)
    0x95, 0x08,  //   Report Count (8)
    0x81, 0x02,  //   Input (Data, Variable, Absolute) ; modifiers
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x08,  //   Report Size (8)
    0x81, 0x01,  //   Input (Constant) ; reserved
    0x95, 0x05,  //   Report Count (5)
    0x75, 0x01,  //   Report Size (1)
    0x05, 0x08,  //   Usage Page (LEDs)
    0x19, 0x01,  //   Usage Minimum (1)
    0x29, 0x05,  //   Usage Maximum (5)
    0x91, 0x02,  //   Output (Data, Variable, Absolute) ; LEDs
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x03,  //   Report Size (3)
    0x91, 0x01,  //   Output (Constant) ; padding
    0x95, 0x06,  //   Report Count (6)
    0x75, 0x08,  //   Report Size (8)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x65,  //   Logical Maximum (101)
    0x05, 0x07,  //   Usage Page (Key Codes)
    0x19, 0x00,  //   Usage Minimum (0)
    0x29, 0x65,  //   Usage Maximum (101)
    0x81, 0x00,  //   Input (Data, Array) ; key array
    0xC0,        // End Collection
    // ---- System Control, report id 2: Power Down, Sleep, Wake Up ----
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x80,  // Usage (System Control)
    0xA1, 0x01,  // Collection (Application)
    0x85, 0x02,  //   Report ID (2)
    0x19, 0x81,  //   Usage Minimum (System Power Down)
    0x29, 0x83,  //   Usage Maximum (System Wake Up)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x01,  //   Logical Maximum (1)
    0x75, 0x01,  //   Report Size (1)
    0x95, 0x03,  //   Report Count (3)
    0x81, 0x06,  //   Input (Data, Variable, Relative) ; one-shot controls
    0x95, 0x05,  //   Report Count (5)
    0x81, 0x01,  //   Input (Constant) ; padding
    0xC0,        // End Collection
};

static esp_hid_raw_report_map_t s_report_maps[] = {
    {.data = REPORT_MAP, .len = sizeof(REPORT_MAP)},
};

static constexpr uint32_t DIRECTED_WINDOW_MS = 1500;  // high duty directed adv lasts max 1.28 s
static constexpr uint32_t KNOCK_MARGIN_MS = 3000;
static constexpr uint32_t ACTION_DELAY_AFTER_CONNECT_MS = 1500;  // let encryption + CCCD settle

// Runs in the "ble_hidd_events" event-loop task
static void hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
  if (g_instance != nullptr)
    g_instance->on_hid_event_(id);
}

// esp_hidd_dev_input_set() blocks until the GATT confirm arrives; never run it in the main loop.
static void report_task(void *arg) {
  auto *self = static_cast<DeckWaker *>(arg);
  esp_err_t err = esp_hidd_dev_input_set(self->hid_dev_, 0, self->report_id_, self->report_buf_, self->report_len_);
  if (err == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(60));
    uint8_t release[8] = {0};
    esp_hidd_dev_input_set(self->hid_dev_, 0, self->report_id_, release, self->report_len_);
  }
  self->report_busy_.store(false);
  vTaskDelete(nullptr);
}

void DeckWaker::setup() { g_instance = this; }

void DeckWaker::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Deck Waker (Bluedroid BLE HID keyboard):\n"
                "  Name: %s\n"
                "  Directed attempts: %u\n"
                "  Pause scanner while knocking: %s",
                this->name_, this->direct_attempts_cfg_, YESNO(this->pause_scanner_));
  ESP_LOGCONFIG(TAG, "  Paired host: %s", this->bonded_peer_str().c_str());
}

void DeckWaker::init_hid_() {
  // We own the GATT server callback: esp32_ble only registers one when esp32_ble_server is used.
  esp_err_t err = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gatts_register_callback failed: %s", esp_err_to_name(err));
    this->init_failed_ = true;
    return;
  }

  // Distribute encryption + identity keys so the host can resolve/reconnect us.
  uint8_t key_mask = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &key_mask, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &key_mask, sizeof(uint8_t));

  static esp_hid_device_config_t cfg = {};
  cfg.vendor_id = 0x16C0;
  cfg.product_id = 0x05DF;
  cfg.version = 0x0101;
  cfg.device_name = this->name_;
  cfg.manufacturer_name = "ESPHome";
  cfg.serial_number = "deckwaker01";
  cfg.report_maps = s_report_maps;
  cfg.report_maps_len = 1;

  err = esp_hidd_dev_init(&cfg, ESP_HID_TRANSPORT_BLE, hidd_event_callback, &this->hid_dev_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_hidd_dev_init failed: %s", esp_err_to_name(err));
    this->init_failed_ = true;
    return;
  }
  this->initialized_ = true;
  ESP_LOGI(TAG, "BLE HID keyboard started; paired host: %s", this->bonded_peer_str().c_str());
}

void DeckWaker::on_hid_event_(int32_t id) {
  switch (id) {
    case ESP_HIDD_CONNECT_EVENT:
      this->connected_.store(true);
      break;
    case ESP_HIDD_DISCONNECT_EVENT:
      this->connected_.store(false);
      break;
    default:
      break;
  }
}

void DeckWaker::configure_adv_data_() {
  std::vector<uint8_t> adv;
  adv.insert(adv.end(), {0x02, 0x01, 0x06});        // Flags
  adv.insert(adv.end(), {0x03, 0x19, 0xC1, 0x03});  // Appearance: HID keyboard
  adv.insert(adv.end(), {0x03, 0x03, 0x12, 0x18});  // 16-bit UUIDs: HID service
  size_t name_len = strlen(this->name_);
  size_t room = 31 - adv.size() - 2;
  if (name_len > room)
    name_len = room;
  adv.push_back(static_cast<uint8_t>(name_len + 1));
  adv.push_back(name_len == strlen(this->name_) ? 0x09 : 0x08);
  adv.insert(adv.end(), this->name_, this->name_ + name_len);

  esp_err_t err = esp_ble_gap_config_adv_data_raw(adv.data(), adv.size());
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "config_adv_data_raw failed: %s", esp_err_to_name(err));
    this->adv_retry_after_ = millis() + 5000;
    return;
  }
  this->adv_data_pending_ = true;
}

void DeckWaker::start_undirected_() {
  esp_ble_adv_params_t p = {};
  p.adv_int_min = 0x00A0;  // 100 ms
  p.adv_int_max = 0x0140;  // 200 ms
  p.adv_type = ADV_TYPE_IND;
  p.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
  p.channel_map = ADV_CHNL_ALL;
  p.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
  esp_err_t err = esp_ble_gap_start_advertising(&p);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "start undirected advertising failed: %s", esp_err_to_name(err));
    this->adv_retry_after_ = millis() + 2000;
    return;
  }
  this->adv_state_ = AdvState::UNDIRECTED;
  this->adv_state_since_ = millis();
}

bool DeckWaker::get_peer_(esp_bd_addr_t addr, esp_ble_addr_type_t *type) {
  int num = esp_ble_get_bond_device_num();
  if (num <= 0)
    return false;
  std::vector<esp_ble_bond_dev_t> list(num);
  if (esp_ble_get_bond_device_list(&num, list.data()) != ESP_OK || num <= 0)
    return false;
  // The most recently stored bond is the host we paired last (the Steam Deck).
  const esp_ble_bond_dev_t &dev = list[num - 1];
  memcpy(addr, dev.bd_addr, sizeof(esp_bd_addr_t));
  *type = dev.bd_addr_type;
  return true;
}

bool DeckWaker::has_bond() {
  esp_bd_addr_t a;
  esp_ble_addr_type_t t;
  return this->get_peer_(a, &t);
}

std::string DeckWaker::bonded_peer_str() {
  if (!this->initialized_ && (this->parent_ == nullptr || !this->parent_->is_active()))
    return "unknown";
  esp_bd_addr_t a;
  esp_ble_addr_type_t t;
  if (!this->get_peer_(a, &t))
    return "none";
  char buf[24];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", a[0], a[1], a[2], a[3], a[4], a[5]);
  return buf;
}

bool DeckWaker::start_directed_() {
  esp_bd_addr_t peer;
  esp_ble_addr_type_t type;
  if (!this->get_peer_(peer, &type)) {
    ESP_LOGW(TAG, "No paired host. Pair the Steam Deck with '%s' first.", this->name_);
    return false;
  }
  esp_ble_adv_params_t p = {};
  p.adv_int_min = 0x0020;  // ignored for high duty directed
  p.adv_int_max = 0x0020;
  p.adv_type = ADV_TYPE_DIRECT_IND_HIGH;
  p.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
  memcpy(p.peer_addr, peer, sizeof(esp_bd_addr_t));
  p.peer_addr_type = type;
  p.channel_map = ADV_CHNL_ALL;
  p.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
  esp_err_t err = esp_ble_gap_start_advertising(&p);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "start directed advertising failed: %s", esp_err_to_name(err));
    return false;
  }
  this->adv_state_ = AdvState::DIRECTED;
  this->adv_state_since_ = millis();
  ESP_LOGI(TAG, "Knock: directed advertising to %02X:%02X:%02X:%02X:%02X:%02X (type %d)", peer[0], peer[1], peer[2],
           peer[3], peer[4], peer[5], type);
  return true;
}

void DeckWaker::stop_adv_() {
  esp_ble_gap_stop_advertising();
  this->adv_state_ = AdvState::STOPPING;
  this->adv_state_since_ = millis();
}

void DeckWaker::scanner_pause_(bool pause) {
#ifdef DECK_WAKER_PAUSE_SCAN
  if (!this->pause_scanner_ || pause == this->scanner_paused_)
    return;
  auto *tracker = esp32_ble_tracker::global_esp32_ble_tracker;
  if (tracker == nullptr)
    return;
  if (pause) {
    tracker->set_scan_continuous(false);
    tracker->stop_scan();
  } else {
    tracker->set_scan_continuous(true);
    tracker->start_scan();
  }
  this->scanner_paused_ = pause;
  ESP_LOGD(TAG, "BLE scanner %s", pause ? "paused" : "resumed");
#endif
}

void DeckWaker::begin_knock_() {
  if (this->knock_active_)
    return;
  set_coex_prefer_bt(true);
  this->coex_bt_ = true;
  this->scanner_pause_(true);
  this->knock_active_ = true;
  // Enough time for every attempt plus a margin for the connection to come up
  this->knock_deadline_ = millis() + this->direct_attempts_cfg_ * DIRECTED_WINDOW_MS + KNOCK_MARGIN_MS;
  this->direct_requested_ = true;
  this->direct_attempts_left_ = this->direct_attempts_cfg_;
}

void DeckWaker::end_knock_() {
  this->knock_active_ = false;
  this->direct_requested_ = false;
  if (this->coex_bt_) {
    set_coex_prefer_bt(false);
    this->coex_bt_ = false;
  }
  this->scanner_pause_(false);
}

void DeckWaker::loop() {
  if (this->init_failed_)
    return;
  if (!this->initialized_) {
    if (this->parent_ != nullptr && this->parent_->is_active())
      this->init_hid_();
    return;
  }

  const uint32_t now = millis();
  const bool connected = this->connected_.load();

  // Connection edges
  if (connected != this->was_connected_) {
    this->was_connected_ = connected;
    // The controller stops advertising when a connection is made; after a disconnect nothing advertises.
    this->adv_state_ = AdvState::IDLE;
    if (connected) {
      ESP_LOGI(TAG, "Host connected");
      if (this->knock_active_)
        this->end_knock_();
      if (this->pending_action_ != HostAction::NONE) {
        this->action_scheduled_ = true;
        this->action_at_ = now + ACTION_DELAY_AFTER_CONNECT_MS;
      }
    } else {
      ESP_LOGI(TAG, "Host disconnected");
      this->action_scheduled_ = false;
    }
  }

  if (this->knock_active_ && (int32_t) (now - this->knock_deadline_) > 0) {
    ESP_LOGW(TAG, "Knock timed out: host did not connect");
    this->end_knock_();
    this->pending_action_ = HostAction::NONE;
  }

  if (connected) {
    if (this->action_scheduled_ && (int32_t) (now - this->action_at_) >= 0) {
      this->action_scheduled_ = false;
      HostAction action = this->pending_action_;
      this->pending_action_ = HostAction::NONE;
      this->perform_(action, this->pending_key_);
    }
    return;
  }

  // Not connected: manage advertising
  if (this->adv_retry_after_ != 0 && (int32_t) (now - this->adv_retry_after_) < 0)
    return;
  this->adv_retry_after_ = 0;

  if (!this->adv_data_set_) {
    if (!this->adv_data_pending_)
      this->configure_adv_data_();
    return;
  }

  // Once a host is bonded we stay silent: a WakeAllowed host wakes up on *any* connectable advertising from
  // us, so undirected advertising is only used while pairing (no bond yet, or pairing mode requested).
  if (!this->bond_check_valid_ || now - this->bond_checked_at_ > 10000) {
    this->bond_cached_ = this->has_bond();
    this->bond_checked_at_ = now;
    this->bond_check_valid_ = true;
  }
  const bool want_undirected = !this->bond_cached_ || this->in_pairing_mode();

  switch (this->adv_state_) {
    case AdvState::IDLE:
      if (this->direct_requested_ && this->direct_attempts_left_ > 0) {
        this->direct_attempts_left_--;
        if (!this->start_directed_()) {
          this->end_knock_();
          this->pending_action_ = HostAction::NONE;
          if (want_undirected)
            this->start_undirected_();
        }
      } else {
        this->direct_requested_ = false;
        if (this->knock_active_) {
          ESP_LOGW(TAG, "All %u knocks sent, host did not connect", this->direct_attempts_cfg_);
          this->end_knock_();
          this->pending_action_ = HostAction::NONE;
        }
        if (want_undirected)
          this->start_undirected_();
      }
      break;
    case AdvState::UNDIRECTED:
      if (this->direct_requested_ || !want_undirected)
        this->stop_adv_();
      break;
    case AdvState::DIRECTED:
      if (now - this->adv_state_since_ > DIRECTED_WINDOW_MS) {
        // High duty directed advertising has expired without a connection; try again or give up.
        if (this->direct_attempts_left_ == 0)
          this->direct_requested_ = false;
        this->stop_adv_();
      }
      break;
    case AdvState::STOPPING:
      if (now - this->adv_state_since_ > 1000)
        this->adv_state_ = AdvState::IDLE;
      break;
  }
}

void DeckWaker::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (!this->initialized_)
    return;
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
      if (!this->adv_data_pending_)
        break;
      this->adv_data_pending_ = false;
      if (param->adv_data_raw_cmpl.status == ESP_BT_STATUS_SUCCESS) {
        this->adv_data_set_ = true;
      } else {
        ESP_LOGW(TAG, "Setting advertising data failed: %d", param->adv_data_raw_cmpl.status);
        this->adv_retry_after_ = millis() + 5000;
      }
      break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS &&
          (this->adv_state_ == AdvState::UNDIRECTED || this->adv_state_ == AdvState::DIRECTED)) {
        ESP_LOGW(TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
        this->adv_state_ = AdvState::IDLE;
        this->adv_retry_after_ = millis() + 2000;
      }
      break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
      if (this->adv_state_ == AdvState::STOPPING)
        this->adv_state_ = AdvState::IDLE;
      break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      const auto &a = param->ble_security.auth_cmpl;
      if (a.success) {
        this->bond_check_valid_ = false;
        this->pairing_until_ = 0;  // paired: go quiet so the host can sleep
        ESP_LOGI(TAG, "Encrypted link with %02X:%02X:%02X:%02X:%02X:%02X", a.bd_addr[0], a.bd_addr[1], a.bd_addr[2],
                 a.bd_addr[3], a.bd_addr[4], a.bd_addr[5]);
      } else {
        ESP_LOGW(TAG, "Pairing with %02X:%02X:%02X:%02X:%02X:%02X failed, reason 0x%x", a.bd_addr[0], a.bd_addr[1],
                 a.bd_addr[2], a.bd_addr[3], a.bd_addr[4], a.bd_addr[5], a.fail_reason);
      }
      break;
    }
    default:
      break;
  }
}

void DeckWaker::request_(HostAction action, uint8_t key) {
  if (!this->initialized_) {
    ESP_LOGW(TAG, "BLE not ready yet");
    return;
  }
  if (this->connected_.load()) {
    if (action == HostAction::NONE) {
      ESP_LOGI(TAG, "Host already connected (awake)");
    } else {
      this->perform_(action, key);
    }
    return;
  }
  if (!this->has_bond()) {
    ESP_LOGW(TAG, "No paired host yet. Pair the Steam Deck with '%s' first.", this->name_);
    return;
  }
  this->pending_action_ = action;
  this->pending_key_ = key;
  ESP_LOGI(TAG, "Knocking (%u attempts)%s", this->direct_attempts_cfg_,
           action == HostAction::NONE ? "" : ", action queued until connected");
  this->begin_knock_();
}

void DeckWaker::wake() { this->request_(HostAction::NONE, 0); }
void DeckWaker::send_key(uint8_t key) { this->request_(HostAction::KEY, key); }
void DeckWaker::sleep_host() { this->request_(HostAction::SYSTEM_SLEEP, 0); }
void DeckWaker::power_key() { this->request_(HostAction::SYSTEM_POWER, 0); }

void DeckWaker::perform_(HostAction action, uint8_t key) {
  switch (action) {
    case HostAction::KEY: {
      uint8_t r[8] = {0, 0, key, 0, 0, 0, 0, 0};
      ESP_LOGI(TAG, "Sending key 0x%02X", key);
      this->send_report_(REPORT_ID_KEYBOARD, r, sizeof(r));
      break;
    }
    case HostAction::SYSTEM_SLEEP: {
      uint8_t r = SYSTEM_SLEEP;
      ESP_LOGI(TAG, "Sending System Sleep");
      this->send_report_(REPORT_ID_SYSTEM, &r, 1);
      break;
    }
    case HostAction::SYSTEM_POWER: {
      uint8_t r = SYSTEM_POWER_DOWN;
      ESP_LOGI(TAG, "Sending System Power");
      this->send_report_(REPORT_ID_SYSTEM, &r, 1);
      break;
    }
    default:
      break;
  }
}

void DeckWaker::send_report_(uint8_t report_id, const uint8_t *data, uint8_t len) {
  if (this->hid_dev_ == nullptr || !this->connected_.load()) {
    ESP_LOGW(TAG, "Cannot send report: not connected");
    return;
  }
  bool expected = false;
  if (!this->report_busy_.compare_exchange_strong(expected, true)) {
    ESP_LOGW(TAG, "Report send already in progress");
    return;
  }
  this->report_id_ = report_id;
  this->report_len_ = len;
  memcpy(this->report_buf_, data, len);
  if (xTaskCreate(report_task, "deck_report", 3072, this, 5, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "Could not start report task");
    this->report_busy_.store(false);
  }
}

void DeckWaker::clear_bonds() {
  int num = esp_ble_get_bond_device_num();
  if (num <= 0) {
    ESP_LOGI(TAG, "No bonds to clear");
    return;
  }
  std::vector<esp_ble_bond_dev_t> list(num);
  esp_ble_get_bond_device_list(&num, list.data());
  for (int i = 0; i < num; i++)
    esp_ble_remove_bond_device(list[i].bd_addr);
  this->bond_check_valid_ = false;
  ESP_LOGI(TAG, "Cleared %d bond(s). Pair the Steam Deck again.", num);
}

void DeckWaker::start_pairing_mode(uint32_t seconds) {
  this->pairing_until_ = millis() + seconds * 1000;
  if (this->pairing_until_ == 0)
    this->pairing_until_ = 1;
  ESP_LOGI(TAG, "Pairing mode: advertising as '%s' for %u s", this->name_, (unsigned) seconds);
}

bool DeckWaker::in_pairing_mode() const {
  return this->pairing_until_ != 0 && (int32_t) (millis() - this->pairing_until_) < 0;
}

}  // namespace esphome::deck_waker
