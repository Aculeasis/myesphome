#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esphome/components/zigbee/zigbee_esp32.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "esp_zigbee.h"
#include "esp_attr.h"
#include "ezbee/zcl/cluster/custom.h"

namespace esphome {
namespace zigbee_batch {

enum class DataType : uint8_t { UINT8, UINT16, UINT32, INT8, INT16, INT32 };

class ZigbeeBatchComponent : public Component {
 public:
  static constexpr size_t MAX_DATA_FIELDS = 21;
  static constexpr size_t MAX_FLAG_BYTES = 3;
  static constexpr size_t MAX_PAYLOAD_SIZE =
      MAX_FLAG_BYTES + MAX_DATA_FIELDS * sizeof(uint32_t);
  static constexpr size_t DIAGNOSTIC_PAYLOAD_SIZE = 7;
  static constexpr uint8_t DIAGNOSTIC_PAYLOAD_VERSION = 2;
  static constexpr size_t MAX_TX_CONTEXTS = 16;

  void set_zigbee(zigbee::ZigbeeComponent *zb) { this->zb_ = zb; }
  void set_endpoint(uint8_t endpoint) { this->endpoint_ = endpoint; }
  void set_destination_endpoint(uint8_t endpoint) {
    this->destination_endpoint_ = endpoint;
  }
  void set_cluster_id(uint16_t cluster_id) { this->cluster_id_ = cluster_id; }
  void set_diagnostic_cluster_id(uint16_t cluster_id) {
    this->diagnostic_cluster_id_ = cluster_id;
  }
  void set_command_id(uint8_t command_id) { this->command_id_ = command_id; }
  void set_data_field_count(size_t count) { this->data_field_count_ = count; }
  void set_data_type(size_t index, DataType type) {
    if (index < MAX_DATA_FIELDS)
      this->data_types_[index] = type;
  }
  void set_noncritical_error_limit(uint8_t limit) {
    this->noncritical_error_limit_ = limit;
  }
  void set_wakeup_pin(uint8_t pin) { this->wakeup_pin_ = pin; }

  void setup() override;
  void loop() override;
  void dump_config() override {}

  // Call begin(), then add() or skip() for fields in declaration order, and
  // finally send(). Trailing fields not supplied before send() are skipped.
  // Unchanged values are automatically omitted.
  void begin();
  void add(float value);
  void skip();
  bool send();

  bool is_idle() const {
    return this->tx_pending_.load(std::memory_order_acquire) == 0;
  }
  uint32_t last_confirm_latency_ms() const {
    return this->last_confirm_latency_ms_.load(std::memory_order_relaxed);
  }
  uint8_t last_tx_status() const {
    return this->last_tx_status_.load(std::memory_order_relaxed);
  }
  bool last_tx_success() const { return this->last_tx_status() == 0; }

  // Stop waiting after the application timeout. A late callback keeps its own
  // context and is ignored, so a later packet cannot be mistaken for it.
  void handle_confirm_timeout();
  void suspend_button_wakeup();
  void resume_button_wakeup();
  // Block only the ESPHome main task. FreeRTOS tickless idle keeps the chip in
  // automatic Light Sleep while the Zigbee task remains free to service its
  // own keepalive/rejoin deadlines. Returns 0 at the regular deadline, 1 for
  // a button wake, or 2 when the noncritical-error limit requires a restart.
  int wait_until(int64_t deadline_us);
  void wait_for_duration(uint32_t duration_ms);
  bool restart_required() const {
    return this->restart_required_.load(std::memory_order_acquire);
  }

  void record_error(uint16_t code);
  void prepare_restart(uint16_t code);
  bool consume_restart_cooldown();
  void finish_tx();

  uint16_t first_error_code() const {
    return this->first_error_code_.load(std::memory_order_acquire);
  }
  uint32_t error_sequence() const {
    return this->error_sequence_.load(std::memory_order_acquire);
  }

 protected:
  enum TxState : uint8_t {
    TX_FREE = 0,
    TX_ACTIVE = 1,
    TX_ABANDONED = 2,
    TX_CALLBACK = 3,
  };

  struct TxContext {
    std::atomic<uint8_t> state{TX_FREE};
    ZigbeeBatchComponent *owner{nullptr};
    bool diagnostic{false};
    std::array<uint8_t, MAX_PAYLOAD_SIZE> payload{};
    uint16_t payload_size{0};
    uint32_t changed_mask{0};
    std::array<uint32_t, MAX_DATA_FIELDS> values{};
  };

  struct __attribute__((packed)) DiagnosticState {
    uint16_t code;
    uint32_t sequence;
  };
  static_assert(sizeof(DiagnosticState) == 6,
                "DiagnosticState must have a stable flash layout");

  static bool app_signal_handler_(const ezb_app_signal_t *app_signal);
  static void tx_confirm_(ezb_zcl_cmd_cnf_t *cnf, void *user_ctx);
  static void IRAM_ATTR button_interrupt_(void *user_context);

  TxContext *acquire_tx_context_();
  bool queue_context_(TxContext *context, bool acquire_lock = true);
  bool queue_diagnostic_(TxContext *context, uint16_t code,
                         uint32_t sequence);
  void finish_context_(TxContext *context);
  void commit_sent_values_(const TxContext *context);

  static size_t data_type_size_(DataType type);
  static uint32_t encode_value_(DataType type, float value);
  static void put_value_(uint8_t *dst, uint32_t value, size_t size);

  void note_noncritical_error_(uint16_t code, bool callback_context);
  void reset_noncritical_errors_();
  void process_button_interrupt_();
  void arm_button_interrupt_();
  void service_pending_rejoin_();
  void queue_callback_error_(uint16_t code);
  void commit_callback_error_();
  void clear_error_();
  void save_diagnostic_state_(uint16_t code, uint32_t sequence);

  zigbee::ZigbeeComponent *zb_{nullptr};
  uint8_t endpoint_{1};
  uint8_t destination_endpoint_{1};
  uint16_t cluster_id_{0xFC01};
  uint16_t diagnostic_cluster_id_{0xFC02};
  uint8_t command_id_{0};

  size_t data_field_count_{0};
  std::array<DataType, MAX_DATA_FIELDS> data_types_{};
  size_t builder_index_{0};
  uint32_t builder_changed_mask_{0};
  std::array<uint32_t, MAX_DATA_FIELDS> builder_values_{};
  uint32_t sent_value_valid_mask_{0};
  std::array<uint32_t, MAX_DATA_FIELDS> sent_values_{};

  std::array<TxContext, MAX_TX_CONTEXTS> tx_contexts_{};
  std::atomic<TxContext *> current_context_{nullptr};
  std::atomic<uint8_t> tx_pending_{0};
  std::atomic<uint32_t> tx_started_ms_{0};
  std::atomic<uint32_t> last_confirm_latency_ms_{0};
  std::atomic<uint8_t> last_tx_status_{0xFF};

  uint8_t noncritical_error_limit_{5};
  std::atomic<uint8_t> consecutive_noncritical_errors_{0};
  std::atomic<bool> restart_required_{false};
  std::atomic<bool> rejoin_requested_{false};
  std::atomic<bool> rejoin_in_progress_{false};

  uint8_t wakeup_pin_{0xFF};
  bool button_handler_ready_{false};
  bool button_wakeup_suspended_{false};
  std::atomic<bool> button_interrupt_pending_{false};

  ESPPreferenceObject diagnostic_preference_;
  ESPPreferenceObject restart_cooldown_preference_;
  std::atomic<uint16_t> first_error_code_{0};
  std::atomic<uint32_t> error_sequence_{0};
  std::atomic<uint16_t> callback_error_code_{0};
  std::atomic<uint16_t> transmitted_error_code_{0};
  std::atomic<uint32_t> transmitted_error_sequence_{0};
  std::atomic<bool> restart_cooldown_{false};

  static ZigbeeBatchComponent *instance_;
};

}  // namespace zigbee_batch
}  // namespace esphome
