#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/zigbee/zigbee_esp32.h"

#include "esp_zigbee.h"
#include "ezbee/zcl/cluster/custom.h"


namespace esphome {
namespace zigbee_batch {


class ZigbeeBatchComponent : public Component {
 public:
  static constexpr uint8_t PAYLOAD_VERSION = 1;
  static constexpr size_t PAYLOAD_SIZE = 15;
  static constexpr uint8_t DIAGNOSTIC_PAYLOAD_VERSION = 2;
  static constexpr size_t DIAGNOSTIC_PAYLOAD_SIZE = 7;

  void set_zigbee(zigbee::ZigbeeComponent *zb) {
    this->zb_ = zb;
  }

  void set_endpoint(uint8_t endpoint) {
    this->endpoint_ = endpoint;
  }

  void set_destination_endpoint(uint8_t endpoint) {
    this->destination_endpoint_ = endpoint;
  }

  void set_cluster_id(uint16_t cluster_id) {
    this->cluster_id_ = cluster_id;
  }

  void set_diagnostic_cluster_id(uint16_t cluster_id) {
    this->diagnostic_cluster_id_ = cluster_id;
  }

  void set_command_id(uint8_t command_id) {
    this->command_id_ = command_id;
  }

  void setup() override;
  void dump_config() override {}

  // previous_tx_wait_ms is the wait duration measured by the previous
  // measurement cycle.
  //
  // Return true only if the Zigbee command was successfully queued.
  bool send(
      float temperature_c,
      float humidity_pct,
      float battery_voltage_v,
      float battery_pct,
      float uptime_s,
      uint32_t previous_tx_wait_ms,
      bool previous_tx_wait_valid);

  // Becomes false immediately before queueing the custom command.
  // Becomes true only when TX confirmation callback fires, or when queueing
  // fails before ownership of the command was accepted by Zigbee.
  bool is_idle() const {
    return this->tx_pending_.load(std::memory_order_acquire) == 0;
  }

  // Diagnostic value measured internally from send() to TX confirm.
  // This is NOT the same metric as the YAML wait_until latency.
  uint32_t last_confirm_latency_ms() const {
    return this->last_confirm_latency_ms_.load(
        std::memory_order_relaxed);
  }

  uint8_t last_tx_status() const {
    return this->last_tx_status_.load(
        std::memory_order_relaxed);
  }

  bool last_tx_success() const {
    return this->last_tx_status() == 0;
  }

  // Persist the first failure only. It is delivered later through the
  // dedicated diagnostic cluster.
  void record_error(uint16_t code);

  // Persist the error and a cooldown before an emergency restart.
  void prepare_restart(uint16_t code);

  // Returns true once after such a restart and clears the persisted flag.
  bool consume_restart_cooldown();

  // Commit errors reported from Zigbee callbacks and clear a diagnostic only
  // after the packet containing that diagnostic was successfully confirmed.
  void finish_tx();

  uint16_t first_error_code() const {
    return this->first_error_code_.load(std::memory_order_acquire);
  }

  uint32_t error_sequence() const {
    return this->error_sequence_.load(std::memory_order_acquire);
  }

 protected:
  struct __attribute__((packed)) DiagnosticState {
    uint16_t code;
    uint32_t sequence;
  };
  static_assert(sizeof(DiagnosticState) == 6,
                "DiagnosticState must have a stable flash layout");

  static bool app_signal_handler_(const ezb_app_signal_t *app_signal);
  static void tx_confirm_(
      ezb_zcl_cmd_cnf_t *cnf,
      void *user_ctx);

  void queue_callback_error_(uint16_t code);
  void commit_callback_error_();
  void clear_error_();
  void save_diagnostic_state_(uint16_t code, uint32_t sequence);
  bool queue_diagnostic_(uint16_t code, uint32_t sequence);

  void build_payload_(
      float temperature_c,
      float humidity_pct,
      float battery_voltage_v,
      float battery_pct,
      float uptime_s,
      uint32_t previous_tx_wait_ms,
      bool previous_tx_wait_valid);

  zigbee::ZigbeeComponent *zb_{nullptr};

  uint8_t endpoint_{1};
  uint8_t destination_endpoint_{1};

  uint16_t cluster_id_{0xFC01};
  uint16_t diagnostic_cluster_id_{0xFC02};
  uint8_t command_id_{0x00};

  // The payload is a member, not a temporary local buffer, so it remains
  // valid until the asynchronous TX confirmation.
  std::array<uint8_t, PAYLOAD_SIZE> payload_{};
  std::array<uint8_t, DIAGNOSTIC_PAYLOAD_SIZE> diagnostic_payload_{};

  std::atomic<uint8_t> tx_pending_{0};

  std::atomic<uint32_t> tx_started_ms_{0};
  std::atomic<uint32_t> last_confirm_latency_ms_{0};

  // 0 == successful AF confirmation.
  // 0xFF == no confirmation yet / unavailable.
  std::atomic<uint8_t> last_tx_status_{0xFF};

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
