#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/components/zigbee/zigbee_esp32.h"

#include "esp_zigbee.h"
#include "ezbee/zcl/cluster/custom.h"


namespace esphome {
namespace zigbee_batch {


class ZigbeeBatchComponent : public Component {
 public:
  static constexpr uint8_t PAYLOAD_VERSION = 1;
  static constexpr size_t PAYLOAD_SIZE = 15;

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

  void set_command_id(uint8_t command_id) {
    this->command_id_ = command_id;
  }

  void dump_config() override;

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

 protected:
  static void tx_confirm_(
      ezb_zcl_cmd_cnf_t *cnf,
      void *user_ctx);

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
  uint8_t command_id_{0x00};

  // The payload is a member, not a temporary local buffer, so it remains
  // valid until the asynchronous TX confirmation.
  std::array<uint8_t, PAYLOAD_SIZE> payload_{};

  std::atomic<uint8_t> tx_pending_{0};

  std::atomic<uint32_t> tx_started_ms_{0};
  std::atomic<uint32_t> last_confirm_latency_ms_{0};

  // 0 == successful AF confirmation.
  // 0xFF == no confirmation yet / unavailable.
  std::atomic<uint8_t> last_tx_status_{0xFF};
};


}  // namespace zigbee_batch
}  // namespace esphome
