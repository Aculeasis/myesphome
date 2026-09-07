#include "zigbee_batch.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "esphome/core/log.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"


namespace esphome {
namespace zigbee_batch {


static const char *const TAG = "zigbee_batch";


static void put_u16_le(uint8_t *dst, uint16_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}


static void put_u32_le(uint8_t *dst, uint32_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  dst[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  dst[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}


static uint32_t now_ms_32() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}


void ZigbeeBatchComponent::dump_config() {
  ESP_LOGCONFIG(
      TAG,
      "Zigbee Batch:\n"
      "  Source endpoint: %u\n"
      "  Destination endpoint: %u\n"
      "  Cluster: 0x%04X\n"
      "  Command: 0x%02X\n"
      "  Payload version: %u\n"
      "  Payload size: %u bytes",
      this->endpoint_,
      this->destination_endpoint_,
      this->cluster_id_,
      this->command_id_,
      PAYLOAD_VERSION,
      static_cast<unsigned>(PAYLOAD_SIZE));
}


void ZigbeeBatchComponent::build_payload_(
    float temperature_c,
    float humidity_pct,
    float battery_voltage_v,
    float battery_pct,
    float uptime_s,
    uint32_t previous_tx_wait_ms,
    bool previous_tx_wait_valid) {
  this->payload_.fill(0);
  this->payload_[0] = PAYLOAD_VERSION;

  uint8_t flags = 0;

  int16_t temperature_raw = std::numeric_limits<int16_t>::min();
  if (std::isfinite(temperature_c)) {
    long value = std::lround(static_cast<double>(temperature_c) * 100.0);
    if (value < -32767L)
      value = -32767L;
    if (value > 32767L)
      value = 32767L;
    temperature_raw = static_cast<int16_t>(value);
    flags |= (1U << 0);
  }

  uint16_t humidity_raw = std::numeric_limits<uint16_t>::max();
  if (std::isfinite(humidity_pct)) {
    double value = humidity_pct;
    if (value < 0.0)
      value = 0.0;
    if (value > 655.34)
      value = 655.34;
    humidity_raw = static_cast<uint16_t>(std::lround(value * 100.0));
    flags |= (1U << 1);
  }

  uint16_t battery_mv = std::numeric_limits<uint16_t>::max();
  if (std::isfinite(battery_voltage_v) && battery_voltage_v >= 0.0f) {
    double value = static_cast<double>(battery_voltage_v) * 1000.0;
    if (value > 65534.0)
      value = 65534.0;
    battery_mv = static_cast<uint16_t>(std::lround(value));
    flags |= (1U << 2);
  }

  uint8_t battery_pct_raw = std::numeric_limits<uint8_t>::max();
  if (std::isfinite(battery_pct)) {
    double value = battery_pct;
    if (value < 0.0)
      value = 0.0;
    if (value > 100.0)
      value = 100.0;
    battery_pct_raw = static_cast<uint8_t>(std::lround(value));
    flags |= (1U << 3);
  }

  uint32_t uptime_raw = std::numeric_limits<uint32_t>::max();
  if (std::isfinite(uptime_s) && uptime_s >= 0.0f) {
    double value = uptime_s;
    constexpr double MAX_VALID_UPTIME =
        static_cast<double>(std::numeric_limits<uint32_t>::max() - 1U);
    if (value > MAX_VALID_UPTIME)
      value = MAX_VALID_UPTIME;
    uptime_raw = static_cast<uint32_t>(std::llround(value));
    flags |= (1U << 4);
  }

  uint16_t previous_wait_raw = 0;
  if (previous_tx_wait_valid) {
    previous_wait_raw = previous_tx_wait_ms > 65535U
                            ? 65535U
                            : static_cast<uint16_t>(previous_tx_wait_ms);
    flags |= (1U << 5);
  }

  this->payload_[1] = flags;
  put_u16_le(&this->payload_[2], static_cast<uint16_t>(temperature_raw));
  put_u16_le(&this->payload_[4], humidity_raw);
  put_u16_le(&this->payload_[6], battery_mv);
  this->payload_[8] = battery_pct_raw;
  put_u32_le(&this->payload_[9], uptime_raw);
  put_u16_le(&this->payload_[13], previous_wait_raw);
}


bool ZigbeeBatchComponent::send(
    float temperature_c,
    float humidity_pct,
    float battery_voltage_v,
    float battery_pct,
    float uptime_s,
    uint32_t previous_tx_wait_ms,
    bool previous_tx_wait_valid) {
  if (this->zb_ == nullptr) {
    ESP_LOGW(TAG, "Cannot send: Zigbee component is null");
    return false;
  }
  if (!this->zb_->is_started()) {
    ESP_LOGW(TAG, "Cannot send: Zigbee stack not started");
    return false;
  }
  if (!this->zb_->is_joined()) {
    ESP_LOGW(TAG, "Cannot send: device is not joined");
    return false;
  }

  uint8_t expected = 0;
  if (!this->tx_pending_.compare_exchange_strong(
          expected, 1, std::memory_order_acq_rel, std::memory_order_acquire)) {
    ESP_LOGW(TAG, "Cannot send: previous batch TX is still pending");
    return false;
  }

  this->build_payload_(temperature_c, humidity_pct, battery_voltage_v,
                       battery_pct, uptime_s, previous_tx_wait_ms,
                       previous_tx_wait_valid);
  this->last_tx_status_.store(0xFF, std::memory_order_relaxed);
  this->tx_started_ms_.store(now_ms_32(), std::memory_order_relaxed);

  if (!esp_zigbee_lock_acquire(pdMS_TO_TICKS(50))) {
    ESP_LOGW(TAG, "Could not acquire Zigbee lock");
    this->tx_pending_.store(0, std::memory_order_release);
    return false;
  }

  ezb_zcl_custom_cluster_cmd_t cmd = {};
  cmd.cmd_ctrl.fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI;
  cmd.cmd_ctrl.fc.dis_default_rsp = 1;
  cmd.cmd_ctrl.fc.manuf_specific = 0;
  cmd.cmd_ctrl.dst_addr.addr_mode = EZB_ADDR_MODE_SHORT;
  cmd.cmd_ctrl.dst_addr.u.short_addr = 0x0000;
  cmd.cmd_ctrl.src_ep = this->endpoint_;
  cmd.cmd_ctrl.dst_ep = this->destination_endpoint_;
  cmd.cmd_ctrl.cluster_id = this->cluster_id_;
  cmd.cmd_ctrl.cnf_ctx.cb = &ZigbeeBatchComponent::tx_confirm_;
  cmd.cmd_ctrl.cnf_ctx.user_ctx = this;
  cmd.cmd_id = this->command_id_;
  cmd.data_length = static_cast<uint16_t>(this->payload_.size());
  cmd.data = this->payload_.data();

  ezb_err_t err = ezb_zcl_custom_cluster_cmd_req(&cmd);
  esp_zigbee_lock_release();

  if (err != EZB_ERR_NONE) {
    ESP_LOGW(TAG, "Could not queue custom Zigbee batch: 0x%X",
             static_cast<unsigned>(err));
    this->tx_pending_.store(0, std::memory_order_release);
    return false;
  }

  ESP_LOGD(TAG,
           "Queued batch: temp=%.2f humidity=%.2f battery=%.3fV/%.0f%% "
           "uptime=%.0fs previous_wait=%s%ums",
           temperature_c, humidity_pct, battery_voltage_v, battery_pct,
           uptime_s, previous_tx_wait_valid ? "" : "invalid/",
           static_cast<unsigned>(previous_tx_wait_ms));
  return true;
}


void ZigbeeBatchComponent::tx_confirm_(ezb_zcl_cmd_cnf_t *cnf,
                                        void *user_ctx) {
  auto *self = static_cast<ZigbeeBatchComponent *>(user_ctx);
  if (self == nullptr)
    return;

  const uint32_t started = self->tx_started_ms_.load(std::memory_order_relaxed);
  const uint32_t elapsed = now_ms_32() - started;
  self->last_confirm_latency_ms_.store(elapsed, std::memory_order_relaxed);

  uint8_t status = 0xFF;
  if (cnf != nullptr)
    status = static_cast<uint8_t>(cnf->status);
  self->last_tx_status_.store(status, std::memory_order_relaxed);

  if (cnf == nullptr) {
    ESP_LOGW(TAG, "Batch TX confirmation is null");
  } else if (cnf->status != 0) {
    ESP_LOGW(TAG, "Batch TX failed, status=0x%02X, latency=%ums",
             static_cast<unsigned>(cnf->status),
             static_cast<unsigned>(elapsed));
  } else {
    ESP_LOGD(TAG, "Batch TX confirmed in %ums", static_cast<unsigned>(elapsed));
  }

  // Release last. is_idle() uses acquire semantics, so observing idle=true
  // also observes the status and latency written above.
  self->tx_pending_.store(0, std::memory_order_release);
}


}  // namespace zigbee_batch
}  // namespace esphome
