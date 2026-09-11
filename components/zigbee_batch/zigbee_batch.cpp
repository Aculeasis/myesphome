#include "zigbee_batch.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "esphome/core/application.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"


namespace esphome {
namespace zigbee_batch {


static constexpr uint32_t LEGACY_ERROR_PREFERENCE_KEY = 0x6E7A4245U;
static constexpr uint32_t RESTART_COOLDOWN_PREFERENCE_KEY = 0x6E7A4246U;
static constexpr uint32_t DIAGNOSTIC_PREFERENCE_KEY = 0x6E7A4247U;

static constexpr uint16_t ERROR_COMPONENT_NULL = 60001;
static constexpr uint16_t ERROR_STACK_NOT_STARTED = 60002;
static constexpr uint16_t ERROR_NOT_JOINED = 60003;
static constexpr uint16_t ERROR_TX_BUSY = 60004;
static constexpr uint16_t ERROR_LOCK_FAILED = 60005;
static constexpr uint16_t ERROR_QUEUE_FAILED = 60006;
static constexpr uint16_t ERROR_CONFIRM_NULL = 60008;
static constexpr uint16_t ERROR_PARENT_LINK_FAILURE = 60011;
static constexpr uint16_t ERROR_DEVICE_REBOOT_FAILED = 60012;
static constexpr uint16_t ERROR_CONFIG_LOCK_FAILED = 60013;
static constexpr uint16_t ERROR_DIAGNOSTIC_QUEUE_FAILED = 60014;
static constexpr uint16_t ERROR_CONFIRM_STATUS_BASE = 61000;

ZigbeeBatchComponent *ZigbeeBatchComponent::instance_ = nullptr;


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


void ZigbeeBatchComponent::setup() {
  this->diagnostic_preference_ =
      global_preferences->make_preference<DiagnosticState>(
          DIAGNOSTIC_PREFERENCE_KEY);
  this->restart_cooldown_preference_ =
      global_preferences->make_preference<bool>(
          RESTART_COOLDOWN_PREFERENCE_KEY);

  DiagnosticState stored_diagnostic{};
  if (this->diagnostic_preference_.load(&stored_diagnostic)) {
    this->first_error_code_.store(stored_diagnostic.code,
                                  std::memory_order_release);
    this->error_sequence_.store(stored_diagnostic.sequence,
                                std::memory_order_release);
  } else {
    auto legacy_error_preference =
        global_preferences->make_preference<uint16_t>(
            LEGACY_ERROR_PREFERENCE_KEY);
    uint16_t legacy_error = 0;
    if (legacy_error_preference.load(&legacy_error) && legacy_error != 0) {
      this->first_error_code_.store(legacy_error,
                                    std::memory_order_release);
      this->error_sequence_.store(1, std::memory_order_release);
      this->save_diagnostic_state_(legacy_error, 1);
    }
  }

  bool restart_cooldown = false;
  if (this->restart_cooldown_preference_.load(&restart_cooldown))
    this->restart_cooldown_.store(restart_cooldown,
                                  std::memory_order_release);

  // 2048 minutes is the smallest supported timeout that is at least one day.
  // Keepalive matches the normal measurement period, so this does not add
  // periodic radio wakeups. Never wait indefinitely for the Zigbee lock.
  if (esp_zigbee_lock_acquire(pdMS_TO_TICKS(50))) {
    ezb_nwk_set_ed_timeout(EZB_NWK_ED_TIMEOUT_2048MIN);
    ezb_nwk_set_keepalive_interval(600000U);
    ezb_nwk_set_rx_on_when_idle(false);
    ezb_nwk_set_fast_poll_interval(30U);
    esp_zigbee_lock_release();
  } else {
    this->record_error(ERROR_CONFIG_LOCK_FAILED);
  }

  instance_ = this;
  ezb_app_signal_add_handler(ZigbeeBatchComponent::app_signal_handler_);
}


void ZigbeeBatchComponent::save_diagnostic_state_(uint16_t code,
                                                   uint32_t sequence) {
  const DiagnosticState state{code, sequence};
  this->diagnostic_preference_.save(&state);
  global_preferences->sync();
}


void ZigbeeBatchComponent::record_error(uint16_t code) {
  if (code == 0)
    return;

  uint16_t expected = 0;
  if (!this->first_error_code_.compare_exchange_strong(
          expected, code, std::memory_order_acq_rel,
          std::memory_order_acquire))
    return;

  uint32_t sequence = this->error_sequence_.load(std::memory_order_relaxed);
  sequence = sequence == std::numeric_limits<uint32_t>::max()
                 ? 1
                 : sequence + 1;
  this->error_sequence_.store(sequence, std::memory_order_release);
  this->save_diagnostic_state_(code, sequence);
}


void ZigbeeBatchComponent::prepare_restart(uint16_t code) {
  this->record_error(code);
  const bool restart_cooldown = true;
  this->restart_cooldown_.store(true, std::memory_order_release);
  this->restart_cooldown_preference_.save(&restart_cooldown);
  global_preferences->sync();
}


bool ZigbeeBatchComponent::consume_restart_cooldown() {
  if (!this->restart_cooldown_.exchange(false, std::memory_order_acq_rel))
    return false;

  const bool restart_cooldown = false;
  this->restart_cooldown_preference_.save(&restart_cooldown);
  global_preferences->sync();
  return true;
}


void ZigbeeBatchComponent::clear_error_() {
  const uint16_t cleared = 0;
  this->first_error_code_.store(cleared, std::memory_order_release);
  this->save_diagnostic_state_(cleared, this->error_sequence());
}


void ZigbeeBatchComponent::queue_callback_error_(uint16_t code) {
  if (code == 0)
    return;
  uint16_t expected = 0;
  this->callback_error_code_.compare_exchange_strong(
      expected, code, std::memory_order_acq_rel,
      std::memory_order_acquire);
}


void ZigbeeBatchComponent::commit_callback_error_() {
  const uint16_t code =
      this->callback_error_code_.exchange(0, std::memory_order_acq_rel);
  this->record_error(code);
}


void ZigbeeBatchComponent::finish_tx() {
  const uint16_t transmitted =
      this->transmitted_error_code_.exchange(0, std::memory_order_acq_rel);
  const uint32_t transmitted_sequence =
      this->transmitted_error_sequence_.exchange(
          0, std::memory_order_acq_rel);
  if (this->last_tx_success() && transmitted != 0 &&
      this->first_error_code() == transmitted &&
      this->error_sequence() == transmitted_sequence) {
    this->clear_error_();
  }

  // If a new failure occurred while an older diagnostic was being delivered,
  // preserve the new one after the delivered code has been cleared.
  this->commit_callback_error_();

}


bool ZigbeeBatchComponent::app_signal_handler_(
    const ezb_app_signal_t *app_signal) {
  auto *self = instance_;
  if (self == nullptr || app_signal == nullptr)
    return true;

  const ezb_app_signal_type_t type = ezb_app_signal_get_type(app_signal);
  if (type == EZB_NWK_SIGNAL_NETWORK_STATUS) {
    const auto *params = static_cast<const ezb_nwk_signal_network_status_params_t *>(
        ezb_app_signal_get_params(app_signal));
    if (params != nullptr &&
        params->status == EZB_NWK_NETWORK_STATUS_PARENT_LINK_FAILURE) {
      self->queue_callback_error_(ERROR_PARENT_LINK_FAILURE);
    }
  } else if (type == EZB_BDB_SIGNAL_DEVICE_REBOOT) {
    const auto *status = static_cast<const ezb_bdb_comm_status_t *>(
        ezb_app_signal_get_params(app_signal));
    if (status != nullptr && *status != EZB_BDB_STATUS_SUCCESS)
      self->queue_callback_error_(ERROR_DEVICE_REBOOT_FAILED);
  }
  return true;
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
  this->commit_callback_error_();

  if (this->zb_ == nullptr) {
    this->record_error(ERROR_COMPONENT_NULL);
    return false;
  }
  if (!this->zb_->is_started()) {
    this->record_error(ERROR_STACK_NOT_STARTED);
    return false;
  }
  if (!this->zb_->is_joined()) {
    this->record_error(ERROR_NOT_JOINED);
    return false;
  }

  uint8_t expected = 0;
  if (!this->tx_pending_.compare_exchange_strong(
          expected, 1, std::memory_order_acq_rel, std::memory_order_acquire)) {
    this->record_error(ERROR_TX_BUSY);
    return false;
  }

  this->transmitted_error_code_.store(0, std::memory_order_release);
  this->transmitted_error_sequence_.store(0, std::memory_order_release);

  this->build_payload_(temperature_c, humidity_pct, battery_voltage_v,
                       battery_pct, uptime_s, previous_tx_wait_ms,
                       previous_tx_wait_valid);
  this->last_tx_status_.store(0xFF, std::memory_order_relaxed);
  this->tx_started_ms_.store(now_ms_32(), std::memory_order_relaxed);

  if (!esp_zigbee_lock_acquire(pdMS_TO_TICKS(50))) {
    this->record_error(ERROR_LOCK_FAILED);
    this->transmitted_error_code_.store(0, std::memory_order_release);
    this->transmitted_error_sequence_.store(0,
                                             std::memory_order_release);
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
    this->record_error(ERROR_QUEUE_FAILED);
    this->transmitted_error_code_.store(0, std::memory_order_release);
    this->transmitted_error_sequence_.store(0,
                                             std::memory_order_release);
    this->tx_pending_.store(0, std::memory_order_release);
    return false;
  }
  return true;
}


bool ZigbeeBatchComponent::queue_diagnostic_(uint16_t code,
                                             uint32_t sequence) {
  this->diagnostic_payload_[0] = DIAGNOSTIC_PAYLOAD_VERSION;
  put_u16_le(&this->diagnostic_payload_[1], code);
  put_u32_le(&this->diagnostic_payload_[3], sequence);

  ezb_zcl_custom_cluster_cmd_t cmd = {};
  cmd.cmd_ctrl.fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI;
  cmd.cmd_ctrl.fc.dis_default_rsp = 1;
  cmd.cmd_ctrl.fc.manuf_specific = 0;
  cmd.cmd_ctrl.dst_addr.addr_mode = EZB_ADDR_MODE_SHORT;
  cmd.cmd_ctrl.dst_addr.u.short_addr = 0x0000;
  cmd.cmd_ctrl.src_ep = this->endpoint_;
  cmd.cmd_ctrl.dst_ep = this->destination_endpoint_;
  cmd.cmd_ctrl.cluster_id = this->diagnostic_cluster_id_;
  cmd.cmd_ctrl.cnf_ctx.cb = &ZigbeeBatchComponent::tx_confirm_;
  cmd.cmd_ctrl.cnf_ctx.user_ctx = this;
  cmd.cmd_id = this->command_id_;
  cmd.data_length = static_cast<uint16_t>(this->diagnostic_payload_.size());
  cmd.data = this->diagnostic_payload_.data();

  this->transmitted_error_code_.store(code, std::memory_order_release);
  this->transmitted_error_sequence_.store(sequence,
                                           std::memory_order_release);
  const ezb_err_t err = ezb_zcl_custom_cluster_cmd_req(&cmd);
  if (err == EZB_ERR_NONE)
    return true;

  this->transmitted_error_code_.store(0, std::memory_order_release);
  this->transmitted_error_sequence_.store(0, std::memory_order_release);
  this->queue_callback_error_(ERROR_DIAGNOSTIC_QUEUE_FAILED);
  return false;
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

  // A saved diagnostic is appended only after the regular batch succeeded.
  // This runs inside the Zigbee callback, so no additional application-side
  // wait or lock acquisition is needed. Keep TX pending until both commands
  // have completed.
  if (cnf != nullptr && cnf->status == 0 &&
      self->transmitted_error_code_.load(std::memory_order_acquire) == 0) {
    const uint16_t diagnostic = self->first_error_code();
    const uint32_t sequence = self->error_sequence();
    if (diagnostic != 0 &&
        self->queue_diagnostic_(diagnostic, sequence))
      return;
  }

  if (cnf == nullptr) {
    self->queue_callback_error_(ERROR_CONFIRM_NULL);
  } else if (cnf->status != 0) {
    self->queue_callback_error_(static_cast<uint16_t>(
        ERROR_CONFIRM_STATUS_BASE + static_cast<uint8_t>(cnf->status)));
  }

  // Release last. is_idle() uses acquire semantics, so observing idle=true
  // also observes the status and latency written above.
  self->tx_pending_.store(0, std::memory_order_release);
  App.wake_loop_threadsafe();
}


}  // namespace zigbee_batch
}  // namespace esphome
