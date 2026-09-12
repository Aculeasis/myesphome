#include "zigbee_batch.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "esphome/core/application.h"
#include "esphome/core/wake.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

namespace esphome {
namespace zigbee_batch {

static constexpr uint32_t LEGACY_ERROR_PREFERENCE_KEY = 0x6E7A4245U;
static constexpr uint32_t RESTART_COOLDOWN_PREFERENCE_KEY = 0x6E7A4246U;
static constexpr uint32_t DIAGNOSTIC_PREFERENCE_KEY = 0x6E7A4247U;
static constexpr uint32_t CONFIRM_TIMEOUT_MS = 3000;

static constexpr uint16_t ERROR_COMPONENT_NULL = 60001;
static constexpr uint16_t ERROR_STACK_NOT_STARTED = 60002;
static constexpr uint16_t ERROR_NOT_JOINED = 60003;
static constexpr uint16_t ERROR_TX_BUSY = 60004;
static constexpr uint16_t ERROR_LOCK_FAILED = 60005;
static constexpr uint16_t ERROR_QUEUE_FAILED = 60006;
static constexpr uint16_t ERROR_CONFIRM_TIMEOUT = 60007;
static constexpr uint16_t ERROR_CONFIRM_NULL = 60008;
static constexpr uint16_t ERROR_PARENT_LINK_FAILURE = 60011;
static constexpr uint16_t ERROR_DEVICE_REBOOT_FAILED = 60012;
static constexpr uint16_t ERROR_CONFIG_LOCK_FAILED = 60013;
static constexpr uint16_t ERROR_DIAGNOSTIC_QUEUE_FAILED = 60014;
static constexpr uint16_t ERROR_PAYLOAD_INCOMPLETE = 60015;
static constexpr uint16_t ERROR_TX_CONTEXT_EXHAUSTED = 60016;
static constexpr uint16_t ERROR_REJOIN_FAILED = 60017;
static constexpr uint16_t ERROR_PM_CONFIG_FAILED = 60018;
static constexpr uint16_t ERROR_BUTTON_IRQ_FAILED = 60019;
static constexpr uint16_t ERROR_CONFIRM_STATUS_BASE = 61000;

static constexpr uint8_t NWK_TARGET_DEVICE_UNAVAILABLE = 0x07;
static constexpr uint8_t NWK_TARGET_ADDRESS_UNALLOCATED = 0x08;
static constexpr uint8_t NWK_PARENT_LINK_FAILURE = 0x09;

ZigbeeBatchComponent *ZigbeeBatchComponent::instance_ = nullptr;

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
      this->first_error_code_.store(legacy_error, std::memory_order_release);
      this->error_sequence_.store(1, std::memory_order_release);
      this->save_diagnostic_state_(legacy_error, 1);
    }
  }

  bool restart_cooldown = false;
  if (this->restart_cooldown_preference_.load(&restart_cooldown))
    this->restart_cooldown_.store(restart_cooldown,
                                  std::memory_order_release);

#if CONFIG_PM_ENABLE && CONFIG_FREERTOS_USE_TICKLESS_IDLE
  esp_pm_config_t pm_config = {
      .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
      .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
      .light_sleep_enable = true,
  };
  if (esp_pm_configure(&pm_config) != ESP_OK)
    this->record_error(ERROR_PM_CONFIG_FAILED);
#endif

  // Tickless idle lets the Zigbee task wake when the regular keepalive is due.
  // Keep it aligned with the normal report period to avoid extra radio wakes.
  // ESPHome already configures the standard 64-minute ED timeout.
  if (esp_zigbee_lock_acquire(pdMS_TO_TICKS(50))) {
    ezb_nwk_set_keepalive_interval(600000U);
    ezb_nwk_set_rx_on_when_idle(false);
    ezb_nwk_set_fast_poll_interval(30U);
    esp_zigbee_lock_release();
  } else {
    this->record_error(ERROR_CONFIG_LOCK_FAILED);
  }

  for (auto &context : this->tx_contexts_)
    context.owner = this;

  if (this->wakeup_pin_ != 0xFF) {
    const esp_err_t service_error =
        gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    if (service_error != ESP_OK && service_error != ESP_ERR_INVALID_STATE) {
      this->record_error(ERROR_BUTTON_IRQ_FAILED);
    } else if (gpio_isr_handler_add(
                   static_cast<gpio_num_t>(this->wakeup_pin_),
                   &ZigbeeBatchComponent::button_interrupt_, this) != ESP_OK) {
      this->record_error(ERROR_BUTTON_IRQ_FAILED);
    } else {
      this->button_handler_ready_ = true;
      if (esp_sleep_enable_gpio_wakeup() != ESP_OK)
        this->record_error(ERROR_BUTTON_IRQ_FAILED);
      else
        this->arm_button_interrupt_();
    }
  }

  instance_ = this;
  ezb_app_signal_add_handler(ZigbeeBatchComponent::app_signal_handler_);
}

void ZigbeeBatchComponent::loop() {
  if (this->button_interrupt_pending_.exchange(false,
                                                std::memory_order_acq_rel))
    this->process_button_interrupt_();

  this->service_pending_rejoin_();
  this->sleep_step_();
}

void ZigbeeBatchComponent::service_pending_rejoin_() {
  if (this->rejoin_retry_scheduled_ ||
      this->rejoin_in_progress_.load(std::memory_order_acquire) ||
      !this->rejoin_requested_.load(std::memory_order_acquire))
    return;

  if (!esp_zigbee_lock_acquire(pdMS_TO_TICKS(50))) {
    this->rejoin_retry_scheduled_ = true;
    this->set_timeout("rejoin-retry", 100, [this]() {
      this->rejoin_retry_scheduled_ = false;
      this->enable_loop_soon_any_context();
    });
    return;
  }

  // A parent failure can arrive while the main task is acquiring the lock.
  // Its native handler already owns recovery; do not issue a second request.
  if (this->rejoin_in_progress_.load(std::memory_order_acquire) ||
      !this->rejoin_requested_.exchange(false, std::memory_order_acq_rel)) {
    this->rejoin_requested_.store(false, std::memory_order_release);
    esp_zigbee_lock_release();
    return;
  }

  // Mark this before starting commissioning: the completion signal may be
  // delivered from the Zigbee task immediately after the request is queued.
  this->rejoin_in_progress_.store(true, std::memory_order_release);
  ezb_err_t error;
  if (ezb_bdb_dev_joined()) {
    ezb_zdo_nwk_mgmt_leave_req_t request = {
        .dst_nwk_addr = ezb_nwk_get_short_address(),
        .field = {.remove_children = false, .rejoin = true},
    };
    error = ezb_zdo_nwk_mgmt_leave_req(&request);
  } else {
    error = ezb_bdb_start_top_level_commissioning(
        EZB_BDB_MODE_INITIALIZATION);
  }
  esp_zigbee_lock_release();
  if (error != EZB_ERR_NONE) {
    this->rejoin_in_progress_.store(false, std::memory_order_release);
    this->note_noncritical_error_(ERROR_REJOIN_FAILED, false);
  }
}

void ZigbeeBatchComponent::start_sleep(int64_t deadline_us) {
  if (!this->sleeping_)
    this->saved_loop_interval_ = App.get_loop_interval();
  this->sleeping_ = true;
  this->waiting_for_tx_ = false;
  this->sleep_deadline_us_ = deadline_us;
  this->sleep_yield_pending_ = true;
  // Our event/deadline wait replaces the normal 16ms loop pacing. In
  // particular, do not add a second timed wake after every real event.
  App.set_loop_interval(0);
}

void ZigbeeBatchComponent::stop_sleep() {
  if (!this->sleeping_)
    return;
  this->sleeping_ = false;
  this->waiting_for_tx_ = false;
  App.set_loop_interval(this->saved_loop_interval_);
}

bool ZigbeeBatchComponent::sleep_finished() const {
  return this->restart_required() ||
         (esp_timer_get_time() >= this->sleep_deadline_us_ &&
          !this->rejoin_in_progress_.load(std::memory_order_acquire) &&
          !this->rejoin_requested_.load(std::memory_order_acquire));
}

void ZigbeeBatchComponent::start_tx_wait() {
  this->start_sleep(0);
  this->waiting_for_tx_ = true;
}

void ZigbeeBatchComponent::sleep_step_() {
  if (!this->sleeping_)
    return;
  if (this->sleep_yield_pending_) {
    // Allow a complete scheduler/component pass after a wake, independent of
    // component order. This also lets the button sensor publish both edges.
    this->sleep_yield_pending_ = false;
    return;
  }

  uint32_t wait_ms;
  if (this->waiting_for_tx_) {
    if (this->is_idle()) {
      this->stop_sleep();
      return;
    }
    this->handle_confirm_timeout();
    if (this->is_idle()) {
      this->stop_sleep();
      return;
    }
    const auto *context = this->current_context_.load(std::memory_order_acquire);
    // A callback owns a context only briefly and always wakes us on completion
    // or when queuing diagnostics. Each packet has its own three-second budget.
    wait_ms = CONFIRM_TIMEOUT_MS;
    if (context != nullptr) {
      const uint32_t elapsed = now_ms_32() - context->started_ms;
      if (elapsed < CONFIRM_TIMEOUT_MS)
        wait_ms -= elapsed;
    }
  } else {
    if (this->sleep_finished()) {
      this->stop_sleep();
      return;
    }
    const int64_t remaining_us = this->sleep_deadline_us_ - esp_timer_get_time();
    wait_ms = remaining_us > 0
                  ? static_cast<uint32_t>(std::min<int64_t>(
                        (remaining_us + 999LL) / 1000LL, UINT32_MAX))
                  : 600000U;
  }

  // Include newly queued timers, including Zigbee's zb_init and GPIO debounce.
  // Sleep to their actual deadline, never poll them at a fixed interval.
  App.scheduler.process_to_add();
  const auto scheduled = App.scheduler.next_schedule_in(now_ms_32());
  if (scheduled.has_value())
    wait_ms = std::min(wait_ms, *scheduled);
  this->sleep_yield_pending_ = true;
  if (wait_ms != 0)
    esphome::internal::wakeable_delay(wait_ms);
}

void ZigbeeBatchComponent::wait_for_duration(uint32_t duration_ms) {
  const int64_t deadline_us =
      esp_timer_get_time() + static_cast<int64_t>(duration_ms) * 1000LL;
  while (true) {
    const int64_t remaining_us = deadline_us - esp_timer_get_time();
    if (remaining_us <= 0)
      return;
    const uint32_t remaining_ms = static_cast<uint32_t>(
        (remaining_us + 999LL) / 1000LL);
    esphome::internal::wakeable_delay(remaining_ms);
  }
}

void IRAM_ATTR ZigbeeBatchComponent::button_interrupt_(void *user_context) {
  auto *self = static_cast<ZigbeeBatchComponent *>(user_context);
  if (self == nullptr)
    return;

  // A GPIO wake interrupt is level-triggered. Disable it on its first entry so
  // a held button cannot continuously retrigger the ISR.
  gpio_intr_disable(static_cast<gpio_num_t>(self->wakeup_pin_));
  self->button_interrupt_pending_.store(true, std::memory_order_release);
  self->enable_loop_soon_any_context();
}

void ZigbeeBatchComponent::process_button_interrupt_() {
  if (!this->button_handler_ready_ || this->button_wakeup_suspended_)
    return;

  const auto pin = static_cast<gpio_num_t>(this->wakeup_pin_);
  gpio_wakeup_disable(pin);
  this->set_timeout("button-debounce", 20, [this]() {
    if (!this->button_wakeup_suspended_) {
      this->arm_button_interrupt_();
      this->enable_loop_soon_any_context();
    }
  });
}

void ZigbeeBatchComponent::arm_button_interrupt_() {
  if (!this->button_handler_ready_ || this->button_wakeup_suspended_)
    return;

  const auto pin = static_cast<gpio_num_t>(this->wakeup_pin_);
  const gpio_int_type_t next_level =
      gpio_get_level(pin) == 0 ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL;
  if (gpio_wakeup_enable(pin, next_level) != ESP_OK) {
    this->record_error(ERROR_BUTTON_IRQ_FAILED);
    return;
  }
  gpio_intr_enable(pin);
}

void ZigbeeBatchComponent::suspend_button_wakeup() {
  if (!this->button_handler_ready_)
    return;
  this->button_wakeup_suspended_ = true;
  this->cancel_timeout("button-debounce");
  const auto pin = static_cast<gpio_num_t>(this->wakeup_pin_);
  gpio_intr_disable(pin);
  gpio_wakeup_disable(pin);
}

void ZigbeeBatchComponent::resume_button_wakeup() {
  if (!this->button_handler_ready_)
    return;
  this->button_interrupt_pending_.store(false, std::memory_order_release);
  this->button_wakeup_suspended_ = false;
  this->arm_button_interrupt_();
  this->enable_loop_soon_any_context();
}

void ZigbeeBatchComponent::begin() {
  this->builder_index_ = 0;
  this->builder_changed_mask_ = 0;
  this->builder_values_.fill(0);
}

size_t ZigbeeBatchComponent::data_type_size_(DataType type) {
  switch (type) {
    case DataType::UINT8:
    case DataType::INT8:
      return 1;
    case DataType::UINT16:
    case DataType::INT16:
      return 2;
    default:
      return 4;
  }
}

uint32_t ZigbeeBatchComponent::encode_value_(DataType type, double value) {
  const double rounded = std::round(value);
  switch (type) {
    case DataType::UINT8:
      return static_cast<uint8_t>(std::clamp(rounded, 0.0, 255.0));
    case DataType::UINT16:
      return static_cast<uint16_t>(std::clamp(rounded, 0.0, 65535.0));
    case DataType::UINT32:
      return static_cast<uint32_t>(std::clamp(
          rounded, 0.0,
          static_cast<double>(std::numeric_limits<uint32_t>::max())));
    case DataType::INT8:
      return static_cast<uint8_t>(static_cast<int8_t>(
          std::clamp(rounded, -128.0, 127.0)));
    case DataType::INT16:
      return static_cast<uint16_t>(static_cast<int16_t>(
          std::clamp(rounded, -32768.0, 32767.0)));
    case DataType::INT32:
      return static_cast<uint32_t>(static_cast<int32_t>(std::clamp(
          rounded,
          static_cast<double>(std::numeric_limits<int32_t>::min()),
          static_cast<double>(std::numeric_limits<int32_t>::max()))));
  }
  return 0;
}

void ZigbeeBatchComponent::add(float value) {
  this->add_value_(value);
}

void ZigbeeBatchComponent::add(double value) {
  this->add_value_(value);
}

void ZigbeeBatchComponent::add(uint32_t value) {
  this->add_value_(value);
}

void ZigbeeBatchComponent::add(int32_t value) {
  this->add_value_(value);
}

void ZigbeeBatchComponent::add_value_(double value) {
  if (this->builder_index_ >= this->data_field_count_) {
    this->record_error(ERROR_PAYLOAD_INCOMPLETE);
    return;
  }
  if (!std::isfinite(value)) {
    this->skip();
    return;
  }

  const size_t index = this->builder_index_++;
  const uint32_t encoded = this->encode_value_(this->data_types_[index], value);
  this->builder_values_[index] = encoded;
  const uint32_t bit = 1UL << index;
  if ((this->sent_value_valid_mask_ & bit) == 0 ||
      this->sent_values_[index] != encoded)
    this->builder_changed_mask_ |= bit;
}

void ZigbeeBatchComponent::skip() {
  if (this->builder_index_ >= this->data_field_count_) {
    this->record_error(ERROR_PAYLOAD_INCOMPLETE);
    return;
  }
  this->builder_index_++;
}

void ZigbeeBatchComponent::put_value_(uint8_t *dst, uint32_t value,
                                      size_t size) {
  for (size_t i = 0; i < size; i++)
    dst[i] = static_cast<uint8_t>((value >> (8U * i)) & 0xFFU);
}

ZigbeeBatchComponent::TxContext *
ZigbeeBatchComponent::acquire_tx_context_() {
  for (auto &context : this->tx_contexts_) {
    uint8_t expected = TX_FREE;
    if (context.state.compare_exchange_strong(
            expected, TX_CALLBACK, std::memory_order_acq_rel,
            std::memory_order_acquire))
      return &context;
  }
  return nullptr;
}

bool ZigbeeBatchComponent::send() {
  this->commit_callback_error_();
  // Unspecified trailing fields are equivalent to skip(). This makes an empty
  // batch useful: begin(); send(); emits only zeroed presence bitmap byte(s).
  this->builder_index_ = this->data_field_count_;
  if (this->zb_ == nullptr) {
    this->record_error(ERROR_COMPONENT_NULL);
    return false;
  }
  if (!this->zb_->is_started()) {
    this->record_error(ERROR_STACK_NOT_STARTED);
    return false;
  }
  if (!this->zb_->is_joined()) {
    this->note_noncritical_error_(ERROR_NOT_JOINED, false);
    this->rejoin_requested_.store(true, std::memory_order_release);
    this->enable_loop_soon_any_context();
    return false;
  }

  uint8_t expected = 0;
  if (!this->tx_pending_.compare_exchange_strong(
          expected, 1, std::memory_order_acq_rel,
          std::memory_order_acquire)) {
    this->record_error(ERROR_TX_BUSY);
    return false;
  }

  TxContext *context = this->acquire_tx_context_();
  if (context == nullptr) {
    this->record_error(ERROR_TX_CONTEXT_EXHAUSTED);
    this->restart_required_.store(true, std::memory_order_release);
    this->tx_pending_.store(0, std::memory_order_release);
    return false;
  }

  context->diagnostic = false;
  context->started_ms = now_ms_32();
  context->changed_mask = this->builder_changed_mask_;
  context->values = this->builder_values_;
  context->payload.fill(0);

  const size_t flag_bytes =
      std::max<size_t>(1, (this->data_field_count_ + 6) / 7);
  for (size_t flag_index = 0; flag_index < flag_bytes; flag_index++) {
    uint8_t flags = flag_index + 1 < flag_bytes ? 0x80U : 0;
    for (size_t bit_index = 0; bit_index < 7; bit_index++) {
      const size_t field_index = flag_index * 7 + bit_index;
      if (field_index < this->data_field_count_ &&
          (context->changed_mask & (1UL << field_index)) != 0)
        flags |= 1U << bit_index;
    }
    context->payload[flag_index] = flags;
  }

  size_t offset = flag_bytes;
  for (size_t index = 0; index < this->data_field_count_; index++) {
    if ((context->changed_mask & (1UL << index)) == 0)
      continue;
    const size_t size = this->data_type_size_(this->data_types_[index]);
    this->put_value_(&context->payload[offset], context->values[index], size);
    offset += size;
  }
  context->payload_size = static_cast<uint16_t>(offset);

  this->transmitted_error_code_.store(0, std::memory_order_release);
  this->transmitted_error_sequence_.store(0, std::memory_order_release);
  this->last_tx_status_.store(0xFF, std::memory_order_relaxed);
  this->tx_started_ms_.store(now_ms_32(), std::memory_order_relaxed);
  this->current_context_.store(context, std::memory_order_release);
  context->state.store(TX_ACTIVE, std::memory_order_release);

  if (!this->queue_context_(context)) {
    context->state.store(TX_FREE, std::memory_order_release);
    this->current_context_.store(nullptr, std::memory_order_release);
    this->tx_pending_.store(0, std::memory_order_release);
    return false;
  }
  return true;
}

bool ZigbeeBatchComponent::queue_context_(TxContext *context,
                                          bool acquire_lock) {
  if (acquire_lock && !esp_zigbee_lock_acquire(pdMS_TO_TICKS(50))) {
    this->record_error(ERROR_LOCK_FAILED);
    return false;
  }

  ezb_zcl_custom_cluster_cmd_t command = {};
  command.cmd_ctrl.fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI;
  command.cmd_ctrl.fc.dis_default_rsp = 1;
  command.cmd_ctrl.dst_addr.addr_mode = EZB_ADDR_MODE_SHORT;
  command.cmd_ctrl.dst_addr.u.short_addr = 0x0000;
  command.cmd_ctrl.src_ep = this->endpoint_;
  command.cmd_ctrl.dst_ep = this->destination_endpoint_;
  command.cmd_ctrl.cluster_id = context->diagnostic
                                    ? this->diagnostic_cluster_id_
                                    : this->cluster_id_;
  command.cmd_ctrl.cnf_ctx.cb = &ZigbeeBatchComponent::tx_confirm_;
  command.cmd_ctrl.cnf_ctx.user_ctx = context;
  command.cmd_id = this->command_id_;
  command.data_length = context->payload_size;
  command.data = context->payload.data();

  const ezb_err_t error = ezb_zcl_custom_cluster_cmd_req(&command);
  if (acquire_lock)
    esp_zigbee_lock_release();
  if (error == EZB_ERR_NONE)
    return true;

  const uint16_t code = context->diagnostic
                            ? ERROR_DIAGNOSTIC_QUEUE_FAILED
                            : ERROR_QUEUE_FAILED;
  if (acquire_lock)
    this->record_error(code);
  else
    this->queue_callback_error_(code);
  return false;
}

bool ZigbeeBatchComponent::queue_diagnostic_(TxContext *context,
                                             uint16_t code,
                                             uint32_t sequence) {
  // Use a distinct context: a timeout inspecting the data packet must never
  // abandon the diagnostic packet that its callback has just queued.
  TxContext *data_context = context;
  context = this->acquire_tx_context_();
  if (context == nullptr) {
    this->queue_callback_error_(ERROR_TX_CONTEXT_EXHAUSTED);
    this->restart_required_.store(true, std::memory_order_release);
    return false;
  }
  context->diagnostic = true;
  context->started_ms = now_ms_32();
  context->payload[0] = DIAGNOSTIC_PAYLOAD_VERSION;
  this->put_value_(&context->payload[1], code, 2);
  this->put_value_(&context->payload[3], sequence, 4);
  context->payload_size = DIAGNOSTIC_PAYLOAD_SIZE;
  this->transmitted_error_code_.store(code, std::memory_order_release);
  this->transmitted_error_sequence_.store(sequence,
                                           std::memory_order_release);
  context->state.store(TX_ACTIVE, std::memory_order_release);
  this->current_context_.store(context, std::memory_order_release);
  // The confirm callback already runs in Zigbee stack context.
  if (this->queue_context_(context, false)) {
    data_context->state.store(TX_FREE, std::memory_order_release);
    App.wake_loop_threadsafe();
    return true;
  }

  this->current_context_.store(data_context, std::memory_order_release);
  context->state.store(TX_FREE, std::memory_order_release);
  this->transmitted_error_code_.store(0, std::memory_order_release);
  this->transmitted_error_sequence_.store(0, std::memory_order_release);
  return false;
}

void ZigbeeBatchComponent::commit_sent_values_(const TxContext *context) {
  for (size_t index = 0; index < this->data_field_count_; index++) {
    const uint32_t bit = 1UL << index;
    if ((context->changed_mask & bit) == 0)
      continue;
    this->sent_values_[index] = context->values[index];
    this->sent_value_valid_mask_ |= bit;
  }
}

void ZigbeeBatchComponent::finish_context_(TxContext *context) {
  TxContext *expected = context;
  this->current_context_.compare_exchange_strong(
      expected, nullptr, std::memory_order_acq_rel,
      std::memory_order_acquire);
  context->state.store(TX_FREE, std::memory_order_release);
  this->tx_pending_.store(0, std::memory_order_release);
  App.wake_loop_threadsafe();
}

void ZigbeeBatchComponent::tx_confirm_(ezb_zcl_cmd_cnf_t *confirmation,
                                       void *user_context) {
  auto *context = static_cast<TxContext *>(user_context);
  if (context == nullptr || context->owner == nullptr)
    return;
  auto *self = context->owner;

  uint8_t expected = TX_ACTIVE;
  if (!context->state.compare_exchange_strong(
          expected, TX_CALLBACK, std::memory_order_acq_rel,
          std::memory_order_acquire)) {
    if (expected == TX_ABANDONED)
      context->state.store(TX_FREE, std::memory_order_release);
    return;
  }

  self->last_confirm_latency_ms_.store(
      now_ms_32() - self->tx_started_ms_.load(std::memory_order_relaxed),
      std::memory_order_relaxed);
  const uint8_t status = confirmation == nullptr
                             ? 0xFF
                             : static_cast<uint8_t>(confirmation->status);
  self->last_tx_status_.store(status, std::memory_order_relaxed);

  if (!context->diagnostic && status == 0) {
    self->commit_sent_values_(context);
    self->reset_noncritical_errors_();
    const uint16_t diagnostic = self->first_error_code();
    const uint32_t sequence = self->error_sequence();
    if (diagnostic != 0 &&
        self->queue_diagnostic_(context, diagnostic, sequence))
      return;
  }

  if (confirmation == nullptr) {
    self->queue_callback_error_(ERROR_CONFIRM_NULL);
  } else if (status != 0) {
    const uint16_t code = ERROR_CONFIRM_STATUS_BASE + status;
    if (status == NWK_TARGET_DEVICE_UNAVAILABLE ||
        status == NWK_TARGET_ADDRESS_UNALLOCATED ||
        status == NWK_PARENT_LINK_FAILURE) {
      self->note_noncritical_error_(code, true);
      if (status == NWK_TARGET_ADDRESS_UNALLOCATED)
        self->rejoin_requested_.store(true, std::memory_order_release);
    } else {
      self->queue_callback_error_(code);
    }
  }

  self->finish_context_(context);
}

void ZigbeeBatchComponent::handle_confirm_timeout() {
  TxContext *context =
      this->current_context_.load(std::memory_order_acquire);
  if (context == nullptr ||
      now_ms_32() - context->started_ms < CONFIRM_TIMEOUT_MS)
    return;

  uint8_t expected = TX_ACTIVE;
  if (!context->state.compare_exchange_strong(
          expected, TX_ABANDONED, std::memory_order_acq_rel,
          std::memory_order_acquire))
    return;

  TxContext *current = context;
  this->current_context_.compare_exchange_strong(
      current, nullptr, std::memory_order_acq_rel,
      std::memory_order_acquire);
  this->transmitted_error_code_.store(0, std::memory_order_release);
  this->transmitted_error_sequence_.store(0, std::memory_order_release);
  this->last_tx_status_.store(0xFF, std::memory_order_relaxed);
  this->tx_pending_.store(0, std::memory_order_release);
  this->note_noncritical_error_(ERROR_CONFIRM_TIMEOUT, false);
}

void ZigbeeBatchComponent::note_noncritical_error_(
    uint16_t code, bool callback_context) {
  if (callback_context)
    this->queue_callback_error_(code);
  else
    this->record_error(code);

  uint8_t current =
      this->consecutive_noncritical_errors_.load(std::memory_order_relaxed);
  while (current < std::numeric_limits<uint8_t>::max() &&
         !this->consecutive_noncritical_errors_.compare_exchange_weak(
             current, static_cast<uint8_t>(current + 1),
             std::memory_order_acq_rel, std::memory_order_relaxed)) {
  }
  const uint8_t count = current == std::numeric_limits<uint8_t>::max()
                            ? current
                            : static_cast<uint8_t>(current + 1);
  if (count >= this->noncritical_error_limit_)
    this->restart_required_.store(true, std::memory_order_release);
}

void ZigbeeBatchComponent::reset_noncritical_errors_() {
  this->consecutive_noncritical_errors_.store(0, std::memory_order_release);
  this->restart_required_.store(false, std::memory_order_release);
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
  this->reset_noncritical_errors_();
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
  this->first_error_code_.store(0, std::memory_order_release);
  this->save_diagnostic_state_(0, this->error_sequence());
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
  this->record_error(
      this->callback_error_code_.exchange(0, std::memory_order_acq_rel));
}

void ZigbeeBatchComponent::finish_tx() {
  const uint16_t transmitted =
      this->transmitted_error_code_.exchange(0, std::memory_order_acq_rel);
  const uint32_t transmitted_sequence =
      this->transmitted_error_sequence_.exchange(
          0, std::memory_order_acq_rel);
  if (this->last_tx_success() && transmitted != 0 &&
      this->first_error_code() == transmitted &&
      this->error_sequence() == transmitted_sequence)
    this->clear_error_();
  this->commit_callback_error_();
}

bool ZigbeeBatchComponent::app_signal_handler_(
    const ezb_app_signal_t *app_signal) {
  auto *self = instance_;
  if (self == nullptr || app_signal == nullptr)
    return true;

  const ezb_app_signal_type_t type = ezb_app_signal_get_type(app_signal);
  if (type == EZB_NWK_SIGNAL_NETWORK_STATUS) {
    const auto *params =
        static_cast<const ezb_nwk_signal_network_status_params_t *>(
            ezb_app_signal_get_params(app_signal));
    if (params != nullptr &&
        params->status == EZB_NWK_NETWORK_STATUS_PARENT_LINK_FAILURE) {
      self->note_noncritical_error_(ERROR_PARENT_LINK_FAILURE, true);
      // ESPHome's Zigbee handler already issues leave(rejoin=true).
      self->rejoin_requested_.store(false, std::memory_order_release);
      self->rejoin_in_progress_.store(true, std::memory_order_release);
      App.wake_loop_threadsafe();
    }
  } else if (type == EZB_BDB_SIGNAL_DEVICE_REBOOT) {
    const auto *status = static_cast<const ezb_bdb_comm_status_t *>(
        ezb_app_signal_get_params(app_signal));
    if (self->rejoin_in_progress_.exchange(false,
                                            std::memory_order_acq_rel)) {
      if (status == nullptr || *status != EZB_BDB_STATUS_SUCCESS)
        self->note_noncritical_error_(ERROR_REJOIN_FAILED, true);
      App.wake_loop_threadsafe();
    } else if (status != nullptr && *status != EZB_BDB_STATUS_SUCCESS) {
      self->queue_callback_error_(ERROR_DEVICE_REBOOT_FAILED);
    }
  }
  return true;
}

}  // namespace zigbee_batch
}  // namespace esphome
