#include "test_support.h"
#include "../../components/zigbee_batch/zigbee_batch.cpp"
#include <cassert>
#include <iostream>
#include <limits>

using namespace esphome;
using namespace esphome::zigbee_batch;
struct Batch : ZigbeeBatchComponent {
  using ZigbeeBatchComponent::TX_ACTIVE;
  using ZigbeeBatchComponent::TX_CALLBACK;
  using ZigbeeBatchComponent::TX_ABANDONED;
  using ZigbeeBatchComponent::app_signal_handler_;
  using ZigbeeBatchComponent::button_interrupt_;
  using ZigbeeBatchComponent::current_context_;
  using ZigbeeBatchComponent::builder_values_;
  using ZigbeeBatchComponent::builder_changed_mask_;
  using ZigbeeBatchComponent::rejoin_requested_;
  using ZigbeeBatchComponent::service_pending_rejoin_;
};
zigbee::ZigbeeComponent stack;
void reset() {
  App = {};
  mock::now_us = 0; mock::clock_hook = nullptr; mock::lock_hook = nullptr;
  mock::notified = false; mock::lock_available = true; mock::joined = true;
  mock::gpio_level = 1; mock::irq_enabled = true;
  mock::leave_count = 0; mock::commissioning_count = 0; mock::queue_error = 0;
  mock::commands.clear(); mock::sleeps.clear();
}
void init(Batch &b) { b.set_zigbee(&stack); b.set_wakeup_pin(9); b.setup(); }
void confirm(size_t index, int status = 0) {
  auto c = mock::commands.at(index);
  ezb_zcl_cmd_cnf_t cnf{status};
  c.cmd_ctrl.cnf_ctx.cb(&cnf, c.cmd_ctrl.cnf_ctx.user_ctx);
}
void tick(Batch &b) { App.scheduler.call(); b.loop(); }
void parent_failure() {
  ezb_nwk_signal_network_status_params_t params{9};
  ezb_app_signal_t signal{EZB_NWK_SIGNAL_NETWORK_STATUS, &params};
  Batch::app_signal_handler_(&signal);
}
int main() {
  {
    reset(); Batch b; init(b);
    b.set_data_field_count(6);
    b.set_data_type(0, DataType::UINT32); b.set_data_type(1, DataType::INT32);
    b.set_data_type(2, DataType::UINT32); b.set_data_type(3, DataType::INT32);
    b.set_data_type(4, DataType::UINT8); b.set_data_type(5, DataType::INT16);
    b.begin(); b.add(uint32_t{16777217}); b.add(int32_t{-16777217});
    b.add(UINT32_MAX); b.add(INT32_MIN); b.add(uint32_t{1000}); b.add(-123.5f);
    assert(b.builder_values_[0] == 16777217);
    assert(b.builder_values_[1] == uint32_t(-16777217));
    assert(b.builder_values_[2] == UINT32_MAX && b.builder_values_[3] == uint32_t(INT32_MIN));
    assert(b.builder_values_[4] == 255 && b.builder_values_[5] == uint16_t(-124));
    assert(b.send()); confirm(0);
    b.begin(); b.add(uint32_t{16777217}); assert(b.builder_changed_mask_ == 0);
    b.begin(); b.add(std::numeric_limits<float>::quiet_NaN()); assert(b.builder_changed_mask_ == 0);
  }
  {
    reset(); Batch b; init(b); b.record_error(60019); b.begin(); assert(b.send());
    auto *data = b.current_context_.load();
    // Timeout observes a context owned by the callback: it must retain it.
    mock::now_us = 3000000; data->state.store(Batch::TX_CALLBACK);
    b.handle_confirm_timeout(); assert(b.current_context_.load() == data && !b.is_idle());
    data->state.store(Batch::TX_ACTIVE); confirm(0);
    auto *diagnostic = b.current_context_.load(); assert(diagnostic != data);
    b.handle_confirm_timeout(); assert(!b.is_idle());
    mock::now_us = 6000000; b.handle_confirm_timeout(); assert(b.is_idle());
    b.begin(); assert(b.send()); auto *next = b.current_context_.load();
    confirm(1); // Late diagnostic callback cannot complete the next packet.
    assert(!b.is_idle() && b.current_context_.load() == next);
    confirm(2); assert(mock::commands.size() == 4); confirm(3); b.finish_tx();
    assert(b.is_idle() && b.first_error_code() == 0);
  }
  {
    reset(); Batch b; init(b); b.record_error(60019); b.begin(); assert(b.send());
    mock::now_us = 3000000;
    // The callback starts AFTER timeout loaded current_context_, before CAS.
    mock::clock_hook = [] { confirm(0); };
    b.handle_confirm_timeout();
    assert(!b.is_idle() && b.current_context_.load()->diagnostic);
    mock::now_us = 6000000; b.handle_confirm_timeout(); assert(b.is_idle());
  }
  {
    reset(); Batch b; init(b); b.record_error(60019); b.begin(); assert(b.send());
    b.start_tx_wait();
    mock::now_us = 2500000; confirm(0);
    mock::now_us = 3000000; b.handle_confirm_timeout(); assert(!b.is_idle());
    mock::now_us = 4500000; confirm(1); b.finish_tx(); b.stop_sleep();
    assert(b.first_error_code() == 0 && b.is_idle() && App.interval == 16);
  }
  {
    reset(); Batch b; init(b); b.begin(); assert(b.send()); b.start_tx_wait();
    for (int i = 0; i < 8 && !b.is_idle(); ++i) tick(b);
    assert(b.is_idle() && mock::now_us == 3000000 && mock::sleeps.size() == 1);
  }
  {
    reset(); Batch b; init(b); b.start_sleep(600000000);
    tick(b); tick(b); assert(mock::sleeps == std::vector<uint32_t>{600000});
    tick(b); tick(b); assert(b.sleep_finished() && App.interval == 16);
  }
  {
    reset(); Batch b; init(b); b.start_sleep(600000000);
    // A pulse already HIGH when loop resumes still rearms the wake interrupt.
    Batch::button_interrupt_(&b); assert(!mock::irq_enabled);
    for (int i = 0; i < 8 && !mock::irq_enabled; ++i) tick(b);
    assert(mock::irq_enabled && mock::irq_level == GPIO_INTR_LOW_LEVEL);
    assert(!b.sleep_finished() && mock::now_us == 20000);
    mock::gpio_level = 0; Batch::button_interrupt_(&b);
    for (int i = 0; i < 8 && !mock::irq_enabled; ++i) tick(b);
    assert(mock::irq_enabled && mock::irq_level == GPIO_INTR_HIGH_LEVEL);
    assert(!b.sleep_finished()); b.stop_sleep();
  }
  {
    reset(); Batch b; init(b); b.start_sleep(600000000);
    bool ran = false;
    b.set_timeout("zb_init", 1000, [&] { ran = true; });
    App.wake_loop_threadsafe();
    for (int i = 0; i < 8 && !ran; ++i) tick(b);
    assert(ran && mock::now_us == 1000000 && !b.sleep_finished());
    b.stop_sleep();
  }
  {
    reset(); Batch b; init(b); parent_failure(); b.loop();
    assert(mock::leave_count == 0 && mock::commissioning_count == 0);
    reset(); Batch c; init(c); c.rejoin_requested_.store(true);
    mock::lock_hook = parent_failure; c.service_pending_rejoin_();
    assert(mock::leave_count == 0);
  }
  {
    reset(); Batch b; init(b); b.start_sleep(600000000);
    mock::lock_available = false; b.rejoin_requested_.store(true); tick(b);
    mock::lock_available = true;
    for (int i = 0; i < 8 && !mock::leave_count; ++i) tick(b);
    assert(mock::leave_count == 1 && mock::now_us == 100000);
    b.stop_sleep();
  }
  {
    reset(); Batch b; init(b); b.record_error(60019); b.begin(); assert(b.send());
    mock::queue_error = 1; confirm(0); b.finish_tx();
    assert(b.is_idle() && b.current_context_.load() == nullptr && b.first_error_code() == 60019);
    mock::queue_error = 0; b.begin(); assert(b.send()); confirm(1); confirm(2);
    b.finish_tx(); assert(b.is_idle() && b.first_error_code() == 0);
  }
  {
    reset(); Batch b; init(b);
    mock::now_us = (int64_t(UINT32_MAX) - 1000) * 1000;
    b.begin(); assert(b.send());
    mock::now_us += 2999000; b.handle_confirm_timeout(); assert(!b.is_idle());
    mock::now_us += 1000; b.handle_confirm_timeout(); assert(b.is_idle());
  }
  std::cout << "PASS: 12 C++ regression scenarios (actual component, mocked platform)\n";
}
