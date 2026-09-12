#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#define IRAM_ATTR
#define ESP_INTR_FLAG_LEVEL3 3
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 1
#define pdMS_TO_TICKS(x) (x)
using esp_err_t = int;
using ezb_err_t = int;
using gpio_num_t = int;
using gpio_int_type_t = int;
constexpr int GPIO_INTR_HIGH_LEVEL = 1, GPIO_INTR_LOW_LEVEL = 0;
constexpr int EZB_ERR_NONE = 0, EZB_BDB_MODE_INITIALIZATION = 1;
constexpr int EZB_ZCL_CMD_DIRECTION_TO_CLI = 1, EZB_ADDR_MODE_SHORT = 2;
constexpr int EZB_NWK_SIGNAL_NETWORK_STATUS = 1, EZB_BDB_SIGNAL_DEVICE_REBOOT = 2;
constexpr int EZB_NWK_NETWORK_STATUS_PARENT_LINK_FAILURE = 9;
constexpr int EZB_BDB_STATUS_SUCCESS = 0;
using ezb_app_signal_type_t = int;
using ezb_bdb_comm_status_t = int;
struct ezb_app_signal_t { int type; const void *params; };
struct ezb_nwk_signal_network_status_params_t { int status; };
inline int ezb_app_signal_get_type(const ezb_app_signal_t *s) { return s->type; }
inline const void *ezb_app_signal_get_params(const ezb_app_signal_t *s) { return s->params; }
inline void ezb_app_signal_add_handler(bool (*)(const ezb_app_signal_t *)) {}
struct ezb_zcl_cmd_cnf_t { int status; };
struct ezb_zcl_custom_cluster_cmd_t {
  struct {
    struct { int direction; int dis_default_rsp; } fc;
    struct { int addr_mode; struct { int short_addr; } u; } dst_addr;
    int src_ep, dst_ep, cluster_id;
    struct { void (*cb)(ezb_zcl_cmd_cnf_t *, void *); void *user_ctx; } cnf_ctx;
  } cmd_ctrl;
  int cmd_id, data_length;
  uint8_t *data;
};
struct ezb_zdo_nwk_mgmt_leave_req_t {
  int dst_nwk_addr;
  struct { bool remove_children, rejoin; } field;
};

namespace mock {
inline int64_t now_us = 0;
inline std::function<void()> clock_hook;
inline std::function<void()> lock_hook;
inline bool lock_available = true;
inline bool joined = true;
inline int gpio_level = 1;
inline bool irq_enabled = true;
inline int irq_level = 0;
inline bool notified = false;
inline int leave_count = 0, commissioning_count = 0;
inline int queue_error = 0;
inline std::vector<ezb_zcl_custom_cluster_cmd_t> commands;
inline std::vector<uint32_t> sleeps;
}
inline int64_t esp_timer_get_time() {
  if (mock::clock_hook) {
    auto hook = std::move(mock::clock_hook);
    mock::clock_hook = nullptr;
    hook();
  }
  return mock::now_us;
}
inline bool esp_zigbee_lock_acquire(int) {
  if (mock::lock_hook) { auto hook = std::move(mock::lock_hook); mock::lock_hook = nullptr; hook(); }
  return mock::lock_available;
}
inline void esp_zigbee_lock_release() {}
inline bool ezb_bdb_dev_joined() { return mock::joined; }
inline int ezb_nwk_get_short_address() { return 1; }
inline int ezb_zdo_nwk_mgmt_leave_req(ezb_zdo_nwk_mgmt_leave_req_t *) { ++mock::leave_count; return 0; }
inline int ezb_bdb_start_top_level_commissioning(int) { ++mock::commissioning_count; return 0; }
inline int ezb_zcl_custom_cluster_cmd_req(ezb_zcl_custom_cluster_cmd_t *c) {
  if (!mock::queue_error) mock::commands.push_back(*c);
  return mock::queue_error;
}
inline void ezb_nwk_set_keepalive_interval(unsigned) {}
inline void ezb_nwk_set_rx_on_when_idle(bool) {}
inline void ezb_nwk_set_fast_poll_interval(unsigned) {}
inline int gpio_install_isr_service(int) { return 0; }
inline int gpio_isr_handler_add(int, void (*)(void *), void *) { return 0; }
inline int esp_sleep_enable_gpio_wakeup() { return 0; }
inline void gpio_intr_disable(int) { mock::irq_enabled = false; }
inline void gpio_intr_enable(int) { mock::irq_enabled = true; }
inline int gpio_get_level(int) { return mock::gpio_level; }
inline void gpio_wakeup_disable(int) {}
inline int gpio_wakeup_enable(int, int level) { mock::irq_level = level; return 0; }

namespace esphome {
class Component;
struct Scheduler {
  struct Timer { int64_t at; std::function<void()> callback; };
  std::map<std::pair<Component *, std::string>, Timer> timers;
  void process_to_add() {}
  std::optional<uint32_t> next_schedule_in(uint32_t) {
    if (timers.empty()) return {};
    int64_t nearest = INT64_MAX;
    for (auto &t : timers) nearest = std::min(nearest, t.second.at);
    return static_cast<uint32_t>(std::max<int64_t>(0, (nearest - mock::now_us + 999) / 1000));
  }
  void call() {
    while (true) {
      auto it = std::find_if(timers.begin(), timers.end(), [](auto &t) { return t.second.at <= mock::now_us; });
      if (it == timers.end()) return;
      auto callback = std::move(it->second.callback);
      timers.erase(it);
      callback();
    }
  }
};
struct Application {
  Scheduler scheduler;
  uint32_t interval = 16;
  uint32_t get_loop_interval() { return interval; }
  void set_loop_interval(uint32_t value) { interval = value; }
  void wake_loop_threadsafe() { mock::notified = true; }
};
inline Application App;
class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
  void enable_loop_soon_any_context() { App.wake_loop_threadsafe(); }
  void set_timeout(const char *name, uint32_t ms, std::function<void()> callback) {
    App.scheduler.timers[{this, name}] = {mock::now_us + int64_t(ms) * 1000, std::move(callback)};
  }
  void cancel_timeout(const char *name) { App.scheduler.timers.erase({this, name}); }
};
class ESPPreferenceObject {
 public:
  template<typename T> bool load(T *) { return false; }
  template<typename T> bool save(const T *) { return true; }
};
struct Preferences {
  template<typename T> ESPPreferenceObject make_preference(uint32_t) { return {}; }
  void sync() {}
};
inline Preferences preferences;
inline Preferences *global_preferences = &preferences;
namespace zigbee {
class ZigbeeComponent {
 public:
  bool is_started() { return true; }
  bool is_joined() { return mock::joined; }
};
}
namespace internal {
inline void wakeable_delay(uint32_t ms) {
  if (mock::notified) { mock::notified = false; return; }
  mock::sleeps.push_back(ms);
  mock::now_us += int64_t(ms) * 1000;
}
}
}
