"""Compile actual zigbee_batch with platform mocks. Never imports ESPHome."""
from pathlib import Path
import subprocess
import yaml

root = Path(__file__).resolve().parents[2]
output = root / "temp_sources" / "zigbee_batch_tests"
output.mkdir(parents=True, exist_ok=True)
headers = [
    "esphome/components/zigbee/zigbee_esp32.h", "esphome/core/component.h",
    "esphome/core/preferences.h", "esp_zigbee.h", "esp_attr.h",
    "ezbee/zcl/cluster/custom.h", "esphome/core/application.h",
    "esphome/core/wake.h", "driver/gpio.h", "esp_intr_alloc.h", "esp_pm.h",
    "esp_sleep.h", "esp_timer.h", "freertos/FreeRTOS.h",
]
for name in headers:
    p = output / "include" / name
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text('#include "test_support.h"\n')
subprocess.run([
    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-g",
    "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
    "-I", str(output / "include"), "-I", str(root / "tests/zigbee_batch"),
    str(root / "tests/zigbee_batch/test.cpp"), "-o", str(output / "test"),
], check=True)
subprocess.run([str(output / "test")], check=True)

# Parse YAML without importing or invoking ESPHome. Guard the battery contract
# against accidentally replacing warmup sleep or adding awake delay actions.
sketch = "sketches/esp32_c6_th/esp32-c6-zigbee-th.yaml"
config = yaml.safe_load((root / sketch).read_text())
baseline = yaml.safe_load(subprocess.check_output(
    ["git", "show", "HEAD:" + sketch], cwd=root, text=True))
def delays(node):
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "delay":
                yield value
            yield from delays(value)
    elif isinstance(node, list):
        for value in node:
            yield from delays(value)
def warmup(config):
    def strings(node):
        if isinstance(node, str):
            yield node
        elif isinstance(node, dict):
            for v in node.values():
                yield from strings(v)
        elif isinstance(node, list):
            for v in node:
                yield from strings(v)
    return next(s for s in strings(config) if "wait_for_duration(2000)" in s)
assert list(delays(config)) == list(delays(baseline))
assert warmup(config) == warmup(baseline)
assert config["zigbee"]["model"] == baseline["zigbee"]["model"] == "ptvo.switch"
assert config["preferences"]["flash_write_interval"] == "never"
assert "600LL * 1000000LL" in (root / sketch).read_text()
print("PASS: YAML parse, unchanged awake delays/warmup/model/report period, no periodic flash wake")
