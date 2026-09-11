Repository: https://github.com/Aculeasis/myesphome

ESPHome components are located in `components/`.

Do not run ESPHome for validation.

`sketches/esp32_c6_th/esp32-c6-zigbee-th.yaml` runs on battery power.
It must spend as much time as possible in Light Sleep to maximize battery life.
Changes that explicitly or implicitly increase awake time or wake-up frequency
are STRICTLY PROHIBITED.
Adding ANY delays (`delay` actions, `delay()` calls, or equivalent awake waits)
to its main measurement/sleep cycle is STRICTLY PROHIBITED. Preserve light
sleep during sensor warmup and other existing power-saving behavior.

