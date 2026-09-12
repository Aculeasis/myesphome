Run from the repository root with a host C++ compiler, Python with PyYAML,
and Node.js:

```sh
python3 tests/zigbee_batch/run.py
node tests/zigbee_batch/converter.cjs
```

The C++ test compiles the actual component with simulated platform interfaces
and AddressSanitizer/UndefinedBehaviorSanitizer. It covers integer boundaries,
callback/timeout interleavings, late confirms, independent diagnostic deadlines,
GPIO rearming, uninterrupted sleep, scheduler deadlines and rejoin ownership.
The YAML check preserves the existing warmup lambda and awake delay actions.
The converter test executes the actual converter with only the exposes builder
stubbed out. No test imports or runs ESPHome.

Generated headers and binaries go to `temp_sources/zigbee_batch_tests`.
These tests do not simulate the closed Zigbee stack or measure physical current.
