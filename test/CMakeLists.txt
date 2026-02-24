# Host-side tests are not enabled because ESPMemoryMonitor depends on ESP-IDF
# heap/FreeRTOS primitives that are unavailable in the default host toolchain.
# Run on-device tests in test/test_memory_monitor_lifecycle with PlatformIO/Arduino.
message(STATUS "ESPMemoryMonitor: host tests are disabled.")
