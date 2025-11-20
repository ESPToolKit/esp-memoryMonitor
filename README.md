# ESPMemoryMonitor

ESPMemoryMonitor is a tiny C++17 helper that wraps ESP-IDF heap/stack inspection APIs. It gathers one-shot or periodic snapshots, keeps a ring buffer for charts, and raises threshold callbacks (with hysteresis) before your firmware runs out of RAM or stack.

## CI / Release / License
[![CI](https://github.com/ESPToolKit/esp-memoryMonitor/actions/workflows/ci.yml/badge.svg)](https://github.com/ESPToolKit/esp-memoryMonitor/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/ESPToolKit/esp-memoryMonitor?sort=semver)](https://github.com/ESPToolKit/esp-memoryMonitor/releases)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE.md)

## Features
- One-shot snapshots via `sampleNow()` or a background sampler task that pushes results through `onSample` and records a ring buffer (default: 60 entries).
- Tracks internal DRAM and PSRAM (`MALLOC_CAP_8BIT` / `MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`) with free bytes, low-water mark, largest free block, and optional fragmentation score.
- Per-region thresholds with hysteresis; `onThreshold` fires on enter/exit of warn/critical bands so alerts do not spam as memory bounces.
- Optional extras: per-task stack high-water (via `uxTaskGetSystemState`), min-ever-free, and IDF failed-allocation callbacks (`heap_caps_register_failed_alloc_callback`).
- Thread-safe with FreeRTOS mutexes; destructor tears down the sampler task and unregisters callbacks.

## Examples
Minimal monitor with threshold alerting:

```cpp
#include <Arduino.h>
#include <ESPMemoryMonitor.h>
#include <esp_log.h>

static ESPMemoryMonitor monitor;

void setup() {
    Serial.begin(115200);

    MemoryMonitorConfig cfg;
    cfg.sampleIntervalMs = 2000;           // sampler task cadence
    cfg.historySize = 30;                  // keep 30 snapshots for charts/debug
    cfg.internal = {40 * 1024, 20 * 1024}; // warn/critical thresholds
    cfg.psram = {200 * 1024, 120 * 1024};
    cfg.enablePerTaskStacks = true;        // include per-task stack watermarks
    monitor.init(cfg);

    monitor.onThreshold([](const ThresholdEvent &evt) {
        const char *region = evt.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
        if (evt.state == ThresholdState::Critical) {
            ESP_LOGE("MEM", "%s critical: %u free bytes", region, static_cast<unsigned>(evt.stats.freeBytes));
        } else if (evt.state == ThresholdState::Warn) {
            ESP_LOGW("MEM", "%s warning: %u free bytes", region, static_cast<unsigned>(evt.stats.freeBytes));
        } else {
            ESP_LOGI("MEM", "%s recovered: %u free bytes", region, static_cast<unsigned>(evt.stats.freeBytes));
        }
    });

    monitor.onSample([](const MemorySnapshot &snapshot) {
        for (const auto &region : snapshot.regions) {
            const char *regionName = region.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
            ESP_LOGI("MEM", "%s free=%uB min=%uB frag=%.02f", regionName,
                     static_cast<unsigned>(region.freeBytes),
                     static_cast<unsigned>(region.minimumFreeBytes),
                     region.fragmentation);
        }
    });
}

void loop() {
    delay(1000);
}
```

When you need richer stack info or failed-allocation events, flip `enablePerTaskStacks` and `enableFailedAllocEvents` in the config. The latest snapshot and the full ring buffer are always available via `sampleNow()`/`history()`.

## API Reference
- `bool init(const MemoryMonitorConfig &cfg = {})` / `void deinit()` – start/stop the monitor. When `enableSamplerTask` is `false`, call `sampleNow()` manually.
- `MemorySnapshot sampleNow()` – collect a snapshot immediately; triggers callbacks and updates the ring buffer.
- `std::vector<MemorySnapshot> history()` – copy the stored snapshots (size capped by `historySize`).
- `MemoryMonitorConfig currentConfig() const` – inspect live settings.
- `void onSample(SampleCallback cb)` – receive every snapshot (from the sampler task or manual calls).
- `void onThreshold(ThresholdCallback cb)` – receive warn/critical transitions per memory region with hysteresis.
- `void onFailedAlloc(FailedAllocCallback cb)` – emit when IDF reports a failed allocation (requires `enableFailedAllocEvents`).

`MemoryMonitorConfig` knobs:

| Field | Default | Description |
| --- | --- | --- |
| `sampleIntervalMs` | `1000` | Period for the sampler task. Ignored when `enableSamplerTask` is `false`. |
| `historySize` | `60` | Ring buffer depth for snapshots (0 disables storage). |
| `stackSize` / `priority` / `coreId` | `4096*sizeof(StackType_t)`, `1`, `tskNO_AFFINITY` | FreeRTOS task parameters for the sampler. |
| `thresholdHysteresisBytes` | `4096` | Free-bytes margin required before leaving warn/critical bands. |
| `internal` / `psram` | `internal: 40KB/20KB`, `psram: disabled` | Warn/critical thresholds per region; keep PSRAM at `0` on boards without PSRAM. |
| `enableSamplerTask` | `true` | Disable to rely on manual `sampleNow()`. |
| `enableFragmentation` | `true` | Include `fragmentation = 1 - largestFreeBlock/freeBytes`. |
| `enableMinEverFree` | `true` | Include IDF’s minimum-ever-free metric per region. |
| `enablePerTaskStacks` | `false` | Collect per-task stack high-water marks (`uxTaskGetSystemState`). |
| `enableFailedAllocEvents` | `false` | Register `heap_caps_register_failed_alloc_callback` and forward failures to `onFailedAlloc`. |

`MemorySnapshot` holds `timestampUs` plus vectors of `RegionStats` (free bytes, low-water, largest block, fragmentation) and optional `TaskStackUsage` entries (task name, priority, state, free high-water bytes).

## Gotchas
- The sampler task is lightweight (a handful of heap calls per interval), but per-task stack scanning relies on `uxTaskGetSystemState` and `configUSE_TRACE_FACILITY=1`.
- `enableFailedAllocEvents` hooks IDF’s global failed-allocation callback; if you already use it elsewhere, coordinate registration to avoid conflicts.
- Threshold hysteresis is byte-based; bump `thresholdHysteresisBytes` if your allocator churns in small bursts.
- Fragmentation is `0` when free bytes are zero; values closer to `1.0` indicate more fragmentation.

## Restrictions
- ESP32 + FreeRTOS (Arduino-ESP32 or ESP-IDF) with C++17 enabled.
- Heap stats rely on `esp_heap_caps.h`; per-task stack data requires trace facility support.
- No dynamic allocation inside the sampler path beyond what the STL containers already hold.

## Tests
A dedicated host test suite is not shipped yet. Exercise the library through `examples/basic_monitor` on hardware and wire it into your CI if you extend the feature set.

## License
MIT — see [LICENSE.md](LICENSE.md).

## ESPToolKit
- Check out other libraries: <https://github.com/orgs/ESPToolKit/repositories>
- Hang out on Discord: <https://discord.gg/WG8sSqAy>
- Support the project: <https://ko-fi.com/esptoolkit>
