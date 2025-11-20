#include <Arduino.h>
#include <ESPMemoryMonitor.h>
#include <esp_log.h>

ESPMemoryMonitor monitor;

void setup() {
    Serial.begin(115200);

    MemoryMonitorConfig cfg;
    cfg.sampleIntervalMs = 1000;
    cfg.historySize = 30;
    cfg.internal = {48 * 1024, 32 * 1024};
    cfg.psram = {256 * 1024, 160 * 1024};
    cfg.enablePerTaskStacks = true;
    cfg.enableFailedAllocEvents = true;
    monitor.init(cfg);

    monitor.onThreshold([](const ThresholdEvent &evt) {
        const char *region = evt.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
        if (evt.state == ThresholdState::Critical) {
            ESP_LOGE("MEM", "%s critical free=%u", region, static_cast<unsigned>(evt.stats.freeBytes));
        } else if (evt.state == ThresholdState::Warn) {
            ESP_LOGW("MEM", "%s warning free=%u", region, static_cast<unsigned>(evt.stats.freeBytes));
        } else {
            ESP_LOGI("MEM", "%s recovered free=%u", region, static_cast<unsigned>(evt.stats.freeBytes));
        }
    });

    monitor.onSample([](const MemorySnapshot &snapshot) {
        for (const auto &region : snapshot.regions) {
            const char *regionName = region.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
            ESP_LOGI("MEM", "%s free=%u min=%u frag=%.02f", regionName,
                     static_cast<unsigned>(region.freeBytes),
                     static_cast<unsigned>(region.minimumFreeBytes),
                     region.fragmentation);
        }
        for (const auto &stack : snapshot.stacks) {
            ESP_LOGD("STACK", "%s water=%uB priority=%u state=%d", stack.name.c_str(),
                     static_cast<unsigned>(stack.freeHighWaterBytes), stack.priority, static_cast<int>(stack.state));
        }
    });

    monitor.onFailedAlloc([](const FailedAllocEvent &evt) {
        ESP_LOGE("MEM", "alloc failed size=%u caps=0x%08x from %s", static_cast<unsigned>(evt.requestedBytes), evt.caps, evt.functionName);
    });
}

void loop() {
    delay(1000);
}
