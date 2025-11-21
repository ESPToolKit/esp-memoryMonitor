#include <Arduino.h>
#include <ESPMemoryMonitor.h>
#include <esp_log.h>

ESPMemoryMonitor monitor;
MemoryTag httpTag;

void setup() {
    Serial.begin(115200);

    MemoryMonitorConfig cfg;
    cfg.sampleIntervalMs = 1000;
    cfg.historySize = 30;
    cfg.internal = {48 * 1024, 32 * 1024};
    cfg.psram = {256 * 1024, 160 * 1024};
    cfg.enablePerTaskStacks = true;
    cfg.enableFailedAllocEvents = true;
    cfg.enableScopes = true;
    cfg.maxScopesInHistory = 16;
    cfg.windowStatsSize = 8;
    cfg.enableTaskTracking = true;
    monitor.init(cfg);

    httpTag = monitor.registerTag("http_server");
    monitor.setTagBudget(httpTag, {60 * 1024, 80 * 1024});

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

    monitor.onScope([](const ScopeStats &s) {
        ESP_LOGI("SCOPE", "%s used %+d DRAM %+d PSRAM in %llu us",
                 s.name.c_str(),
                 static_cast<int>(s.deltaInternalBytes),
                 static_cast<int>(s.deltaPsramBytes),
                 static_cast<unsigned long long>(s.durationUs));
    });

    monitor.onTagThreshold([](const TagThresholdEvent &evt) {
        ESP_LOGW("TAG", "%s now %s at %u bytes",
                 evt.usage.name.c_str(),
                 evt.usage.state == ThresholdState::Critical ? "CRITICAL" :
                 evt.usage.state == ThresholdState::Warn ? "WARN" : "OK",
                 static_cast<unsigned>(evt.usage.totalInternalBytes + evt.usage.totalPsramBytes));
    });

    monitor.onTaskStackThreshold([](const TaskStackEvent &evt) {
        if (evt.appeared) {
            ESP_LOGI("TASK", "task created: %s", evt.usage.name.c_str());
        } else if (evt.disappeared) {
            ESP_LOGW("TASK", "task disappeared: %s", evt.usage.name.c_str());
        } else {
            ESP_LOGW("TASK", "task %s stack %s (%uB headroom)", evt.usage.name.c_str(),
                     evt.state == StackState::Critical ? "CRITICAL" :
                     evt.state == StackState::Warn ? "WARN" : "SAFE",
                     static_cast<unsigned>(evt.usage.freeHighWaterBytes));
        }
    });

    monitor.onLeakCheck([](const LeakCheckResult &res) {
        for (const auto &d : res.deltas) {
            const char *regionName = d.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
            ESP_LOGI("LEAK", "%s drift %+0.1fB frag %+0.2f", regionName, d.deltaFreeBytes, d.deltaFragmentation);
        }
    });
}

void loop() {
    auto scope = monitor.beginScope("http_req", httpTag);
    delay(1000);
    scope.end();

    static uint32_t counter = 0;
    if (++counter % 60 == 0) {
        monitor.markLeakCheckPoint("steady_state");
    }
}
