#include <Arduino.h>
#include <ESPMemoryMonitor.h>
#include <esp_log.h>
#include <vector>

ESPMemoryMonitor monitor;
MemoryTag networkTag;

static uint32_t lastSampleMs = 0;
static constexpr uint32_t kSamplePeriodMs = 1500;

static void logSnapshot(const MemorySnapshot &snapshot) {
    for (const auto &region : snapshot.regions) {
        const char *name = region.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
        ESP_LOGI("MEM", "%s free=%uB min=%uB frag=%.02f slope=%.01fB/s",
                 name,
                 static_cast<unsigned>(region.freeBytes),
                 static_cast<unsigned>(region.minimumFreeBytes),
                 region.fragmentation,
                 region.freeBytesSlope);
    }
}

void setup() {
    Serial.begin(115200);

    // Disable the background sampler; we will call sampleNow() manually.
    MemoryMonitorConfig cfg;
    cfg.enableSamplerTask = false;
    cfg.historySize = 8;
    cfg.windowStatsSize = 5;
    cfg.internal = {48 * 1024, 32 * 1024};
    cfg.psram = {256 * 1024, 160 * 1024};
    cfg.enableScopes = true;
    cfg.maxScopesInHistory = 12;
    monitor.init(cfg);

    networkTag = monitor.registerTag("network");
    monitor.setTagBudget(networkTag, {48 * 1024, 64 * 1024});

    monitor.onThreshold([](const ThresholdEvent &evt) {
        const char *region = evt.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
        ESP_LOGW("THRESH", "%s now %s (%u free bytes)",
                 region,
                 evt.state == ThresholdState::Critical ? "CRITICAL" :
                 evt.state == ThresholdState::Warn ? "WARN" : "OK",
                 static_cast<unsigned>(evt.stats.freeBytes));
    });

    monitor.onSample(logSnapshot);

    monitor.onTagThreshold([](const TagThresholdEvent &evt) {
        ESP_LOGW("TAG", "%s %s at %u bytes",
                 evt.usage.name.c_str(),
                 evt.usage.state == ThresholdState::Critical ? "CRITICAL" :
                 evt.usage.state == ThresholdState::Warn ? "WARN" : "OK",
                 static_cast<unsigned>(evt.usage.totalInternalBytes + evt.usage.totalPsramBytes));
    });

    // Kick off an initial measurement so history() has data immediately.
    lastSampleMs = millis();
    monitor.sampleNow();
}

void loop() {
    // Simulate a request that consumes heap; attribute it to a tag.
    auto scope = monitor.beginScope("net_req", networkTag);
    std::vector<uint8_t> payload(6 * 1024, 0xCD);
    delay(40);
    scope.end();

    // Manual sampling cadence (no background task running).
    const uint32_t now = millis();
    if (now - lastSampleMs >= kSamplePeriodMs) {
        monitor.sampleNow();
        lastSampleMs = now;
    }

    // Periodically inspect the history buffer.
    static uint32_t historyTick = 0;
    if (++historyTick % 50 == 0) {
        const auto hist = monitor.history();
        ESP_LOGI("HIST", "stored snapshots: %u", static_cast<unsigned>(hist.size()));
    }

    delay(60);
}
