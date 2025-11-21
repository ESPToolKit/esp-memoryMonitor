#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPMemoryMonitor.h>
#include <esp_log.h>

static ESPMemoryMonitor monitor;
static MemoryTag httpTag;
static MemoryTag otaTag;

void setup() {
    Serial.begin(115200);

    MemoryMonitorConfig cfg;
    cfg.sampleIntervalMs = 1000;
    cfg.historySize = 40;
    cfg.internal = {48 * 1024, 32 * 1024};
    cfg.psram = {256 * 1024, 160 * 1024};
    cfg.enableScopes = true;
    cfg.maxScopesInHistory = 24;
    cfg.windowStatsSize = 8;
    cfg.enablePerTaskStacks = true;
    cfg.enableTaskTracking = true;
    monitor.init(cfg);

    httpTag = monitor.registerTag("http_server");
    otaTag = monitor.registerTag("ota");
    monitor.setTagBudget(httpTag, {60 * 1024, 80 * 1024});
    monitor.setTagBudget(otaTag, {40 * 1024, 64 * 1024});

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

    monitor.onLeakCheck([](const LeakCheckResult &res) {
        for (const auto &d : res.deltas) {
            const char *regionName = d.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
            ESP_LOGI("LEAK", "%s drift %+0.1fB frag %+0.2f", regionName, d.deltaFreeBytes, d.deltaFragmentation);
        }
        if (res.leakSuspected) {
            ESP_LOGW("LEAK", "possible leak between %s -> %s", res.fromLabel.c_str(), res.toLabel.c_str());
        }
    });

    monitor.onTaskStackThreshold([](const TaskStackEvent &evt) {
        if (evt.appeared) {
            ESP_LOGI("TASK", "task created: %s", evt.usage.name.c_str());
            return;
        }
        if (evt.disappeared) {
            ESP_LOGW("TASK", "task disappeared: %s", evt.usage.name.c_str());
            return;
        }
        ESP_LOGW("TASK", "%s stack %s (%uB headroom)",
                 evt.usage.name.c_str(),
                 evt.state == StackState::Critical ? "CRITICAL" :
                 evt.state == StackState::Warn ? "WARN" : "SAFE",
                 static_cast<unsigned>(evt.usage.freeHighWaterBytes));
    });
}

static void simulateHttpRequest() {
    auto scope = monitor.beginScope("http_req", httpTag);
    std::string payload(8 * 1024, 'x');  // simulate buffer
    delay(50);
    scope.end();
}

static void simulateOtaChunk() {
    auto scope = monitor.beginScope("ota_chunk", otaTag);
    std::vector<uint8_t> chunk(16 * 1024, 0xAA);
    delay(20);
    scope.end();
}

static void exportSnapshotJson() {
#if ESPMM_HAS_ARDUINOJSON
    StaticJsonDocument<2048> doc;
    const MemorySnapshot snap = monitor.sampleNow();
    toJson(snap, doc);
    serializeJson(doc, Serial);
    Serial.println();
#else
    (void)monitor;
#endif
}

void loop() {
    simulateHttpRequest();
    simulateOtaChunk();

    static uint32_t counter = 0;
    if (++counter % 30 == 0) {
        monitor.markLeakCheckPoint("steady_state");
        exportSnapshotJson();
    }

    delay(500);
}
