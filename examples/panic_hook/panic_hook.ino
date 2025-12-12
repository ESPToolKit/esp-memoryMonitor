#include <Arduino.h>
#if defined(__has_include)
#if __has_include(<ArduinoJson.h>)
#include <ArduinoJson.h>
#define ESPMM_EXAMPLE_HAS_JSON 1
#endif
#endif
#ifndef ESPMM_EXAMPLE_HAS_JSON
#define ESPMM_EXAMPLE_HAS_JSON 0
#endif
#include <ESPMemoryMonitor.h>
#include <esp_log.h>

#include <vector>

ESPMemoryMonitor monitor;
static uint32_t lastSampleMs = 0;

static void emitSnapshot(const MemorySnapshot& snap) {
#if ESPMM_HAS_ARDUINOJSON && ESPMM_EXAMPLE_HAS_JSON
    JsonDocument doc;
    toJson(snap, doc);
    serializeJson(doc, Serial);
    Serial.println();
#else
    for (const auto& region : snap.regions) {
        const char* name = region.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
        ESP_LOGI("MEM", "%s free=%uB min=%uB", name,
                 static_cast<unsigned>(region.freeBytes),
                 static_cast<unsigned>(region.minimumFreeBytes));
    }
#endif
}

void setup() {
    Serial.begin(115200);
    ESP_LOGI("PANIC", "Send 'p' over serial to simulate a crash and dump a panic snapshot");

    MemoryMonitorConfig cfg;
    cfg.sampleIntervalMs = 1200;
    cfg.historySize = 16;
    cfg.windowStatsSize = 6;
    cfg.internal = {48 * 1024, 32 * 1024};
    cfg.psram = {256 * 1024, 160 * 1024};
    cfg.enableScopes = true;
    cfg.maxScopesInHistory = 20;
    cfg.enablePerTaskStacks = true;
    cfg.enableTaskTracking = true;
    cfg.enableFailedAllocEvents = true;
    monitor.init(cfg);

    monitor.setTaskStackThreshold("loopTask", {2048, 1024});

    monitor.onSample(emitSnapshot);

    monitor.onFailedAlloc([](const FailedAllocEvent& evt) {
        ESP_LOGE("ALLOC", "failed alloc size=%u caps=0x%08x from %s",
                 static_cast<unsigned>(evt.requestedBytes), evt.caps, evt.functionName);
    });

    monitor.onTaskStackThreshold([](const TaskStackEvent& evt) {
        if (evt.appeared) {
            ESP_LOGI("TASK", "task appeared: %s", evt.usage.name.c_str());
            return;
        }
        if (evt.disappeared) {
            ESP_LOGW("TASK", "task disappeared: %s", evt.usage.name.c_str());
            return;
        }
        ESP_LOGW("TASK", "%s stack %s (%uB headroom)",
                 evt.usage.name.c_str(),
                 evt.state == StackState::Critical ? "CRITICAL" : evt.state == StackState::Warn ? "WARN"
                                                                                                : "SAFE",
                 static_cast<unsigned>(evt.usage.freeHighWaterBytes));
    });

    if (!monitor.installPanicHook([](const MemorySnapshot& snap) {
            ESP_LOGE("PANIC", "captured panic snapshot");
            emitSnapshot(snap);
        })) {
        ESP_LOGE("PANIC", "failed to register panic hook");
    }

    lastSampleMs = millis();
}

void loop() {
    // Create some heap churn; in real code this could be your payload buffers.
    {
        auto scope = monitor.beginScope("stream_chunk");
        std::vector<uint8_t> frame(12 * 1024, 0xA5);
        delay(40);
        (void)frame;
    }

    // Periodic sampling from the main loop.
    const uint32_t now = millis();
    if (now - lastSampleMs >= 1200) {
        monitor.sampleNow();
        lastSampleMs = now;
    }

    // Send 'p' over serial to force a panic and see the hook in action.
    if (Serial.available()) {
        const int c = Serial.read();
        if (c == 'p' || c == 'P') {
            ESP_LOGE("PANIC", "simulating panic...");
            abort();
        }
    }

    delay(80);
}
