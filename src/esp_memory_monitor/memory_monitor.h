#pragma once

#include <Arduino.h>

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <type_traits>
#include <vector>

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
}

#include <esp_heap_caps.h>
#include <esp_timer.h>

enum class MemoryRegion {
    Internal = 0,
    Psram = 1,
};

enum class ThresholdState {
    Normal = 0,
    Warn,
    Critical,
};

struct RegionThreshold {
    size_t warnBytes = 40 * 1024;
    size_t criticalBytes = 20 * 1024;
};

struct MemoryMonitorConfig {
    static constexpr BaseType_t any = tskNO_AFFINITY;

    uint32_t sampleIntervalMs = 1000;
    size_t historySize = 60;
    uint32_t stackSize = 4096 * sizeof(StackType_t);
    BaseType_t coreId = any;
    UBaseType_t priority = 1;
    uint32_t thresholdHysteresisBytes = 4 * 1024;
    RegionThreshold internal{};
    RegionThreshold psram{};
    bool enableSamplerTask = true;
    bool enableFragmentation = true;
    bool enableMinEverFree = true;
    bool enablePerTaskStacks = false;
    bool enableFailedAllocEvents = false;
};

struct RegionStats {
    MemoryRegion region{MemoryRegion::Internal};
    size_t freeBytes = 0;
    size_t minimumFreeBytes = 0;
    size_t largestFreeBlock = 0;
    float fragmentation = 0.0f;
};

struct ThresholdEvent {
    MemoryRegion region{MemoryRegion::Internal};
    ThresholdState state{ThresholdState::Normal};
    RegionStats stats{};
};

struct TaskStackUsage {
    std::string name;
    size_t freeHighWaterBytes = 0;
    eTaskState state = eInvalid;
    UBaseType_t priority = 0;
};

struct FailedAllocEvent {
    size_t requestedBytes = 0;
    uint32_t caps = 0;
    const char* functionName = nullptr;
    uint64_t timestampUs = 0;
};

struct MemorySnapshot {
    uint64_t timestampUs = 0;
    std::vector<RegionStats> regions;
    std::vector<TaskStackUsage> stacks;
};

using ThresholdCallback = std::function<void(const ThresholdEvent&)>;
using SampleCallback = std::function<void(const MemorySnapshot&)>;
using FailedAllocCallback = std::function<void(const FailedAllocEvent&)>;

class ESPMemoryMonitor {
  public:
    ESPMemoryMonitor() = default;
    ~ESPMemoryMonitor();

    bool init(const MemoryMonitorConfig& config = MemoryMonitorConfig{});
    void deinit();
    bool isInitialized() const { return _initialized; }

    MemorySnapshot sampleNow();
    std::vector<MemorySnapshot> history() const;
    MemoryMonitorConfig currentConfig() const;

    void onSample(SampleCallback callback);
    void onThreshold(ThresholdCallback callback);
    void onFailedAlloc(FailedAllocCallback callback);

  private:
    enum class AllocHookType {
        None = 0,
        WithArg,
        NoArg,
    };

    struct LockGuard {
        explicit LockGuard(SemaphoreHandle_t handle) : _handle(handle) {
            if (_handle != nullptr) {
                xSemaphoreTake(_handle, portMAX_DELAY);
            }
        }

        ~LockGuard() {
            if (_handle != nullptr) {
                xSemaphoreGive(_handle);
            }
        }

        LockGuard(const LockGuard&) = delete;
        LockGuard& operator=(const LockGuard&) = delete;

      private:
        SemaphoreHandle_t _handle;
    };

    static void samplerTaskThunk(void* arg);
    void samplerTaskLoop();

    MemorySnapshot captureSnapshot() const;
    RegionStats captureRegion(uint32_t caps, MemoryRegion region) const;
    std::vector<TaskStackUsage> captureStacks() const;

    void appendHistoryLocked(const MemorySnapshot& snapshot);
    std::vector<ThresholdEvent> evaluateThresholdsLocked(const MemorySnapshot& snapshot);
    ThresholdState evaluateState(ThresholdState current, const RegionThreshold& threshold, size_t freeBytes) const;
    void handleAllocEvent(size_t requestedBytes, uint32_t caps, const char* functionName);

    bool registerFailedAllocCallback();
    void unregisterFailedAllocCallback();
    static void failedAllocThunk3(size_t size, uint32_t caps, const char* functionName);
    static void failedAllocThunk4(size_t size, uint32_t caps, const char* functionName, void* arg);

    MemoryMonitorConfig _config{};
    bool _initialized = false;
    bool _running = false;
    SemaphoreHandle_t _mutex = nullptr;
    TaskHandle_t _samplerTask = nullptr;
    std::deque<MemorySnapshot> _history;
    std::array<ThresholdState, 2> _thresholdStates{ThresholdState::Normal, ThresholdState::Normal};
    SampleCallback _sampleCallback;
    ThresholdCallback _thresholdCallback;
    FailedAllocCallback _allocCallback;
    AllocHookType _allocHookType = AllocHookType::None;

    static ESPMemoryMonitor* _allocInstance;
};
