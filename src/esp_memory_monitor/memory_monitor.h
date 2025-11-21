#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
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

struct TagBudget {
    size_t warnBytes = 0;
    size_t criticalBytes = 0;
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
    bool enableScopes = false;
    size_t maxScopesInHistory = 32;
    size_t windowStatsSize = 0;
    bool enableTaskTracking = false;
    size_t defaultTaskStackBytes = 4096;
    float stackWarnFraction = 0.25f;
    float stackCriticalFraction = 0.10f;
    size_t leakNoiseBytes = 1024;
};

struct WindowStats {
    size_t minFree = 0;
    size_t maxFree = 0;
    size_t avgFree = 0;
    float avgFragmentation = 0.0f;
};

struct RegionStats {
    MemoryRegion region{MemoryRegion::Internal};
    size_t freeBytes = 0;
    size_t minimumFreeBytes = 0;
    size_t largestFreeBlock = 0;
    float fragmentation = 0.0f;
    float freeBytesSlope = 0.0f;
    uint32_t secondsToWarn = 0;
    uint32_t secondsToCritical = 0;
    WindowStats window{};
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
    TaskHandle_t handle = nullptr;
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

using MemoryTag = uint16_t;
constexpr MemoryTag kInvalidMemoryTag = 0;

struct ScopeStats {
    std::string name;
    MemoryTag tag = kInvalidMemoryTag;
    size_t startInternalFreeBytes = 0;
    size_t startPsramFreeBytes = 0;
    size_t endInternalFreeBytes = 0;
    size_t endPsramFreeBytes = 0;
    int64_t deltaInternalBytes = 0;
    int64_t deltaPsramBytes = 0;
    uint64_t durationUs = 0;
    uint64_t startedUs = 0;
    uint64_t endedUs = 0;
};

struct TagUsage {
    MemoryTag tag = kInvalidMemoryTag;
    std::string name;
    size_t totalInternalBytes = 0;
    size_t totalPsramBytes = 0;
    ThresholdState state{ThresholdState::Normal};
};

struct TagThresholdEvent {
    TagUsage usage;
    TagBudget budget;
    ScopeStats lastScope;
};

enum class StackState {
    Safe = 0,
    Warn,
    Critical,
};

struct TaskStackThreshold {
    size_t warnBytes = 0;
    size_t criticalBytes = 0;
};

struct TaskStackEvent {
    TaskStackUsage usage;
    StackState state{StackState::Safe};
    bool appeared = false;
    bool disappeared = false;
};

struct LeakCheckDelta {
    MemoryRegion region{MemoryRegion::Internal};
    float averageStartFreeBytes = 0.0f;
    float averageEndFreeBytes = 0.0f;
    float averageStartFragmentation = 0.0f;
    float averageEndFragmentation = 0.0f;
    float deltaFreeBytes = 0.0f;
    float deltaFragmentation = 0.0f;
};

struct LeakCheckResult {
    std::string fromLabel;
    std::string toLabel;
    std::vector<LeakCheckDelta> deltas;
    bool leakSuspected = false;
};

using ThresholdCallback = std::function<void(const ThresholdEvent&)>;
using SampleCallback = std::function<void(const MemorySnapshot&)>;
using FailedAllocCallback = std::function<void(const FailedAllocEvent&)>;
using ScopeCallback = std::function<void(const ScopeStats&)>;
using TagThresholdCallback = std::function<void(const TagThresholdEvent&)>;
using TaskStackThresholdCallback = std::function<void(const TaskStackEvent&)>;
using LeakCheckCallback = std::function<void(const LeakCheckResult&)>;
using PanicCallback = std::function<void(const MemorySnapshot&)>;

class MemoryScope;
class ESPMemoryMonitor;

class MemoryScope {
  public:
    MemoryScope() = default;
    ~MemoryScope();

    MemoryScope(const MemoryScope&) = delete;
    MemoryScope& operator=(const MemoryScope&) = delete;
    MemoryScope(MemoryScope&& other) noexcept;
    MemoryScope& operator=(MemoryScope&& other) noexcept;

    ScopeStats end();
    bool active() const { return _monitor != nullptr && !_ended; }

  private:
    friend class ESPMemoryMonitor;
    MemoryScope(ESPMemoryMonitor* monitor, std::string name, MemoryTag tag, size_t startInternal, size_t startPsram, uint64_t startUs);

    ESPMemoryMonitor* _monitor = nullptr;
    std::string _name;
    MemoryTag _tag = kInvalidMemoryTag;
    size_t _startInternal = 0;
    size_t _startPsram = 0;
    uint64_t _startUs = 0;
    bool _ended = false;
};

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
    void onScope(ScopeCallback callback);
    void onTagThreshold(TagThresholdCallback callback);
    void onTaskStackThreshold(TaskStackThresholdCallback callback);
    void onLeakCheck(LeakCheckCallback callback);

    MemoryScope beginScope(const std::string& name, MemoryTag tag = kInvalidMemoryTag);
    MemoryTag registerTag(const std::string& name);
    bool setTagBudget(MemoryTag tag, const TagBudget& budget);
    std::vector<ScopeStats> scopeHistory() const;
    std::vector<TagUsage> tagUsage() const;

    LeakCheckResult markLeakCheckPoint(const std::string& label);

    bool setTaskStackThreshold(const std::string& taskName, const TaskStackThreshold& threshold);

    bool installPanicHook(PanicCallback callback = nullptr);
    void uninstallPanicHook();

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

    static void allocFailedHook(size_t requestedBytes, uint32_t caps, const char* functionName);
    static ESPMemoryMonitor* _failedAllocInstance;
    
    MemorySnapshot captureSnapshot() const;
    RegionStats captureRegion(uint32_t caps, MemoryRegion region) const;
    std::vector<TaskStackUsage> captureStacks() const;
    ScopeStats finalizeScope(const MemoryScope& scope);

    void appendHistoryLocked(const MemorySnapshot& snapshot);
    std::vector<ThresholdEvent> evaluateThresholdsLocked(const MemorySnapshot& snapshot);
    ThresholdState evaluateState(ThresholdState current, const RegionThreshold& threshold, size_t freeBytes) const;
    void handleAllocEvent(size_t requestedBytes, uint32_t caps, const char* functionName);

    void enrichSnapshotLocked(MemorySnapshot& snapshot) const;
    void appendScopeLocked(const ScopeStats& stats);
    std::vector<TagThresholdEvent> evaluateTagThresholdsLocked(const ScopeStats& stats);
    ThresholdState evaluateRisingState(ThresholdState current, const TagBudget& budget, size_t usageBytes) const;
    StackState computeStackState(const TaskStackUsage& usage) const;
    void trackTasksLocked(const MemorySnapshot& snapshot, std::vector<TaskStackEvent>& events);
    LeakCheckResult buildLeakCheckLocked(const std::string& label);
    void runPanicHook();

    bool registerFailedAllocCallback();
    void unregisterFailedAllocCallback();
    bool registerPanicHandler();
    void unregisterPanicHandler();

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
    ScopeCallback _scopeCallback;
    TagThresholdCallback _tagThresholdCallback;
    TaskStackThresholdCallback _taskStackCallback;
    LeakCheckCallback _leakCallback;
    PanicCallback _panicCallback;
    std::deque<ScopeStats> _scopeHistory;
    std::vector<TagUsage> _tagUsage;
    std::vector<TagBudget> _tagBudgets;
    std::unordered_map<std::string, TaskStackThreshold> _taskThresholds;
    std::unordered_map<TaskHandle_t, TaskStackUsage> _knownTasks;
    std::vector<LeakCheckResult> _leakHistory;
    std::vector<uint64_t> _leakCheckpoints;
    bool _panicHookInstalled = false;
};

void toJson(const MemorySnapshot& snap, JsonDocument& doc);
