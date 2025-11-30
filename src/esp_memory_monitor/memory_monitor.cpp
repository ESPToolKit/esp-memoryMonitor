#include "esp_memory_monitor/memory_monitor.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

#include <esp_log.h>
#include <esp_system.h>

namespace {
constexpr const char* kSamplerTaskName = "ESPMemoryMon";
constexpr const char* kLogTag = "ESPMemoryMon";
constexpr float kFragmentationEpsilon = 0.05f;

inline TickType_t delayTicks(uint32_t intervalMs) {
    const TickType_t ticks = pdMS_TO_TICKS(intervalMs);
    return ticks == 0 ? 1 : ticks;
}

inline size_t regionIndex(MemoryRegion region) {
    return region == MemoryRegion::Psram ? 1 : 0;
}

inline size_t saturatingSubtract(size_t value, size_t delta) {
    return value > delta ? value - delta : 0;
}
}  // namespace

static ESPMemoryMonitor* gPanicInstance = nullptr;
ESPMemoryMonitor* ESPMemoryMonitor::_failedAllocInstance = nullptr;

MemoryScope::MemoryScope(ESPMemoryMonitor* monitor, std::string name, MemoryTag tag, size_t startInternal, size_t startPsram, uint64_t startUs)
    : _monitor(monitor), _name(std::move(name)), _tag(tag), _startInternal(startInternal), _startPsram(startPsram), _startUs(startUs) {}

MemoryScope::~MemoryScope() {
    if (active()) {
        end();
    }
}

MemoryScope::MemoryScope(MemoryScope&& other) noexcept {
    *this = std::move(other);
}

MemoryScope& MemoryScope::operator=(MemoryScope&& other) noexcept {
    if (this != &other) {
        if (active()) {
            end();
        }
        _monitor = other._monitor;
        _name = std::move(other._name);
        _tag = other._tag;
        _startInternal = other._startInternal;
        _startPsram = other._startPsram;
        _startUs = other._startUs;
        _ended = other._ended;

        other._monitor = nullptr;
        other._ended = true;
    }
    return *this;
}

ScopeStats MemoryScope::end() {
    if (!active()) {
        return {};
    }

    _ended = true;
    ScopeStats stats = _monitor->finalizeScope(*this);
    _monitor = nullptr;
    return stats;
}

ESPMemoryMonitor::~ESPMemoryMonitor() {
    deinit();
}

bool ESPMemoryMonitor::init(const MemoryMonitorConfig& config) {
    if (_initialized) {
        deinit();
    }

    _config = config;

    _mutex = xSemaphoreCreateMutex();
    if (_mutex == nullptr) {
        return false;
    }

    _tagUsage.clear();
    _tagBudgets.clear();
    _scopeHistory.clear();
    _knownTasks.clear();
    _leakHistory.clear();
    _leakCheckpoints.clear();

    if (_config.enableFailedAllocEvents) {
        registerFailedAllocCallback();
    }

    _running = _config.enableSamplerTask && _config.sampleIntervalMs > 0;

    if (_running) {
        const BaseType_t created = xTaskCreatePinnedToCore(
            samplerTaskThunk,
            kSamplerTaskName,
            _config.stackSize,
            this,
            _config.priority,
            &_samplerTask,
            _config.coreId);

        if (created != pdPASS) {
            _running = false;
            unregisterFailedAllocCallback();
            vSemaphoreDelete(_mutex);
            _mutex = nullptr;
            return false;
        }
    }

    _initialized = true;
    return true;
}

void ESPMemoryMonitor::deinit() {
    if (!_initialized) {
        return;
    }

    _running = false;

    if (_samplerTask != nullptr) {
        vTaskDelete(_samplerTask);
        _samplerTask = nullptr;
    }

    unregisterFailedAllocCallback();
    unregisterPanicHandler();

    {
        LockGuard guard(_mutex);
        _history.clear();
        _scopeHistory.clear();
        _tagUsage.clear();
        _tagBudgets.clear();
        _knownTasks.clear();
        _leakHistory.clear();
        _leakCheckpoints.clear();
        _thresholdStates = {ThresholdState::Normal, ThresholdState::Normal};
        _sampleCallback = nullptr;
        _thresholdCallback = nullptr;
        _allocCallback = nullptr;
        _scopeCallback = nullptr;
        _tagThresholdCallback = nullptr;
        _taskStackCallback = nullptr;
        _leakCallback = nullptr;
        _panicCallback = nullptr;
    }

    if (_mutex != nullptr) {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }

    _initialized = false;
    _panicHookInstalled = false;
}

MemorySnapshot ESPMemoryMonitor::sampleNow() {
    if (!_initialized) {
        return {};
    }

    MemorySnapshot snapshot = captureSnapshot();
    SampleCallback sampleCb;
    ThresholdCallback thresholdCb;
    TaskStackThresholdCallback stackCb;
    std::vector<ThresholdEvent> events;
    std::vector<TaskStackEvent> stackEvents;

    {
        LockGuard guard(_mutex);
        enrichSnapshotLocked(snapshot);
        appendHistoryLocked(snapshot);
        events = evaluateThresholdsLocked(snapshot);
        trackTasksLocked(snapshot, stackEvents);
        sampleCb = _sampleCallback;
        thresholdCb = _thresholdCallback;
        stackCb = _taskStackCallback;
    }

    for (const auto& evt : events) {
        if (thresholdCb) {
            thresholdCb(evt);
        }
    }

    for (const auto& evt : stackEvents) {
        if (stackCb) {
            stackCb(evt);
        }
    }

    if (sampleCb) {
        sampleCb(snapshot);
    }

    return snapshot;
}

std::vector<MemorySnapshot> ESPMemoryMonitor::history() const {
    LockGuard guard(_mutex);
    return std::vector<MemorySnapshot>(_history.begin(), _history.end());
}

MemoryMonitorConfig ESPMemoryMonitor::currentConfig() const {
    LockGuard guard(_mutex);
    return _config;
}

void ESPMemoryMonitor::onSample(SampleCallback callback) {
    LockGuard guard(_mutex);
    _sampleCallback = std::move(callback);
}

void ESPMemoryMonitor::onThreshold(ThresholdCallback callback) {
    LockGuard guard(_mutex);
    _thresholdCallback = std::move(callback);
}

void ESPMemoryMonitor::onFailedAlloc(FailedAllocCallback callback) {
    LockGuard guard(_mutex);
    _allocCallback = std::move(callback);
}

void ESPMemoryMonitor::onScope(ScopeCallback callback) {
    LockGuard guard(_mutex);
    _scopeCallback = std::move(callback);
}

void ESPMemoryMonitor::onTagThreshold(TagThresholdCallback callback) {
    LockGuard guard(_mutex);
    _tagThresholdCallback = std::move(callback);
}

void ESPMemoryMonitor::onTaskStackThreshold(TaskStackThresholdCallback callback) {
    LockGuard guard(_mutex);
    _taskStackCallback = std::move(callback);
}

void ESPMemoryMonitor::onLeakCheck(LeakCheckCallback callback) {
    LockGuard guard(_mutex);
    _leakCallback = std::move(callback);
}

MemoryScope ESPMemoryMonitor::beginScope(const std::string& name, MemoryTag tag) {
    if (!_initialized || !_config.enableScopes) {
        return {};
    }

    const uint64_t startUs = esp_timer_get_time();
    const RegionStats internal = captureRegion(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT, MemoryRegion::Internal);
    const RegionStats psram = captureRegion(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MemoryRegion::Psram);

    return MemoryScope(this, name, tag, internal.freeBytes, psram.freeBytes, startUs);
}

MemoryTag ESPMemoryMonitor::registerTag(const std::string& name) {
    LockGuard guard(_mutex);
    const MemoryTag tag = static_cast<MemoryTag>(_tagUsage.size() + 1);
    _tagUsage.push_back({tag, name, 0, 0, ThresholdState::Normal});
    _tagBudgets.push_back({});
    return tag;
}

bool ESPMemoryMonitor::setTagBudget(MemoryTag tag, const TagBudget& budget) {
    LockGuard guard(_mutex);
    if (tag == kInvalidMemoryTag || tag == 0 || static_cast<size_t>(tag) > _tagBudgets.size()) {
        return false;
    }

    _tagBudgets[tag - 1] = budget;
    return true;
}

std::vector<ScopeStats> ESPMemoryMonitor::scopeHistory() const {
    LockGuard guard(_mutex);
    return std::vector<ScopeStats>(_scopeHistory.begin(), _scopeHistory.end());
}

std::vector<TagUsage> ESPMemoryMonitor::tagUsage() const {
    LockGuard guard(_mutex);
    return _tagUsage;
}

LeakCheckResult ESPMemoryMonitor::markLeakCheckPoint(const std::string& label) {
    if (!_initialized) {
        return {};
    }

    sampleNow();

    LeakCheckResult result{};
    LeakCheckCallback cb;
    {
        LockGuard guard(_mutex);
        result = buildLeakCheckLocked(label);
        cb = _leakCallback;
    }

    if (cb && !result.deltas.empty()) {
        cb(result);
    }

    return result;
}

bool ESPMemoryMonitor::setTaskStackThreshold(const std::string& taskName, const TaskStackThreshold& threshold) {
    LockGuard guard(_mutex);
    _taskThresholds[taskName] = threshold;
    return true;
}

bool ESPMemoryMonitor::installPanicHook(PanicCallback callback) {
    if (!_initialized) {
        return false;
    }

    LockGuard guard(_mutex);
    _panicCallback = std::move(callback);
    if (_panicHookInstalled) {
        return true;
    }

    if (!registerPanicHandler()) {
        return false;
    }

    _panicHookInstalled = true;
    return true;
}

void ESPMemoryMonitor::uninstallPanicHook() {
    LockGuard guard(_mutex);
    _panicCallback = nullptr;
    unregisterPanicHandler();
    _panicHookInstalled = false;
}

void ESPMemoryMonitor::samplerTaskThunk(void* arg) {
    auto* self = static_cast<ESPMemoryMonitor*>(arg);
    if (self == nullptr) {
        vTaskDelete(nullptr);
        return;
    }

    self->samplerTaskLoop();
    vTaskDelete(nullptr);
}

void ESPMemoryMonitor::samplerTaskLoop() {
    while (_running) {
        sampleNow();
        vTaskDelay(delayTicks(_config.sampleIntervalMs));
    }
}

MemorySnapshot ESPMemoryMonitor::captureSnapshot() const {
    MemorySnapshot snapshot{};
    snapshot.timestampUs = esp_timer_get_time();

    snapshot.regions.push_back(captureRegion(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT, MemoryRegion::Internal));
    snapshot.regions.push_back(captureRegion(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MemoryRegion::Psram));

    if (_config.enablePerTaskStacks) {
        snapshot.stacks = captureStacks();
    }

    return snapshot;
}

RegionStats ESPMemoryMonitor::captureRegion(uint32_t caps, MemoryRegion region) const {
    RegionStats stats{};
    stats.region = region;

    multi_heap_info_t info{};
    heap_caps_get_info(&info, caps);

    stats.freeBytes = info.total_free_bytes;
    stats.largestFreeBlock = info.largest_free_block;
    if (_config.enableMinEverFree) {
        stats.minimumFreeBytes = heap_caps_get_minimum_free_size(caps);
    }
    if (_config.enableFragmentation && stats.freeBytes > 0 && stats.largestFreeBlock <= stats.freeBytes) {
        stats.fragmentation = 1.0f - static_cast<float>(stats.largestFreeBlock) / static_cast<float>(stats.freeBytes);
    } else {
        stats.fragmentation = 0.0f;
    }

    return stats;
}

std::vector<TaskStackUsage> ESPMemoryMonitor::captureStacks() const {
#if defined(configUSE_TRACE_FACILITY) && (configUSE_TRACE_FACILITY == 1)
    const UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    if (taskCount == 0) {
        return {};
    }

    std::vector<TaskStatus_t> statuses(taskCount);
    uint32_t totalRuntime = 0;
    const UBaseType_t written = uxTaskGetSystemState(statuses.data(), statuses.size(), &totalRuntime);
    statuses.resize(written);

    std::vector<TaskStackUsage> usages;
    usages.reserve(statuses.size());
    for (const auto& status : statuses) {
        TaskStackUsage usage{};
        usage.name = status.pcTaskName != nullptr ? status.pcTaskName : "unknown";
        usage.freeHighWaterBytes = static_cast<size_t>(status.usStackHighWaterMark) * sizeof(StackType_t);
        usage.state = status.eCurrentState;
        usage.priority = status.uxCurrentPriority;
        usage.handle = status.xHandle;
        usages.push_back(std::move(usage));
    }

    return usages;
#else
    return {};
#endif
}

ScopeStats ESPMemoryMonitor::finalizeScope(const MemoryScope& scope) {
    ScopeStats stats{};
    stats.name = scope._name;
    stats.tag = scope._tag;
    stats.startInternalFreeBytes = scope._startInternal;
    stats.startPsramFreeBytes = scope._startPsram;

    const uint64_t endUs = esp_timer_get_time();
    const RegionStats internal = captureRegion(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT, MemoryRegion::Internal);
    const RegionStats psram = captureRegion(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MemoryRegion::Psram);

    stats.endInternalFreeBytes = internal.freeBytes;
    stats.endPsramFreeBytes = psram.freeBytes;
    stats.deltaInternalBytes = static_cast<int64_t>(scope._startInternal) - static_cast<int64_t>(internal.freeBytes);
    stats.deltaPsramBytes = static_cast<int64_t>(scope._startPsram) - static_cast<int64_t>(psram.freeBytes);
    stats.startedUs = scope._startUs;
    stats.endedUs = endUs;
    stats.durationUs = endUs - scope._startUs;

    ScopeCallback scopeCb;
    TagThresholdCallback tagCb;
    std::vector<TagThresholdEvent> tagEvents;
    {
        LockGuard guard(_mutex);
        appendScopeLocked(stats);
        tagEvents = evaluateTagThresholdsLocked(stats);
        scopeCb = _scopeCallback;
        tagCb = _tagThresholdCallback;
    }

    if (scopeCb) {
        scopeCb(stats);
    }

    if (tagCb) {
        for (const auto& evt : tagEvents) {
            tagCb(evt);
        }
    }

    return stats;
}

void ESPMemoryMonitor::appendHistoryLocked(const MemorySnapshot& snapshot) {
    if (_config.historySize == 0) {
        return;
    }

    _history.push_back(snapshot);
    while (_history.size() > _config.historySize) {
        _history.pop_front();
    }
}

std::vector<ThresholdEvent> ESPMemoryMonitor::evaluateThresholdsLocked(const MemorySnapshot& snapshot) {
    std::vector<ThresholdEvent> events;
    events.reserve(snapshot.regions.size());

    for (const auto& regionStats : snapshot.regions) {
        const size_t idx = regionIndex(regionStats.region);
        const ThresholdState current = _thresholdStates[idx];
        const RegionThreshold& limits = regionStats.region == MemoryRegion::Psram ? _config.psram : _config.internal;

        if (limits.warnBytes == 0 && limits.criticalBytes == 0) {
            continue;
        }

        const ThresholdState next = evaluateState(current, limits, regionStats.freeBytes);
        if (next != current) {
            _thresholdStates[idx] = next;
            events.push_back({regionStats.region, next, regionStats});
        }
    }

    return events;
}

ThresholdState ESPMemoryMonitor::evaluateState(ThresholdState current, const RegionThreshold& threshold, size_t freeBytes) const {
    const size_t warnRecover = threshold.warnBytes + _config.thresholdHysteresisBytes;
    const size_t criticalRecover = threshold.criticalBytes + _config.thresholdHysteresisBytes;

    switch (current) {
        case ThresholdState::Normal:
            if (threshold.criticalBytes > 0 && freeBytes <= threshold.criticalBytes) {
                return ThresholdState::Critical;
            }
            if (threshold.warnBytes > 0 && freeBytes <= threshold.warnBytes) {
                return ThresholdState::Warn;
            }
            return ThresholdState::Normal;

        case ThresholdState::Warn:
            if (threshold.criticalBytes > 0 && freeBytes <= threshold.criticalBytes) {
                return ThresholdState::Critical;
            }
            if (threshold.warnBytes == 0 || freeBytes > warnRecover) {
                return ThresholdState::Normal;
            }
            return ThresholdState::Warn;

        case ThresholdState::Critical:
            if (threshold.criticalBytes == 0) {
                return threshold.warnBytes > 0 && freeBytes <= threshold.warnBytes ? ThresholdState::Warn : ThresholdState::Normal;
            }
            if (freeBytes <= threshold.criticalBytes || freeBytes <= criticalRecover) {
                return ThresholdState::Critical;
            }
            if (threshold.warnBytes > 0 && freeBytes <= warnRecover) {
                return ThresholdState::Warn;
            }
            return ThresholdState::Normal;
    }

    return ThresholdState::Normal;
}

void ESPMemoryMonitor::handleAllocEvent(size_t requestedBytes, uint32_t caps, const char* functionName) {
    FailedAllocCallback cb;
    {
        LockGuard guard(_mutex);
        cb = _allocCallback;
    }

    if (!cb) {
        return;
    }

    FailedAllocEvent event{};
    event.requestedBytes = requestedBytes;
    event.caps = caps;
    event.functionName = functionName;
    event.timestampUs = esp_timer_get_time();

    cb(event);
}

void ESPMemoryMonitor::allocFailedHook(size_t requestedBytes, uint32_t caps, const char* functionName) {
    if (_failedAllocInstance != nullptr) {
        _failedAllocInstance->handleAllocEvent(requestedBytes, caps, functionName);
    }
}

bool ESPMemoryMonitor::registerFailedAllocCallback() {
    _failedAllocInstance = this;

    esp_err_t err = heap_caps_register_failed_alloc_callback(&ESPMemoryMonitor::allocFailedHook);
    if (err != ESP_OK) {
        _failedAllocInstance = nullptr;
        return false;
    }
    return true;
}

void ESPMemoryMonitor::unregisterFailedAllocCallback() {
    heap_caps_register_failed_alloc_callback(nullptr);
    if (_failedAllocInstance == this) {
        _failedAllocInstance = nullptr;
    }
}

void ESPMemoryMonitor::enrichSnapshotLocked(MemorySnapshot& snapshot) const {
    const size_t windowSize = _config.windowStatsSize;
    if (windowSize == 0) {
        return;
    }

    for (auto& region : snapshot.regions) {
        const size_t idx = regionIndex(region.region);

        std::vector<const RegionStats*> window;
        std::vector<uint64_t> timestamps;
        const size_t targetWindow = std::min(windowSize, _history.size() + 1);
        window.reserve(targetWindow);
        timestamps.reserve(targetWindow);

        if (targetWindow > 1) {
            size_t collected = 0;
            for (auto it = _history.rbegin(); it != _history.rend() && collected < targetWindow - 1; ++it, ++collected) {
                if (idx < it->regions.size()) {
                    window.push_back(&it->regions[idx]);
                    timestamps.push_back(it->timestampUs);
                }
            }
        }

        window.push_back(&region);
        timestamps.push_back(snapshot.timestampUs);

        std::reverse(window.begin(), window.end());
        std::reverse(timestamps.begin(), timestamps.end());

        size_t minFree = window.front()->freeBytes;
        size_t maxFree = window.front()->freeBytes;
        uint64_t sumFree = 0;
        float sumFrag = 0.0f;
        for (const auto* rs : window) {
            minFree = std::min(minFree, rs->freeBytes);
            maxFree = std::max(maxFree, rs->freeBytes);
            sumFree += rs->freeBytes;
            sumFrag += rs->fragmentation;
        }

        const float count = static_cast<float>(window.size());
        region.window.minFree = minFree;
        region.window.maxFree = maxFree;
        region.window.avgFree = static_cast<size_t>(sumFree / static_cast<uint64_t>(window.size()));
        region.window.avgFragmentation = count > 0.0f ? sumFrag / count : 0.0f;

        region.freeBytesSlope = 0.0f;
        region.secondsToWarn = 0;
        region.secondsToCritical = 0;

        if (window.size() >= 2) {
            const float deltaFree = static_cast<float>(window.back()->freeBytes) - static_cast<float>(window.front()->freeBytes);
            const float deltaSeconds = static_cast<float>(timestamps.back() - timestamps.front()) / 1'000'000.0f;
            if (deltaSeconds > 0.0f) {
                region.freeBytesSlope = deltaFree / deltaSeconds;
            }

            if (region.freeBytesSlope < 0.0f) {
                const RegionThreshold& limits = region.region == MemoryRegion::Psram ? _config.psram : _config.internal;
                if (limits.warnBytes > 0 && region.freeBytes > limits.warnBytes) {
                    const float seconds = (static_cast<float>(region.freeBytes) - static_cast<float>(limits.warnBytes)) / -region.freeBytesSlope;
                    region.secondsToWarn = static_cast<uint32_t>(seconds);
                }
                if (limits.criticalBytes > 0 && region.freeBytes > limits.criticalBytes) {
                    const float seconds = (static_cast<float>(region.freeBytes) - static_cast<float>(limits.criticalBytes)) / -region.freeBytesSlope;
                    region.secondsToCritical = static_cast<uint32_t>(seconds);
                }
            }
        }
    }
}

void ESPMemoryMonitor::appendScopeLocked(const ScopeStats& stats) {
    if (!_config.enableScopes) {
        return;
    }

    auto adjustUsage = [&](const ScopeStats& s, int direction) {
        if (s.tag == kInvalidMemoryTag || s.tag == 0 || static_cast<size_t>(s.tag) > _tagUsage.size()) {
            return;
        }

        TagUsage& usage = _tagUsage[s.tag - 1];
        auto apply = [&](size_t& target, int64_t delta) {
            int64_t next = static_cast<int64_t>(target) + direction * delta;
            if (next < 0) {
                next = 0;
            }
            target = static_cast<size_t>(next);
        };

        apply(usage.totalInternalBytes, s.deltaInternalBytes);
        apply(usage.totalPsramBytes, s.deltaPsramBytes);
    };

    adjustUsage(stats, 1);

    if (_config.maxScopesInHistory == 0) {
        return;
    }

    _scopeHistory.push_back(stats);
    while (_scopeHistory.size() > _config.maxScopesInHistory) {
        const ScopeStats& dropped = _scopeHistory.front();
        adjustUsage(dropped, -1);
        _scopeHistory.pop_front();
    }
}

std::vector<TagThresholdEvent> ESPMemoryMonitor::evaluateTagThresholdsLocked(const ScopeStats& stats) {
    std::vector<TagThresholdEvent> events;
    if (!_config.enableScopes || _tagUsage.empty()) {
        return events;
    }

    const size_t count = _tagUsage.size();
    events.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        TagUsage& usage = _tagUsage[i];
        const TagBudget& budget = _tagBudgets[i];
        if (budget.warnBytes == 0 && budget.criticalBytes == 0) {
            continue;
        }

        const size_t totalBytes = usage.totalInternalBytes + usage.totalPsramBytes;
        const ThresholdState next = evaluateRisingState(usage.state, budget, totalBytes);
        if (next != usage.state) {
            usage.state = next;
            TagThresholdEvent evt{};
            evt.usage = usage;
            evt.budget = budget;
            if (usage.tag == stats.tag) {
                evt.lastScope = stats;
            }
            events.push_back(std::move(evt));
        }
    }

    return events;
}

ThresholdState ESPMemoryMonitor::evaluateRisingState(ThresholdState current, const TagBudget& budget, size_t usageBytes) const {
    const size_t warnRecover = saturatingSubtract(budget.warnBytes, _config.thresholdHysteresisBytes);
    const size_t criticalRecover = saturatingSubtract(budget.criticalBytes, _config.thresholdHysteresisBytes);

    switch (current) {
        case ThresholdState::Normal:
            if (budget.criticalBytes > 0 && usageBytes >= budget.criticalBytes) {
                return ThresholdState::Critical;
            }
            if (budget.warnBytes > 0 && usageBytes >= budget.warnBytes) {
                return ThresholdState::Warn;
            }
            return ThresholdState::Normal;

        case ThresholdState::Warn:
            if (budget.criticalBytes > 0 && usageBytes >= budget.criticalBytes) {
                return ThresholdState::Critical;
            }
            if (budget.warnBytes == 0 || usageBytes <= warnRecover) {
                return ThresholdState::Normal;
            }
            return ThresholdState::Warn;

        case ThresholdState::Critical:
            if (budget.criticalBytes == 0) {
                if (budget.warnBytes > 0 && usageBytes >= budget.warnBytes) {
                    return ThresholdState::Warn;
                }
                return ThresholdState::Normal;
            }
            if (usageBytes >= criticalRecover) {
                return ThresholdState::Critical;
            }
            if (budget.warnBytes > 0 && usageBytes >= warnRecover) {
                return ThresholdState::Warn;
            }
            return ThresholdState::Normal;
    }

    return ThresholdState::Normal;
}

StackState ESPMemoryMonitor::computeStackState(const TaskStackUsage& usage) const {
    TaskStackThreshold threshold{};
    auto it = _taskThresholds.find(usage.name);
    if (it != _taskThresholds.end()) {
        threshold = it->second;
    } else {
        threshold.warnBytes = static_cast<size_t>(_config.defaultTaskStackBytes * _config.stackWarnFraction);
        threshold.criticalBytes = static_cast<size_t>(_config.defaultTaskStackBytes * _config.stackCriticalFraction);
    }

    if (threshold.criticalBytes > 0 && usage.freeHighWaterBytes <= threshold.criticalBytes) {
        return StackState::Critical;
    }
    if (threshold.warnBytes > 0 && usage.freeHighWaterBytes <= threshold.warnBytes) {
        return StackState::Warn;
    }
    return StackState::Safe;
}

void ESPMemoryMonitor::trackTasksLocked(const MemorySnapshot& snapshot, std::vector<TaskStackEvent>& events) {
    if (!_config.enablePerTaskStacks || !_config.enableTaskTracking) {
        return;
    }

    std::unordered_map<TaskHandle_t, TaskStackUsage> current;
    current.reserve(snapshot.stacks.size());

    for (const auto& usage : snapshot.stacks) {
        if (usage.handle == nullptr) {
            continue;
        }

        current[usage.handle] = usage;
        const StackState state = computeStackState(usage);

        auto it = _knownTasks.find(usage.handle);
        if (it == _knownTasks.end()) {
            events.push_back({usage, state, true, false});
        } else {
            const StackState previousState = computeStackState(it->second);
            if (previousState != state) {
                events.push_back({usage, state, false, false});
            }
        }
    }

    for (const auto& [handle, prior] : _knownTasks) {
        if (current.find(handle) == current.end()) {
            events.push_back({prior, computeStackState(prior), false, true});
        }
    }

    _knownTasks.swap(current);
}

LeakCheckResult ESPMemoryMonitor::buildLeakCheckLocked(const std::string& label) {
    LeakCheckResult result{};
    if (_history.empty()) {
        return result;
    }

    const uint64_t latestTs = _history.back().timestampUs;
    const uint64_t startTs = _leakCheckpoints.empty() ? _history.front().timestampUs : _leakCheckpoints.back();
    _leakCheckpoints.push_back(latestTs);

    auto computeAverages = [&](uint64_t from, uint64_t to) {
        struct RegionAvg {
            MemoryRegion region;
            float freeBytes = 0.0f;
            float fragmentation = 0.0f;
            size_t samples = 0;
        };

        std::array<RegionAvg, 2> avgs{{{MemoryRegion::Internal, 0.0f, 0.0f, 0}, {MemoryRegion::Psram, 0.0f, 0.0f, 0}}};

        for (const auto& snap : _history) {
            if (snap.timestampUs < from || snap.timestampUs > to) {
                continue;
            }
            for (const auto& region : snap.regions) {
                const size_t idx = regionIndex(region.region);
                avgs[idx].freeBytes += static_cast<float>(region.freeBytes);
                avgs[idx].fragmentation += region.fragmentation;
                avgs[idx].samples++;
            }
        }

        for (auto& avg : avgs) {
            if (avg.samples == 0) {
                continue;
            }
            avg.freeBytes /= static_cast<float>(avg.samples);
            avg.fragmentation /= static_cast<float>(avg.samples);
        }

        return avgs;
    };

    const auto averages = computeAverages(startTs, latestTs);

    if (_leakHistory.empty()) {
        result.fromLabel = label;
        result.toLabel = label;
        for (const auto& avg : averages) {
            LeakCheckDelta delta{};
            delta.region = avg.region;
            delta.averageStartFreeBytes = avg.freeBytes;
            delta.averageEndFreeBytes = avg.freeBytes;
            delta.averageStartFragmentation = avg.fragmentation;
            delta.averageEndFragmentation = avg.fragmentation;
            delta.deltaFreeBytes = 0.0f;
            delta.deltaFragmentation = 0.0f;
            result.deltas.push_back(delta);
        }
        _leakHistory.push_back(result);
        return result;
    }

    const auto& previous = _leakHistory.back();
    result.fromLabel = previous.toLabel.empty() ? previous.fromLabel : previous.toLabel;
    result.toLabel = label;

    for (const auto& avg : averages) {
        LeakCheckDelta delta{};
        delta.region = avg.region;
        delta.averageEndFreeBytes = avg.freeBytes;
        delta.averageEndFragmentation = avg.fragmentation;

        for (const auto& prevDelta : previous.deltas) {
            if (prevDelta.region == avg.region) {
                delta.averageStartFreeBytes = prevDelta.averageEndFreeBytes;
                delta.averageStartFragmentation = prevDelta.averageEndFragmentation;
                break;
            }
        }

        delta.deltaFreeBytes = delta.averageEndFreeBytes - delta.averageStartFreeBytes;
        delta.deltaFragmentation = delta.averageEndFragmentation - delta.averageStartFragmentation;
        if (delta.deltaFreeBytes < -static_cast<float>(_config.leakNoiseBytes) || delta.deltaFragmentation > kFragmentationEpsilon) {
            result.leakSuspected = true;
        }

        result.deltas.push_back(delta);
    }

    _leakHistory.push_back(result);
    return result;
}

void ESPMemoryMonitor::runPanicHook() {
    MemorySnapshot snapshot = captureSnapshot();
    PanicCallback cb;
    {
        LockGuard guard(_mutex);
        cb = _panicCallback;
    }

    for (const auto& region : snapshot.regions) {
        const char* name = region.region == MemoryRegion::Psram ? "PSRAM" : "DRAM";
        ESP_EARLY_LOGE(kLogTag, "panic snapshot %s free=%uB min=%uB largest=%uB frag=%.03f",
                       name,
                       static_cast<unsigned>(region.freeBytes),
                       static_cast<unsigned>(region.minimumFreeBytes),
                       static_cast<unsigned>(region.largestFreeBlock),
                       region.fragmentation);
    }

    if (cb) {
        cb(snapshot);
    }
}

bool ESPMemoryMonitor::registerPanicHandler() {
    gPanicInstance = this;
    const esp_err_t err = esp_register_shutdown_handler(&ESPMemoryMonitor::panicShutdownThunk);
    if (err != ESP_OK) {
        gPanicInstance = nullptr;
        return false;
    }
    return true;
}

void ESPMemoryMonitor::unregisterPanicHandler() {
    esp_unregister_shutdown_handler(&ESPMemoryMonitor::panicShutdownThunk);
    if (gPanicInstance == this) {
        gPanicInstance = nullptr;
    }
}

void ESPMemoryMonitor::panicShutdownThunk() {
    if (gPanicInstance != nullptr) {
        gPanicInstance->runPanicHook();
    }
}

#if ESPMM_HAS_ARDUINOJSON
void toJson(const MemorySnapshot& snap, JsonDocument& doc) {
    JsonObject root = doc.to<JsonObject>();
    root["timestampUs"] = snap.timestampUs;

    JsonArray regions = root["regions"].to<JsonArray>();
    for (const auto& region : snap.regions) {
        JsonObject obj = regions.add<JsonObject>();
        obj["region"] = region.region == MemoryRegion::Psram ? "psram" : "internal";
        obj["freeBytes"] = region.freeBytes;
        obj["minimumFreeBytes"] = region.minimumFreeBytes;
        obj["largestFreeBlock"] = region.largestFreeBlock;
        obj["fragmentation"] = region.fragmentation;
        obj["freeBytesSlope"] = region.freeBytesSlope;
        obj["secondsToWarn"] = region.secondsToWarn;
        obj["secondsToCritical"] = region.secondsToCritical;

        JsonObject window = obj["window"].to<JsonObject>();
        window["minFree"] = region.window.minFree;
        window["maxFree"] = region.window.maxFree;
        window["avgFree"] = region.window.avgFree;
        window["avgFragmentation"] = region.window.avgFragmentation;
    }

    JsonArray stacks = root["stacks"].to<JsonArray>();
    for (const auto& stack : snap.stacks) {
        JsonObject obj = stacks.add<JsonObject>();
        obj["name"] = stack.name;
        obj["freeHighWaterBytes"] = stack.freeHighWaterBytes;
        obj["state"] = static_cast<int>(stack.state);
        obj["priority"] = static_cast<unsigned>(stack.priority);
    }
}
#endif
