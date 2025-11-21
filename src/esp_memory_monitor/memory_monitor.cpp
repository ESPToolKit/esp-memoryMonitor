#include "esp_memory_monitor/memory_monitor.h"

#include <algorithm>
#include <type_traits>
#include <utility>

ESPMemoryMonitor* ESPMemoryMonitor::_allocInstance = nullptr;

namespace {
constexpr const char* kSamplerTaskName = "ESPMemoryMon";

using RegisterFn = decltype(&heap_caps_register_failed_alloc_callback);
using HookFn = esp_alloc_failed_hook_t;

template <class AlwaysVoid, template <class...> class Op, class... Args>
struct is_detected_impl : std::false_type {};
template <template <class...> class Op, class... Args>
struct is_detected_impl<std::void_t<Op<Args...>>, Op, Args...> : std::true_type {};
template <template <class...> class Op, class... Args>
constexpr bool is_detected_v = is_detected_impl<void, Op, Args...>::value;

using RegisterWithArgFn = esp_err_t (*)(HookFn, void*);
using RegisterNoArgFn = esp_err_t (*)(HookFn);
using HookWithArgFn = void (*)(size_t, uint32_t, const char*, void*);
using HookNoArgFn = void (*)(size_t, uint32_t, const char*);

template <typename Hook>
using RegisterCallWithArgOp = decltype(heap_caps_register_failed_alloc_callback(std::declval<Hook>(), std::declval<void*>()));
template <typename Hook>
using RegisterCallNoArgOp = decltype(heap_caps_register_failed_alloc_callback(std::declval<Hook>()));

template <typename Hook>
using HookCallWithArgOp =
    decltype(std::declval<Hook>()(std::declval<size_t>(), std::declval<uint32_t>(), static_cast<const char*>(nullptr), static_cast<void*>(nullptr)));
template <typename Hook>
using HookCallNoArgOp =
    decltype(std::declval<Hook>()(std::declval<size_t>(), std::declval<uint32_t>(), static_cast<const char*>(nullptr)));

constexpr bool kRegisterSupportsArg = is_detected_v<RegisterCallWithArgOp, HookFn>;
constexpr bool kRegisterSupportsNoArg = is_detected_v<RegisterCallNoArgOp, HookFn>;

constexpr bool kHookTakesArg = is_detected_v<HookCallWithArgOp, HookFn>;
constexpr bool kHookTakesNoArg = is_detected_v<HookCallNoArgOp, HookFn>;

static_assert(kRegisterSupportsArg || kRegisterSupportsNoArg,
              "Unsupported heap_caps_register_failed_alloc_callback signature");
static_assert(kHookTakesArg || kHookTakesNoArg, "Unsupported failed alloc hook signature");

constexpr bool kCanUseThunk4 = kHookTakesArg && is_detected_v<RegisterCallWithArgOp, HookWithArgFn>;
constexpr bool kCanUseThunk3 = kHookTakesNoArg && (is_detected_v<RegisterCallWithArgOp, HookNoArgFn> || is_detected_v<RegisterCallNoArgOp, HookNoArgFn>);

static_assert(kCanUseThunk4 || kCanUseThunk3, "Unsupported failed alloc callback signature");

inline TickType_t delayTicks(uint32_t intervalMs) {
    const TickType_t ticks = pdMS_TO_TICKS(intervalMs);
    return ticks == 0 ? 1 : ticks;
}

inline size_t regionIndex(MemoryRegion region) {
    return region == MemoryRegion::Psram ? 1 : 0;
}
}  // namespace

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

    {
        LockGuard guard(_mutex);
        _history.clear();
        _thresholdStates = {ThresholdState::Normal, ThresholdState::Normal};
        _sampleCallback = nullptr;
        _thresholdCallback = nullptr;
        _allocCallback = nullptr;
    }

    if (_mutex != nullptr) {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }

    _initialized = false;
}

MemorySnapshot ESPMemoryMonitor::sampleNow() {
    if (!_initialized) {
        return {};
    }

    const MemorySnapshot snapshot = captureSnapshot();
    SampleCallback sampleCb;
    ThresholdCallback thresholdCb;
    std::vector<ThresholdEvent> events;

    {
        LockGuard guard(_mutex);
        appendHistoryLocked(snapshot);
        events = evaluateThresholdsLocked(snapshot);
        sampleCb = _sampleCallback;
        thresholdCb = _thresholdCallback;
    }

    for (const auto& evt : events) {
        if (thresholdCb) {
            thresholdCb(evt);
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
        usages.push_back(std::move(usage));
    }

    return usages;
#else
    return {};
#endif
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

bool ESPMemoryMonitor::registerFailedAllocCallback() {
    if constexpr (kCanUseThunk4) {
        heap_caps_register_failed_alloc_callback(&ESPMemoryMonitor::failedAllocThunk4, this);
        _allocHookType = AllocHookType::WithArg;
        return true;
    } else if constexpr (kCanUseThunk3) {
        _allocInstance = this;

        if constexpr (kRegisterSupportsArg) {
            heap_caps_register_failed_alloc_callback(&ESPMemoryMonitor::failedAllocThunk3, this);
        } else {
            heap_caps_register_failed_alloc_callback(&ESPMemoryMonitor::failedAllocThunk3);
        }

        _allocHookType = AllocHookType::NoArg;
        return true;
    } else {
        return false;
    }
}

void ESPMemoryMonitor::unregisterFailedAllocCallback() {
    if (_allocHookType == AllocHookType::WithArg) {
        if constexpr (kRegisterSupportsArg) {
            // Newer IDF builds may support unregister, but registering nullptr disconnects too.
            heap_caps_register_failed_alloc_callback(nullptr, nullptr);
        }
    } else if (_allocHookType == AllocHookType::NoArg) {
        if (_allocInstance == this) {
            _allocInstance = nullptr;

            if constexpr (kRegisterSupportsNoArg) {
                heap_caps_register_failed_alloc_callback(nullptr);
            } else if constexpr (kRegisterSupportsArg) {
                heap_caps_register_failed_alloc_callback(nullptr, nullptr);
            }
        }
    }

    _allocHookType = AllocHookType::None;
}

void ESPMemoryMonitor::failedAllocThunk3(size_t size, uint32_t caps, const char* functionName) {
    auto* self = _allocInstance;
    if (self == nullptr) {
        return;
    }
    self->handleAllocEvent(size, caps, functionName);
}

void ESPMemoryMonitor::failedAllocThunk4(size_t size, uint32_t caps, const char* functionName, void* arg) {
    auto* self = static_cast<ESPMemoryMonitor*>(arg);
    if (self == nullptr) {
        return;
    }
    self->handleAllocEvent(size, caps, functionName);
}
