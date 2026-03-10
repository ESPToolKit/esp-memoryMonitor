#pragma once

#if __has_include(<ESPBufferManager.h>)
#include <ESPBufferManager.h>
#define ESP_MEMORY_MONITOR_HAS_BUFFER_MANAGER 1
#elif __has_include(<esp_buffer_manager/buffer_manager.h>)
#include <esp_buffer_manager/buffer_manager.h>
#define ESP_MEMORY_MONITOR_HAS_BUFFER_MANAGER 1
#else
#define ESP_MEMORY_MONITOR_HAS_BUFFER_MANAGER 0
#endif

#include <cstddef>
#include <cstdlib>
#include <deque>
#include <limits>
#include <new>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace memory_monitor_allocator_detail {
inline void *allocate(std::size_t bytes, bool usePSRAMBuffers) noexcept {
#if ESP_MEMORY_MONITOR_HAS_BUFFER_MANAGER
	return ESPBufferManager::allocate(bytes, usePSRAMBuffers);
#else
	(void)usePSRAMBuffers;
	return std::malloc(bytes);
#endif
}

inline void deallocate(void *ptr) noexcept {
#if ESP_MEMORY_MONITOR_HAS_BUFFER_MANAGER
	ESPBufferManager::deallocate(ptr);
#else
	std::free(ptr);
#endif
}
} // namespace memory_monitor_allocator_detail

template <typename T> class MemoryMonitorAllocator {
  public:
	using value_type = T;
	using propagate_on_container_move_assignment = std::true_type;
	using propagate_on_container_copy_assignment = std::true_type;
	using propagate_on_container_swap = std::true_type;
	using is_always_equal = std::false_type;

	MemoryMonitorAllocator() noexcept = default;
	explicit MemoryMonitorAllocator(bool usePSRAMBuffers) noexcept
	    : _usePSRAMBuffers(usePSRAMBuffers) {
	}

	template <typename U>
	MemoryMonitorAllocator(const MemoryMonitorAllocator<U> &other) noexcept
	    : _usePSRAMBuffers(other.usePSRAMBuffers()) {
	}

	T *allocate(std::size_t n) {
		if (n == 0) {
			return nullptr;
		}
		if (n > (std::numeric_limits<std::size_t>::max() / sizeof(T))) {
#if defined(__cpp_exceptions)
			throw std::bad_alloc();
#else
			std::abort();
#endif
		}

		void *memory = memory_monitor_allocator_detail::allocate(n * sizeof(T), _usePSRAMBuffers);
		if (memory == nullptr) {
#if defined(__cpp_exceptions)
			throw std::bad_alloc();
#else
			std::abort();
#endif
		}

		return static_cast<T *>(memory);
	}

	void deallocate(T *ptr, std::size_t) noexcept {
		memory_monitor_allocator_detail::deallocate(ptr);
	}

	bool usePSRAMBuffers() const noexcept {
		return _usePSRAMBuffers;
	}

	template <typename U> bool operator==(const MemoryMonitorAllocator<U> &other) const noexcept {
		return _usePSRAMBuffers == other.usePSRAMBuffers();
	}

	template <typename U> bool operator!=(const MemoryMonitorAllocator<U> &other) const noexcept {
		return !(*this == other);
	}

  private:
	template <typename> friend class MemoryMonitorAllocator;

	bool _usePSRAMBuffers = false;
};

template <typename T> using MemoryMonitorVector = std::vector<T, MemoryMonitorAllocator<T>>;

template <typename T> using MemoryMonitorDeque = std::deque<T, MemoryMonitorAllocator<T>>;

template <typename K, typename V, typename Hash = std::hash<K>, typename Eq = std::equal_to<K>>
using MemoryMonitorUnorderedMap =
    std::unordered_map<K, V, Hash, Eq, MemoryMonitorAllocator<std::pair<const K, V>>>;

using MemoryMonitorString =
    std::basic_string<char, std::char_traits<char>, MemoryMonitorAllocator<char>>;
