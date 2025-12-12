# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]
### Added
- More ESPMemoryMonitor examples: manual sampling without the background task and a panic-hook sketch that captures a final snapshot.
### Fixed
- Mark the failed-allocation hook instance pointer as static so Arduino builds compile the callback correctly.

## [1.0.0] - 2025-12-02
### Added
- Initial ESPMemoryMonitor library with snapshot API, background sampler task, and ring-buffer history for DRAM + PSRAM.
- Per-region warn/critical thresholds with hysteresis and callbacks.
- Optional fragmentation scores, min-ever-free tracking, per-task stack high-water marks, and failed-allocation event hook.
- Basic Arduino example sketch demonstrating sampling and alerts.

[Unreleased]: https://github.com/ESPToolKit/esp-memoryMonitor/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/ESPToolKit/esp-memoryMonitor/releases/tag/v1.0.0
