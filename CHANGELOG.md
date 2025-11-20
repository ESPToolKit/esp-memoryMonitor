# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

## [0.1.0] - 2025-02-20
### Added
- Initial ESPMemoryMonitor library with snapshot API, background sampler task, and ring-buffer history for DRAM + PSRAM.
- Per-region warn/critical thresholds with hysteresis and callbacks.
- Optional fragmentation scores, min-ever-free tracking, per-task stack high-water marks, and failed-allocation event hook.
- Basic Arduino example sketch demonstrating sampling and alerts.
