#include <Arduino.h>
#include <ESPMemoryMonitor.h>
#include <unity.h>

namespace {

MemoryMonitorConfig testConfig() {
	MemoryMonitorConfig cfg{};
	cfg.enableSamplerTask = false;
	cfg.sampleIntervalMs = 0;
	cfg.historySize = 4;
	cfg.windowStatsSize = 0;
	cfg.enableScopes = false;
	cfg.enablePerTaskStacks = false;
	cfg.enableTaskTracking = false;
	cfg.enableFailedAllocEvents = false;
	return cfg;
}

void test_deinit_is_safe_before_init() {
	ESPMemoryMonitor monitor;
	TEST_ASSERT_FALSE(monitor.isInitialized());

	monitor.deinit();
	TEST_ASSERT_FALSE(monitor.isInitialized());
}

void test_deinit_is_idempotent() {
	ESPMemoryMonitor monitor;
	TEST_ASSERT_TRUE(monitor.init(testConfig()));
	TEST_ASSERT_TRUE(monitor.isInitialized());

	monitor.deinit();
	TEST_ASSERT_FALSE(monitor.isInitialized());

	monitor.deinit();
	TEST_ASSERT_FALSE(monitor.isInitialized());
}

void test_reinit_after_deinit() {
	ESPMemoryMonitor monitor;
	TEST_ASSERT_TRUE(monitor.init(testConfig()));
	TEST_ASSERT_TRUE(monitor.isInitialized());

	monitor.deinit();
	TEST_ASSERT_FALSE(monitor.isInitialized());

	TEST_ASSERT_TRUE(monitor.init(testConfig()));
	TEST_ASSERT_TRUE(monitor.isInitialized());
	monitor.deinit();
}

void test_deinit_releases_runtime_state() {
	ESPMemoryMonitor monitor;
	TEST_ASSERT_TRUE(monitor.init(testConfig()));

	(void)monitor.sampleNow();
	(void)monitor.sampleNow();
	TEST_ASSERT_TRUE(monitor.history().size() > 0);

	monitor.deinit();
	TEST_ASSERT_FALSE(monitor.isInitialized());
	TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(monitor.history().size()));
}

void test_destructor_deinits_active_instance() {
	{
		ESPMemoryMonitor scoped;
		TEST_ASSERT_TRUE(scoped.init(testConfig()));
		TEST_ASSERT_TRUE(scoped.isInitialized());
		(void)scoped.sampleNow();
	}

	ESPMemoryMonitor second;
	TEST_ASSERT_TRUE(second.init(testConfig()));
	TEST_ASSERT_TRUE(second.isInitialized());
	second.deinit();
}

} // namespace

void setUp() {
}
void tearDown() {
}

void setup() {
	delay(2000);
	UNITY_BEGIN();
	RUN_TEST(test_deinit_is_safe_before_init);
	RUN_TEST(test_deinit_is_idempotent);
	RUN_TEST(test_reinit_after_deinit);
	RUN_TEST(test_deinit_releases_runtime_state);
	RUN_TEST(test_destructor_deinits_active_instance);
	UNITY_END();
}

void loop() {
	delay(1000);
}
