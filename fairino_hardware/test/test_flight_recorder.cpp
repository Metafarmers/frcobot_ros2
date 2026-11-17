#include "fairino_hardware/flight_recorder.hpp"

#include <gtest/gtest.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <new>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace
{
thread_local bool count_control_allocations = false;
thread_local size_t control_allocations = 0;
}

// Only the exact producer calls in the dedicated test enable this counter.
// Other test/framework/background allocations retain normal allocator behavior.
void * operator new(std::size_t bytes)
{
  if (count_control_allocations) {++control_allocations;}
  if (void * memory = std::malloc(bytes ? bytes : 1)) {return memory;}
  throw std::bad_alloc();
}
void * operator new[](std::size_t bytes) {return ::operator new(bytes);}
void operator delete(void * memory) noexcept {std::free(memory);}
void operator delete[](void * memory) noexcept {std::free(memory);}
void operator delete(void * memory, std::size_t) noexcept {std::free(memory);}
void operator delete[](void * memory, std::size_t) noexcept {std::free(memory);}

namespace fairino_hardware
{
namespace
{
constexpr int64_t kMs = 1000000;

class FlightRecorderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char path[] = "/tmp/fairino-recorder-test-XXXXXX";
    directory_ = mkdtemp(path);
    options_.output_directory = directory_;
    options_.metadata_directory = directory_;
    options_.pre_window_ns = 100 * kMs;
    options_.post_window_ns = 100 * kMs;
    options_.correlation_window_ns = 100 * kMs;
    options_.queue_capacity = 64;
    options_.history_capacity = 64;
    options_.incident_capacity = 128;
    options_.max_trace_files = 2;
  }
  void TearDown() override {std::filesystem::remove_all(directory_);}
  TraceEvent event(int64_t ms, TraceKind kind = TraceKind::STATE)
  {
    TraceEvent value{};
    value.stamp = {ms * kMs, 1700000000000000000LL + ms * kMs};
    value.control_tid = 1234;
    value.kind = kind;
    return value;
  }
  std::string read(const std::string & name)
  {
    std::ifstream stream(name);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  }
  std::string metadata()
  {
    return read(directory_ + "/hardware-" + std::to_string(getpid()) + ".json");
  }
  std::string trace()
  {
    std::string result;
    for (const auto & entry : std::filesystem::directory_iterator(directory_)) {
      if (entry.path().extension() == ".jsonl") {result += read(entry.path().string());}
    }
    return result;
  }
  size_t traces()
  {
    size_t count = 0;
    for (const auto & entry : std::filesystem::directory_iterator(directory_)) {
      if (entry.path().extension() == ".jsonl") {++count;}
    }
    return count;
  }
  FlightRecorderOptions options_;
  std::string directory_;
};

TEST_F(FlightRecorderTest, HealthyEventsOnlyWriteIdentityMetadataOnce) {
  FlightRecorder recorder(options_);
  for (int n = 1; n <= 20; ++n) {
    auto sample = event(n * 20, TraceKind::READ_BEGIN);
    sample.nominal_period_sec = .02;
    ASSERT_TRUE(recorder.record(sample));
    recorder.pollForTest(sample.stamp.mono_ns);
  }
  EXPECT_EQ(traces(), 0u);
  EXPECT_NE(metadata().find("\"phase\":\"armed\""), std::string::npos);
  EXPECT_NE(metadata().find("\"control_tid\":1234"), std::string::npos);
  EXPECT_EQ(metadata().find("\"process_start_ticks\":0"), std::string::npos);
  const auto file = directory_ + "/hardware-" + std::to_string(getpid()) + ".json";
  const auto modified = std::filesystem::last_write_time(file);
  ASSERT_TRUE(recorder.record(event(420)));
  recorder.pollForTest(420 * kMs);
  EXPECT_EQ(std::filesystem::last_write_time(file), modified);
  EXPECT_EQ(recorder.stats().incidents_started, 0u);
}

TEST_F(FlightRecorderTest, DefaultBuffersRetainOneMinuteBeforeAndAfterAt50Hz) {
  FlightRecorderOptions minute;
  minute.output_directory = directory_;
  minute.metadata_directory = directory_;
  EXPECT_EQ(minute.pre_window_ns, 60000000000LL);
  EXPECT_EQ(minute.post_window_ns, 60000000000LL);
  if (minute.pre_window_ns != 60000000000LL || minute.post_window_ns != 60000000000LL) {
    return;
  }
  FlightRecorder recorder(minute);
  const TraceKind kinds[] = {TraceKind::READ_BEGIN, TraceKind::RPC_BEGIN,
    TraceKind::RPC_END, TraceKind::READ_END, TraceKind::WRITE_BEGIN,
    TraceKind::RPC_BEGIN, TraceKind::RPC_END, TraceKind::WRITE_END};
  for (int64_t ms = 20; ms <= 125000; ms += 20) {
    for (size_t index = 0; index < 8; ++index) {
      auto sample = event(ms, kinds[index]);
      sample.nominal_period_sec = .02;
      sample.cycle_index = static_cast<uint64_t>(ms / 20);
      if (index == 1 || index == 2) {sample.rpc = TraceRpc::STATE;}
      if (index == 5 || index == 6) {sample.rpc = TraceRpc::SERVO;}
      if (ms == 65000 && index == 4) {sample.trigger_reasons = ACTION_ABORT;}
      ASSERT_TRUE(recorder.record(sample));
    }
    recorder.pollForTest(ms * kMs);
  }
  recorder.pollForTest(125001 * kMs);
  const auto output = trace();
  EXPECT_NE(output.find("\"mono_ns\":5000000000,"), std::string::npos);
  EXPECT_NE(output.find("\"mono_ns\":125000000000,"), std::string::npos);
  EXPECT_NE(output.find("\"post_until_mono_ns\":125000000000"), std::string::npos);
  EXPECT_EQ(recorder.stats().queue_dropped, 0u);
  EXPECT_EQ(recorder.stats().incident_dropped, 0u);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
}

TEST_F(FlightRecorderTest, UnstartedDestructorDiscardsWithoutMetadataOrTraceRotation) {
  options_.output_directory = directory_ + "/existing-traces";
  options_.metadata_directory = directory_ + "/metadata-must-not-be-created";
  options_.max_trace_files = 1;
  std::filesystem::create_directory(options_.output_directory);
  const auto genuine_trace = options_.output_directory + "/hardware-genuine.jsonl";
  std::ofstream(genuine_trace) << "existing hardware evidence";
  {
    FlightRecorder recorder(options_);
    auto sample = event(100, TraceKind::RPC_END);
    sample.rpc = TraceRpc::SERVO;
    sample.rc = 14;
    ASSERT_TRUE(recorder.record(sample));
    // No writer start and no explicit offline polling: destructor has no disk authority.
  }
  EXPECT_EQ(read(genuine_trace), "existing hardware evidence");
  EXPECT_FALSE(std::filesystem::exists(options_.metadata_directory));
  size_t files = 0;
  for (const auto & entry : std::filesystem::directory_iterator(options_.output_directory)) {
    (void)entry;
    ++files;
  }
  EXPECT_EQ(files, 1u);
}

TEST_F(FlightRecorderTest, ProducerDoesNotAllocateEvenWhenQueueIsFull) {
  options_.queue_capacity = 2;
  FlightRecorder recorder(options_);
  auto sample = event(100);
  control_allocations = 0;
  count_control_allocations = true;
  for (size_t index = 0; index < 1000; ++index) {
    sample.stamp = clockNow();
    sample.control_tid = controlThreadId();
    recorder.record(sample);
  }
  count_control_allocations = false;
  EXPECT_EQ(control_allocations, 0u);
  EXPECT_EQ(recorder.stats().queue_dropped, 998u);
}

TEST_F(FlightRecorderTest, SuccessfulSlowRpcTriggersAndPreservesPrePostState) {
  FlightRecorder recorder(options_);
  for (int n = 1; n < 10; ++n) {
    ASSERT_TRUE(recorder.record(event(n * 20)));
    recorder.pollForTest(n * 20 * kMs);
  }
  auto slow = event(200, TraceKind::RPC_END);
  slow.rpc = TraceRpc::SERVO;
  slow.rpc_begin_mono_ns = 150 * kMs;
  slow.rc = 0;
  slow.sdk_observed = true;
  slow.state_valid = true;
  slow.sdk_observed_mono_ns = 180 * kMs;
  slow.raw_sdk.frame_cnt = 42;
  slow.raw_sdk.actual_qdd[2] = 17.25;
  slow.raw_sdk.jt_cur_tor[5] = -5.5;
  slow.raw_sdk.rbtEnableState = 1;
  slow.raw_sdk.cl_dgt_input_l = 4;
  slow.command_valid = true;
  slow.command_rad[0] = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(recorder.record(slow));
  recorder.pollForTest(200 * kMs);
  EXPECT_NE(metadata().find("\"phase\":\"capturing\""), std::string::npos);
  EXPECT_NE(trace().find("\"mono_ns\":100000000"), std::string::npos);
  EXPECT_NE(trace().find("RPC_SLOW"), std::string::npos);
  ASSERT_TRUE(recorder.record(event(280)));
  recorder.pollForTest(301 * kMs);
  const auto output = trace();
  EXPECT_NE(output.find("RPC_SLOW"), std::string::npos);
  EXPECT_NE(output.find("\"mono_ns\":100000000"), std::string::npos);
  EXPECT_EQ(output.find("\"mono_ns\":80000000,"), std::string::npos);
  EXPECT_NE(output.find("\"mono_ns\":280000000"), std::string::npos);
  EXPECT_NE(output.find("\"frame_cnt\":42"), std::string::npos);
  EXPECT_NE(output.find("17.25"), std::string::npos);
  EXPECT_NE(output.find("\"raw_sdk_hex\":"), std::string::npos);
  EXPECT_NE(output.find("\"command_rad\":[null,"), std::string::npos);
  EXPECT_EQ(output.find("nan"), std::string::npos);
  EXPECT_EQ(recorder.stats().incidents_completed, 1u);
}

TEST_F(FlightRecorderTest, LongGapAndFollowingBurstAreCorrelatedWithoutExtension) {
  FlightRecorder recorder(options_);
  for (const int ms : {100, 160, 161, 162}) {
    auto sample = event(ms, TraceKind::WRITE_BEGIN);
    sample.nominal_period_sec = .02;
    ASSERT_TRUE(recorder.record(sample));
    recorder.pollForTest(ms * kMs);
  }
  recorder.pollForTest(261 * kMs);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  EXPECT_NE(trace().find("CYCLE_GAP"), std::string::npos);
  EXPECT_NE(trace().find("CATCH_UP"), std::string::npos);
  EXPECT_NE(metadata().find("\"post_until_mono_ns\":260000000"), std::string::npos);
}

TEST_F(FlightRecorderTest, ConsecutiveRapidCyclesTriggerWithoutPrecedingLongGap) {
  FlightRecorder recorder(options_);
  for (const int ms : {100, 101, 102}) {
    auto sample = event(ms, TraceKind::READ_BEGIN);
    sample.nominal_period_sec = .02;
    recorder.record(sample);
    recorder.pollForTest(sample.stamp.mono_ns);
  }
  recorder.pollForTest(203 * kMs);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  EXPECT_NE(trace().find("CATCH_UP"), std::string::npos);
}

TEST_F(FlightRecorderTest, FirstCycleDurationTriggersEvenWithoutPreviousCycle) {
  FlightRecorder recorder(options_);
  recorder.record(event(100, TraceKind::WRITE_BEGIN));
  recorder.record(event(150, TraceKind::WRITE_END));
  recorder.pollForTest(251 * kMs);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  EXPECT_NE(trace().find("CYCLE_GAP"), std::string::npos);
}

TEST_F(FlightRecorderTest, ExternalAbortSharesIncidentAndRetainsGoalIdentity) {
  FlightRecorder recorder(options_);
  auto fault = event(100);
  fault.trigger_reasons = CONTROLLER_FAULT;
  ASSERT_TRUE(recorder.record(fault));
  recorder.pollForTest(100 * kMs);
  recorder.pollForTest(201 * kMs);
  recorder.requestIncident(
    ACTION_ABORT, event(250).stamp,
    "01234567-89ab-cdef-0123-456789abcdef", 1699999999999999999LL);
  recorder.pollForTest(250 * kMs);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  EXPECT_NE(trace().find("ACTION_ABORT"), std::string::npos);
  EXPECT_NE(trace().find("01234567-89ab-cdef-0123-456789abcdef"), std::string::npos);
  EXPECT_NE(trace().find("1699999999999999999"), std::string::npos);
  EXPECT_NE(metadata().find("\"post_until_mono_ns\":200000000"), std::string::npos);
}

TEST_F(FlightRecorderTest, PersistentFaultStateDoesNotTriggerRecurringEpisodes) {
  FlightRecorder recorder(options_);
  for (int n = 1; n < 20; ++n) {
    auto sample = event(n * 20);
    sample.sdk_observed = true;
    sample.raw_sdk.main_code = 1;
    sample.raw_sdk.sub_code = 149;
    ASSERT_TRUE(recorder.record(sample));
    recorder.pollForTest(sample.stamp.mono_ns);
  }
  EXPECT_EQ(traces(), 0u);
  EXPECT_EQ(recorder.stats().incidents_started, 0u);
  auto edge = event(400);
  edge.trigger_reasons = CONTROLLER_FAULT | STALE_STATE | MISSING_STATE;
  recorder.record(edge);
  recorder.pollForTest(501 * kMs);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  EXPECT_NE(trace().find("MISSING_STATE"), std::string::npos);
  EXPECT_NE(trace().find("STALE_STATE"), std::string::npos);
}

TEST_F(FlightRecorderTest, PersistentFastRpcErrorIsAnEdgeAndSlowCallsRemainIndependent) {
  FlightRecorder recorder(options_);
  const auto rpc = [&](int ms, int rc, int duration_ms = 1, TraceRpc call = TraceRpc::SERVO) {
      auto sample = event(ms, TraceKind::RPC_END);
      sample.rpc = call;
      sample.rc = rc;
      sample.rpc_begin_mono_ns = (ms - duration_ms) * kMs;
      recorder.record(sample);
      recorder.pollForTest(ms * kMs);
    };
  rpc(100, 14);
  recorder.pollForTest(201 * kMs);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  rpc(350, 14);
  rpc(500, 14);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  EXPECT_EQ(traces(), 1u);
  rpc(600, 0);
  rpc(601, 14);
  recorder.pollForTest(702 * kMs);
  EXPECT_EQ(recorder.stats().incidents_started, 2u);
  // The same error code can still have a distinct slow-call incident.
  rpc(900, 14, 50);
  recorder.pollForTest(1001 * kMs);
  EXPECT_EQ(recorder.stats().incidents_started, 3u);
  EXPECT_NE(trace().find("\"trigger_reasons\":[\"RPC_SLOW\"]"), std::string::npos);
  rpc(1200, 14, 1, TraceRpc::STATE);
  recorder.pollForTest(1301 * kMs);
  EXPECT_EQ(recorder.stats().incidents_started, 4u);
  rpc(1500, 15, 1, TraceRpc::STATE);
  recorder.pollForTest(1601 * kMs);
  rpc(1800, 15, 1, TraceRpc::STATE);
  EXPECT_EQ(recorder.stats().incidents_started, 5u);
  recorder.shutdown();
  recorder.start();
  auto after_restart = event(2100, TraceKind::RPC_END);
  after_restart.rpc = TraceRpc::STATE;
  after_restart.rc = 15;
  after_restart.rpc_begin_mono_ns = 2099 * kMs;
  recorder.record(after_restart);
  recorder.shutdown();
  EXPECT_EQ(recorder.stats().incidents_started, 6u);
}

TEST_F(FlightRecorderTest, TraceIoFailureIsVisibleInMetadataAndCounters) {
  options_.output_directory = directory_ + "/occupied";
  std::ofstream(options_.output_directory) << "fixture blocks directory creation";
  FlightRecorder recorder(options_);
  auto sample = event(100, TraceKind::RPC_END);
  sample.rpc = TraceRpc::STATE;
  sample.rc = 14;
  recorder.record(sample);
  recorder.pollForTest(201 * kMs);
  EXPECT_GT(recorder.stats().io_errors, 0u);
  EXPECT_NE(metadata().find("\"phase\":\"complete\""), std::string::npos);
  EXPECT_EQ(metadata().find("\"io_errors\":0"), std::string::npos);
}

TEST_F(FlightRecorderTest, PersistentTraceRetentionIncludesPreviousProcesses) {
  std::ofstream(directory_ + "/hardware-old-process-1.jsonl") << "older fixture";
  std::ofstream(directory_ + "/hardware-old-process-2.jsonl") << "older fixture";
  std::ofstream(directory_ + "/hardware-old-process-3.jsonl.tmp") << "interrupted finalization";
  std::ofstream(directory_ + "/host-companion.jsonl") << "must retain";
  FlightRecorder recorder(options_);
  auto sample = event(100);
  sample.trigger_reasons = SAFETY_FAULT;
  recorder.record(sample);
  recorder.pollForTest(201 * kMs);
  size_t hardware_files = 0;
  for (const auto & entry : std::filesystem::directory_iterator(directory_)) {
    if (entry.path().filename().string().rfind("hardware-", 0) == 0 &&
      entry.path().extension() == ".jsonl") {++hardware_files;}
  }
  EXPECT_LE(hardware_files, options_.max_trace_files);
  EXPECT_FALSE(std::filesystem::exists(directory_ + "/hardware-old-process-3.jsonl.tmp"));
  EXPECT_EQ(read(directory_ + "/host-companion.jsonl"), "must retain");
}

TEST_F(FlightRecorderTest, ConcurrentBackgroundRequestsAreBoundedAndSingleEpisode) {
  FlightRecorder recorder(options_);
  std::atomic<int> callbacks{0};
  const auto stamp = clockNow();
  std::array<std::thread, 4> requesters;
  for (auto & requester : requesters) {
    requester = std::thread(
      [&] {
        for (size_t n = 0; n < 50; ++n) {
          recorder.requestIncident(ACTION_ABORT, stamp);
        }
      });
  }
  for (auto & requester : requesters) {
    requester.join();
  }
  recorder.start([&](const IncidentInfo &) {++callbacks;});
  recorder.shutdown();
  EXPECT_EQ(callbacks.load(), 1);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  EXPECT_GT(recorder.stats().incident_dropped, 0u);
}

TEST_F(FlightRecorderTest, LiveProducerAndBackgroundAbortDrainBeforeShutdown) {
  options_.post_window_ns = 1000000000LL;
  options_.queue_capacity = 512;
  options_.incident_capacity = 1024;
  FlightRecorder recorder(options_);
  std::atomic<int> callbacks{0};
  size_t accepted = 0;
  int32_t producer_tid = 0;
  recorder.start([&](const IncidentInfo &) {++callbacks;});
  std::thread producer([&] {
      producer_tid = controlThreadId();
      for (size_t index = 0; index < 2000; ++index) {
        auto sample = event(100);
        sample.stamp = clockNow();
        sample.control_tid = producer_tid;
        sample.cycle_index = index;
        if (recorder.record(sample)) {++accepted;}
      }
    });
  std::thread background([&] {recorder.requestIncident(ACTION_ABORT);});
  producer.join();
  background.join();
  recorder.shutdown();
  EXPECT_EQ(recorder.stats().queue_dropped, 2000 - accepted);
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  EXPECT_EQ(recorder.stats().incidents_completed, 1u);
  EXPECT_EQ(callbacks.load(), 1);
  EXPECT_NE(
    metadata().find("\"control_tid\":" + std::to_string(producer_tid)),
    std::string::npos);
  EXPECT_NE(trace().find("\"partial\":true"), std::string::npos);
}

TEST_F(FlightRecorderTest, QueueAndIncidentCapacityDropsAreExplicitAndFilesBounded) {
  options_.queue_capacity = 3;
  options_.history_capacity = 3;
  options_.incident_capacity = 3;
  FlightRecorder recorder(options_);
  for (int n = 0; n < 10; ++n) {
    recorder.record(event(100 + n));
  }
  EXPECT_GT(recorder.stats().queue_dropped, 0u);
  recorder.pollForTest(110 * kMs);
  for (int n = 0; n < 5; ++n) {
    recorder.record(event(120 + n));
    recorder.pollForTest((120 + n) * kMs);
  }
  recorder.pollForTest(220 * kMs);
  EXPECT_GT(recorder.stats().incident_dropped, 0u);
  EXPECT_NE(trace().find("\"queue_dropped\":"), std::string::npos);
  for (int n = 1; n < 5; ++n) {
    recorder.requestIncident(SAFETY_FAULT, event(n * 1000).stamp);
    recorder.pollForTest(n * 1000 * kMs);
    recorder.pollForTest((n * 1000 + 101) * kMs);
  }
  EXPECT_LE(traces(), options_.max_trace_files);
}

TEST_F(FlightRecorderTest, ShutdownFlushesPartialCaptureOnBackgroundThread) {
  FlightRecorder recorder(options_);
  std::atomic<int> callbacks{0};
  std::thread::id callback_thread;
  const auto caller = std::this_thread::get_id();
  recorder.start(
    [&](const IncidentInfo & info) {
      callback_thread = std::this_thread::get_id();
      EXPECT_GT(info.trigger_mono_ns, 0);
      callbacks.fetch_add(1);
    });
  auto sample = event(100);
  sample.stamp = clockNow();
  sample.trigger_reasons = STALE_STATE;
  recorder.record(sample);
  recorder.shutdown();
  EXPECT_EQ(callbacks.load(), 1);
  EXPECT_NE(callback_thread, caller);
  EXPECT_EQ(recorder.stats().incidents_completed, 1u);
  EXPECT_NE(trace().find("\"partial\":true"), std::string::npos);
  EXPECT_NE(metadata().find("\"phase\":\"stopped\""), std::string::npos);
  const auto metadata_file = directory_ + "/hardware-" + std::to_string(getpid()) + ".json";
  const auto modified = std::filesystem::last_write_time(metadata_file);
  recorder.shutdown();
  EXPECT_EQ(std::filesystem::last_write_time(metadata_file), modified);
}

TEST_F(FlightRecorderTest, ReactivationArmsKnownIdentityAndStartsNewCapture) {
  FlightRecorder recorder(options_);
  auto sample = event(100);
  sample.stamp = clockNow();
  sample.trigger_reasons = SAFETY_FAULT;
  recorder.record(sample);
  recorder.pollForTest(sample.stamp.mono_ns);
  recorder.shutdown();
  EXPECT_EQ(recorder.stats().incidents_started, 1u);
  std::string previous_trace;
  for (const auto & entry : std::filesystem::directory_iterator(directory_)) {
    if (entry.path().extension() == ".jsonl") {previous_trace = entry.path().string();}
  }
  recorder.start();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  std::string armed;
  do {
    armed = metadata();
    if (armed.find("\"phase\":\"armed\"") != std::string::npos) {break;}
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  EXPECT_NE(armed.find("\"phase\":\"armed\""), std::string::npos);
  EXPECT_NE(armed.find(std::filesystem::path(previous_trace).stem().string()), std::string::npos);
  EXPECT_NE(armed.find("\"trace_path\":\"" + previous_trace + "\""), std::string::npos);
  EXPECT_NE(
    armed.find("\"trigger_mono_ns\":" + std::to_string(sample.stamp.mono_ns)),
    std::string::npos);
  EXPECT_NE(
    armed.find("\"trigger_unix_ns\":" + std::to_string(sample.stamp.unix_ns)),
    std::string::npos);
  EXPECT_NE(
    armed.find(
      "\"post_until_mono_ns\":" +
      std::to_string(sample.stamp.mono_ns + options_.post_window_ns)), std::string::npos);
  auto next_control_thread = sample;
  next_control_thread.control_tid = 4321;
  next_control_thread.trigger_reasons = TRIGGER_NONE;
  recorder.record(next_control_thread);
  const auto identity_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  do {
    armed = metadata();
    if (armed.find("\"control_tid\":4321") != std::string::npos) {break;}
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < identity_deadline);
  EXPECT_NE(armed.find("\"phase\":\"armed\""), std::string::npos);
  EXPECT_NE(armed.find("\"control_tid\":4321"), std::string::npos);
  EXPECT_NE(armed.find(std::filesystem::path(previous_trace).stem().string()), std::string::npos);
  // The retained handoff is not an eligible old correlation episode after re-arm.
  recorder.requestIncident(ACTION_ABORT, {sample.stamp.mono_ns + 1, sample.stamp.unix_ns + 1});
  recorder.shutdown();
  EXPECT_EQ(recorder.stats().incidents_started, 2u);
  EXPECT_EQ(recorder.stats().incidents_completed, 2u);
}

TEST(FlightRecorderProcessIdentityTest, ParsesField22AfterComplexCommAndSignedFields) {
  const auto stat = [](const std::string & comm, const std::string & start_ticks) {
      std::string row = "123 (" + comm + ") S";
      for (int index = 4; index < 22; ++index) {
        row += " " + std::to_string(-index);
      }
      return row + " " + start_ticks + " 999 888\n";
    };
  EXPECT_EQ(parseProcessStartTicks(stat("driver", "123456789")), 123456789u);
  EXPECT_EQ(parseProcessStartTicks(stat("arm (control)) thread", "987654321")), 987654321u);
  EXPECT_EQ(parseProcessStartTicks(stat("odd ) name (with\nnewline", "42")), 42u);
  EXPECT_EQ(
    parseProcessStartTicks(stat("driver", "18446744073709551615")),
    std::numeric_limits<uint64_t>::max());
  EXPECT_EQ(parseProcessStartTicks(stat("driver", "18446744073709551616")), 0u);
  EXPECT_EQ(parseProcessStartTicks(stat("driver", "-5")), 0u);
  EXPECT_EQ(parseProcessStartTicks(stat("driver", "123x")), 0u);
  EXPECT_EQ(parseProcessStartTicks("123 (missing fields) S 1 2"), 0u);
  EXPECT_EQ(parseProcessStartTicks("malformed comm"), 0u);
}

}  // namespace
}  // namespace fairino_hardware
