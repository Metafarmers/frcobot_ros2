#ifndef FAIRINO_HARDWARE_FLIGHT_RECORDER_HPP_
#define FAIRINO_HARDWARE_FLIGHT_RECORDER_HPP_

#include "libfairino/include/robot_types.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>

namespace fairino_hardware
{

struct ClockStamp
{
  int64_t mono_ns = 0;
  int64_t unix_ns = 0;
};

ClockStamp clockNow() noexcept;
int32_t controlThreadId() noexcept;
// Pure parser for /proc/<pid>/stat field 22. Unknown/malformed identity returns 0.
uint64_t parseProcessStartTicks(const std::string & stat) noexcept;

enum class TraceKind : uint8_t
{
  READ_BEGIN, READ_END, WRITE_BEGIN, WRITE_END, RPC_BEGIN, RPC_END, STATE,
  EXTERNAL_TRIGGER
};
enum class TraceRpc : uint8_t
{
  NONE, STATE, SERVO, STOP, END, CLEAR, START, DRAG, QUEUE, RESET
};

enum TriggerReason : uint64_t
{
  TRIGGER_NONE = 0,
  RPC_SLOW = 1ULL << 0,
  RPC_ERROR = 1ULL << 1,
  CYCLE_GAP = 1ULL << 2,
  CATCH_UP = 1ULL << 3,
  STALE_STATE = 1ULL << 4,
  CONTROLLER_FAULT = 1ULL << 5,
  SAFETY_FAULT = 1ULL << 6,
  ACTION_ABORT = 1ULL << 7,
  MISSING_STATE = 1ULL << 8,
  NONFINITE_STATE = 1ULL << 9,
  DROP = 1ULL << 10
};

// Single control-thread producer. Values are observations of the SDK's cached
// state, never socket receive timestamps. The installed SDK ABI is preserved.
struct TraceEvent
{
  ClockStamp stamp{};
  TraceKind kind = TraceKind::STATE;
  TraceRpc rpc = TraceRpc::NONE;
  int32_t control_tid = 0;
  int32_t rc = 0;
  uint64_t cycle_index = 0;
  uint64_t write_index = 0;
  int64_t rpc_begin_mono_ns = 0;
  double nominal_period_sec = 0;
  double command_period_sec = 0;
  double reported_period_sec = 0;
  double actual_period_sec = 0;
  double frame_age_sec = 0;
  uint64_t flags = 0;
  uint64_t trigger_reasons = TRIGGER_NONE;
  bool sdk_observed = false;
  bool state_valid = false;
  int64_t sdk_observed_mono_ns = 0;
  ROBOT_STATE_PKG raw_sdk{};
  double command_rad[6]{};
  double attempted_target_rad[6]{};
  double successful_target_rad[6]{};
  bool command_valid = false;
  bool attempted_target_valid = false;
  bool successful_target_valid = false;
  char goal_uuid[37]{};
  int64_t goal_accepted_ros_ns = 0;
};
static_assert(
  std::is_trivially_copyable<TraceEvent>::value,
  "The RT recorder queue requires fixed-size copyable events");

struct FlightRecorderOptions
{
  // Empty resolves MF_FAIRINO_DIAGNOSTICS_DIR, else HOME/.ros/fairino_diagnostics
  // once during construction, outside the control thread.
  std::string output_directory;
  std::string metadata_directory = "/tmp/mf_fairino_flight_recorder";
  int64_t pre_window_ns = 60000000000LL;
  int64_t post_window_ns = 60000000000LL;
  int64_t correlation_window_ns = 5000000000LL;
  int64_t slow_threshold_ns = 50000000LL;
  size_t queue_capacity = 8192;
  size_t history_capacity = 32768;
  size_t incident_capacity = 65536;
  size_t max_trace_files = 4;
};

struct IncidentInfo
{
  std::string incident_id;
  int32_t pid = 0;
  int32_t control_tid = 0;
  int64_t trigger_mono_ns = 0;
  int64_t trigger_unix_ns = 0;
  int64_t post_until_mono_ns = 0;
  std::string trace_path;
};

struct FlightRecorderStats
{
  uint64_t queue_dropped = 0;
  uint64_t history_evicted = 0;
  uint64_t incident_dropped = 0;
  uint64_t correlated_triggers = 0;
  uint64_t io_errors = 0;
  uint64_t incidents_started = 0;
  uint64_t incidents_completed = 0;
};

class FlightRecorder
{
public:
  using IncidentCallback = std::function<void (const IncidentInfo &)>;
  explicit FlightRecorder(FlightRecorderOptions options = {});
  ~FlightRecorder();
  FlightRecorder(const FlightRecorder &) = delete;
  FlightRecorder & operator=(const FlightRecorder &) = delete;

  // Activation/background only: creates the dedicated writer outside RT.
  void start(IncidentCallback callback = {});
  // RT: bounded SPSC copy and lock-free atomics; no wakeup, disk or formatting.
  bool record(const TraceEvent & event) noexcept;
  // Background callback only. Multiple callbacks may request concurrently.
  void requestIncident(
    uint64_t reasons, ClockStamp stamp = clockNow(),
    const char * goal_uuid = nullptr, int64_t goal_accepted_ros_ns = 0) noexcept;
  // Call AFTER physical SDK stop/close. Drains queued events and joins the writer.
  // Instances never started or explicitly polled discard events without disk I/O.
  void shutdown();
  FlightRecorderStats stats() const noexcept;
  // Deterministic offline seam, exclusive with start()/the writer thread.
  void pollForTest(int64_t now_mono_ns);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace fairino_hardware
#endif  // FAIRINO_HARDWARE_FLIGHT_RECORDER_HPP_
