#include "fairino_hardware/flight_recorder.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <locale>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace fairino_hardware
{
namespace
{
constexpr int64_t kNsPerSecond = 1000000000LL;
static_assert(
  std::atomic<uint64_t>::is_always_lock_free,
  "Flight recorder RT counters must be lock free");
static_assert(
  std::atomic<size_t>::is_always_lock_free,
  "Flight recorder RT queue indices must be lock free");

int64_t nanoseconds(const timespec & value) noexcept
{
  return value.tv_sec * kNsPerSecond + value.tv_nsec;
}

uint64_t readProcessStartTicks()
{
  std::ifstream input("/proc/self/stat");
  if (!input) {return 0;}
  const std::string stat{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  return parseProcessStartTicks(stat);
}

const char * kindName(TraceKind kind)
{
  switch (kind) {
    case TraceKind::READ_BEGIN: return "READ_BEGIN";
    case TraceKind::READ_END: return "READ_END";
    case TraceKind::WRITE_BEGIN: return "WRITE_BEGIN";
    case TraceKind::WRITE_END: return "WRITE_END";
    case TraceKind::RPC_BEGIN: return "RPC_BEGIN";
    case TraceKind::RPC_END: return "RPC_END";
    case TraceKind::STATE: return "STATE";
    case TraceKind::EXTERNAL_TRIGGER: return "EXTERNAL_TRIGGER";
  }
  return "UNKNOWN";
}
const char * rpcName(TraceRpc rpc)
{
  switch (rpc) {
    case TraceRpc::NONE: return "NONE";
    case TraceRpc::STATE: return "STATE";
    case TraceRpc::SERVO: return "SERVO";
    case TraceRpc::STOP: return "STOP";
    case TraceRpc::END: return "END";
    case TraceRpc::CLEAR: return "CLEAR";
    case TraceRpc::START: return "START";
    case TraceRpc::DRAG: return "DRAG";
    case TraceRpc::QUEUE: return "QUEUE";
    case TraceRpc::RESET: return "RESET";
  }
  return "UNKNOWN";
}

void jsonString(std::ostream & out, const std::string & value)
{
  out << '"';
  for (const unsigned char ch : value) {
    if (ch == '"' || ch == '\\') {out << '\\' << ch;} else if (ch < 32) {
      out << "\\u00" << std::hex << std::setw(2)
          << std::setfill('0') << static_cast<unsigned>(ch) << std::dec;
    } else {out << ch;}
  }
  out << '"';
}

template<class T>
void number(std::ostream & out, T value)
{
  // Nonfinite telemetry must not turn an otherwise useful incident into invalid JSON.
  if (std::is_floating_point<T>::value && !std::isfinite(static_cast<double>(value))) {
    out << "null";
  } else {
    out << +value;
  }
}
template<class T>
void field(std::ostream & out, const char * name, T value)
{
  out << ",\"" << name << "\":";
  number(out, value);
}
template<class T>
void arrayField(std::ostream & out, const char * name, const T * values, size_t count)
{
  out << ",\"" << name << "\":[";
  // SDK state is packed to one byte. Never bind a double/uint16 reference to
  // its arrays: copy the bytes into an aligned scalar before decoding.
  const auto * bytes = reinterpret_cast<const unsigned char *>(values);
  for (size_t index = 0; index < count; ++index) {
    if (index) {out << ',';}
    T value;
    std::memcpy(&value, bytes + index * sizeof(T), sizeof(T));
    number(out, value);
  }
  out << ']';
}

void reasons(std::ostream & out, uint64_t value)
{
  out << '[';
  bool first = true;
  const std::pair<uint64_t, const char *> names[] = {
    {RPC_SLOW, "RPC_SLOW"}, {RPC_ERROR, "RPC_ERROR"}, {CYCLE_GAP, "CYCLE_GAP"},
    {CATCH_UP, "CATCH_UP"}, {STALE_STATE, "STALE_STATE"},
    {CONTROLLER_FAULT, "CONTROLLER_FAULT"}, {SAFETY_FAULT, "SAFETY_FAULT"},
    {ACTION_ABORT, "ACTION_ABORT"}, {MISSING_STATE, "MISSING_STATE"},
    {NONFINITE_STATE, "NONFINITE_STATE"}, {DROP, "DROP"}};
  for (const auto & name : names) {
    if (!(value & name.first)) {continue;}
    if (!first) {out << ',';}
    jsonString(out, name.second);
    first = false;
  }
  out << ']';
}

void sdkState(std::ostream & out, const ROBOT_STATE_PKG & state)
{
  out << ",\"raw_sdk_size\":" << sizeof(state) << ",\"raw_sdk_hex\":\"";
  constexpr char hex[] = "0123456789abcdef";
  const auto * bytes = reinterpret_cast<const unsigned char *>(&state);
  for (size_t index = 0; index < sizeof(state); ++index) {
    out << hex[bytes[index] >> 4] << hex[bytes[index] & 15];
  }
  out << "\",\"sdk_state\":{\"frame_head\":" << state.frame_head;
#define SDK_FIELD(name) field(out, #name, state.name)
#define SDK_ARRAY(name) arrayField( \
    out, #name, state.name, \
    sizeof(state.name) / sizeof(state.name[0]))
  SDK_FIELD(frame_cnt); SDK_FIELD(data_len); SDK_FIELD(program_state);
  SDK_FIELD(robot_state); SDK_FIELD(main_code); SDK_FIELD(sub_code); SDK_FIELD(robot_mode);
  SDK_ARRAY(jt_cur_pos); SDK_ARRAY(tl_cur_pos); SDK_ARRAY(flange_cur_pos);
  SDK_ARRAY(actual_qd); SDK_ARRAY(actual_qdd); SDK_ARRAY(target_TCP_CmpSpeed);
  SDK_ARRAY(target_TCP_Speed); SDK_ARRAY(actual_TCP_CmpSpeed); SDK_ARRAY(actual_TCP_Speed);
  SDK_ARRAY(jt_cur_tor); SDK_FIELD(tool); SDK_FIELD(user);
  SDK_FIELD(cl_dgt_output_h); SDK_FIELD(cl_dgt_output_l); SDK_FIELD(tl_dgt_output_l);
  SDK_FIELD(cl_dgt_input_h); SDK_FIELD(cl_dgt_input_l); SDK_FIELD(tl_dgt_input_l);
  SDK_ARRAY(cl_analog_input); SDK_FIELD(tl_anglog_input);
  SDK_ARRAY(ft_sensor_raw_data); SDK_ARRAY(ft_sensor_data); SDK_FIELD(ft_sensor_active);
  SDK_FIELD(EmergencyStop); SDK_FIELD(motion_done); SDK_FIELD(gripper_motiondone);
  SDK_FIELD(mc_queue_len); SDK_FIELD(collisionState); SDK_FIELD(trajectory_pnum);
  SDK_FIELD(safety_stop0_state); SDK_FIELD(safety_stop1_state); SDK_FIELD(gripper_fault_id);
  SDK_FIELD(gripper_fault); SDK_FIELD(gripper_active); SDK_FIELD(gripper_position);
  SDK_FIELD(gripper_speed); SDK_FIELD(gripper_current); SDK_FIELD(gripper_temp);
  SDK_FIELD(gripper_voltage);
  out << ",\"aux_state\":{\"servoId\":" << +state.aux_state.servoId;
  field(out, "servoErrCode", state.aux_state.servoErrCode);
  field(out, "servoState", state.aux_state.servoState);
  field(out, "servoPos", state.aux_state.servoPos);
  field(out, "servoVel", state.aux_state.servoVel);
  field(out, "servoTorque", state.aux_state.servoTorque);
  out << "},\"extAxisStatus\":[";
  for (size_t index = 0; index < 4; ++index) {
    if (index) {out << ',';}
    const auto & axis = state.extAxisStatus[index];
    out << "{\"index\":" << index;
    field(out, "pos", axis.pos); field(out, "vel", axis.vel);
    field(out, "errorCode", axis.errorCode); field(out, "ready", axis.ready);
    field(out, "inPos", axis.inPos); field(out, "alarm", axis.alarm);
    field(out, "flerr", axis.flerr); field(out, "nlimit", axis.nlimit);
    field(out, "pLimit", axis.pLimit); field(out, "mdbsOffLine", axis.mdbsOffLine);
    field(out, "mdbsTimeout", axis.mdbsTimeout); field(out, "homingStatus", axis.homingStatus);
    out << '}';
  }
  out << ']';
  SDK_ARRAY(extDIState); SDK_ARRAY(extDOState); SDK_ARRAY(extAIState); SDK_ARRAY(extAOState);
  SDK_FIELD(rbtEnableState); SDK_ARRAY(jointDriverTorque); SDK_ARRAY(jointDriverTemperature);
  out << ",\"robotTime\":{\"year\":" << state.robotTime.year;
  field(out, "mouth", state.robotTime.mouth); field(out, "day", state.robotTime.day);
  field(out, "hour", state.robotTime.hour); field(out, "minute", state.robotTime.minute);
  field(out, "second", state.robotTime.second);
  field(out, "millisecond", state.robotTime.millisecond);
  out << '}';
  SDK_FIELD(softwareUpgradeState); SDK_FIELD(endLuaErrCode); SDK_ARRAY(cl_analog_output);
  SDK_FIELD(tl_analog_output); SDK_FIELD(gripperRotNum); SDK_FIELD(gripperRotSpeed);
  SDK_FIELD(gripperRotTorque);
  out << ",\"weldingBreakOffState\":{\"breakOffState\":"
      << +state.weldingBreakOffState.breakOffState;
  field(out, "weldArcState", state.weldingBreakOffState.weldArcState);
  out << '}';
  SDK_ARRAY(jt_tgt_tor); SDK_FIELD(smartToolState); SDK_FIELD(wideVoltageCtrlBoxTemp);
  SDK_FIELD(wideVoltageCtrlBoxFanCurrent); SDK_ARRAY(toolCoord); SDK_ARRAY(wobjCoord);
  SDK_ARRAY(extoolCoord); SDK_ARRAY(exAxisCoord); SDK_FIELD(load); SDK_ARRAY(loadCog);
  SDK_ARRAY(lastServoTarget); SDK_FIELD(servoJCmdNum); SDK_FIELD(check_sum);
#undef SDK_FIELD
#undef SDK_ARRAY
  out << '}';
}

void eventJson(std::ostream & out, const TraceEvent & event, const IncidentInfo & info)
{
  out << "{\"schema_version\":1,\"type\":\"event\",\"incident_id\":";
  jsonString(out, info.incident_id);
  field(out, "pid", info.pid);
  field(out, "control_tid", event.control_tid);
  field(out, "mono_ns", event.stamp.mono_ns);
  field(out, "unix_ns", event.stamp.unix_ns);
  out << ",\"kind\":"; jsonString(out, kindName(event.kind));
  out << ",\"rpc\":"; jsonString(out, rpcName(event.rpc));
  field(out, "rc", event.rc);
  field(out, "cycle_index", event.cycle_index); field(out, "write_index", event.write_index);
  field(out, "rpc_begin_mono_ns", event.rpc_begin_mono_ns);
  field(
    out, "rpc_duration_ns", event.kind == TraceKind::RPC_END &&
    event.rpc_begin_mono_ns > 0 ? event.stamp.mono_ns - event.rpc_begin_mono_ns : 0);
  field(out, "nominal_period_sec", event.nominal_period_sec);
  field(out, "command_period_sec", event.command_period_sec);
  field(out, "reported_period_sec", event.reported_period_sec);
  field(out, "actual_period_sec", event.actual_period_sec);
  field(out, "frame_age_sec", event.frame_age_sec); field(out, "flags", event.flags);
  field(out, "trigger_reason_bits", event.trigger_reasons);
  out << ",\"trigger_reasons\":"; reasons(out, event.trigger_reasons);
  out << ",\"sdk_observed\":" << (event.sdk_observed ? "true" : "false")
      << ",\"state_valid\":" << (event.state_valid ? "true" : "false");
  field(out, "sdk_observed_mono_ns", event.sdk_observed_mono_ns);
  out << ",\"state_timestamp_semantics\":\"SDK_cache_observation_not_socket_receive\"";
  if (event.sdk_observed) {sdkState(out, event.raw_sdk);}
  out << ",\"command_valid\":" << (event.command_valid ? "true" : "false")
      << ",\"attempted_target_valid\":" << (event.attempted_target_valid ? "true" : "false")
      << ",\"successful_target_valid\":" << (event.successful_target_valid ? "true" : "false");
  arrayField(out, "command_rad", event.command_rad, 6);
  arrayField(out, "attempted_target_rad", event.attempted_target_rad, 6);
  arrayField(out, "successful_target_rad", event.successful_target_rad, 6);
  out << ",\"goal_uuid\":";
  jsonString(out, std::string(event.goal_uuid, strnlen(event.goal_uuid, sizeof(event.goal_uuid))));
  field(out, "goal_accepted_ros_ns", event.goal_accepted_ros_ns);
  out << "}\n";
}
}  // namespace

ClockStamp clockNow() noexcept
{
  timespec mono{}, realtime{};
  clock_gettime(CLOCK_MONOTONIC, &mono);
  clock_gettime(CLOCK_REALTIME, &realtime);
  return {nanoseconds(mono), nanoseconds(realtime)};
}
int32_t controlThreadId() noexcept
{
  static thread_local const int32_t tid = static_cast<int32_t>(syscall(SYS_gettid));
  return tid;
}

uint64_t parseProcessStartTicks(const std::string & stat) noexcept
{
  // comm (field 2) may contain spaces and ')' characters. The closing delimiter
  // is the last ')'; every following field is a whitespace-separated token.
  size_t position = stat.rfind(')');
  if (position == std::string::npos) {return 0;}
  ++position;
  for (unsigned field_index = 3; field_index <= 22; ++field_index) {
    while (position < stat.size() &&
      std::isspace(static_cast<unsigned char>(stat[position]))) {++position;}
    const size_t begin = position;
    while (position < stat.size() &&
      !std::isspace(static_cast<unsigned char>(stat[position]))) {++position;}
    if (position == begin) {return 0;}
    if (field_index != 22) {continue;}
    uint64_t value = 0;
    for (size_t digit_index = begin; digit_index < position; ++digit_index) {
      const unsigned char digit = stat[digit_index];
      if (digit < '0' || digit > '9') {return 0;}
      const uint64_t next = digit - '0';
      if (value > (std::numeric_limits<uint64_t>::max() - next) / 10) {return 0;}
      value = value * 10 + next;
    }
    return value;
  }
  return 0;
}

class FlightRecorder::Impl
{
public:
  explicit Impl(FlightRecorderOptions configuration)
  : options(std::move(configuration)), pid(static_cast<int32_t>(getpid())),
    process_start_ticks(readProcessStartTicks()),
    queue(std::max<size_t>(options.queue_capacity, 1)),
    history(std::max<size_t>(options.history_capacity, 1))
  {
    if (options.pre_window_ns < 0 || options.post_window_ns < 0 ||
      options.correlation_window_ns < 0 || options.slow_threshold_ns <= 0 ||
      !options.incident_capacity || !options.max_trace_files)
    {
      throw std::invalid_argument("Invalid bounded flight recorder options");
    }
    incident.reserve(options.incident_capacity);
    if (options.output_directory.empty()) {
      const char * configured = std::getenv("MF_FAIRINO_DIAGNOSTICS_DIR");
      const char * home = std::getenv("HOME");
      if (configured && *configured) {
        options.output_directory = configured;
      } else if (home && *home) {
        options.output_directory =
          (std::filesystem::path(home) / ".ros/fairino_diagnostics").string();
      } else {
        throw std::invalid_argument("Flight recorder persistent output directory is unavailable");
      }
    }
    metadata_path = (std::filesystem::path(options.metadata_directory) /
      ("hardware-" + std::to_string(pid) + ".json")).string();
  }

  FlightRecorderStats snapshot() const noexcept
  {
    return {queue_dropped.load(), history_evicted.load(), incident_dropped.load(),
      correlated_triggers.load(), io_errors.load(), incidents_started.load(),
      incidents_completed.load()};
  }

  void statsJson(std::ostream & out) const
  {
    const auto counters = snapshot();
    field(out, "queue_dropped", counters.queue_dropped);
    field(out, "history_evicted", counters.history_evicted);
    field(out, "incident_dropped", counters.incident_dropped);
    field(out, "correlated_triggers", counters.correlated_triggers);
    field(out, "io_errors", counters.io_errors);
    field(out, "incidents_started", counters.incidents_started);
    field(out, "incidents_completed", counters.incidents_completed);
  }

  bool prepareDirectory(const std::string & directory)
  {
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error) {++io_errors; return false;}
    return true;
  }

  void metadata(const char * phase)
  {
    if (!prepareDirectory(options.metadata_directory)) {return;}
    const auto temporary = metadata_path + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    out.imbue(std::locale::classic());
    out << "{\"schema_version\":1,\"pid\":" << pid;
    field(out, "process_start_ticks", process_start_ticks);
    field(out, "control_tid", identity_tid);
    out << ",\"phase\":"; jsonString(out, phase);
    out << ",\"incident_id\":"; jsonString(out, info.incident_id);
    field(out, "trigger_mono_ns", info.trigger_mono_ns);
    field(out, "trigger_unix_ns", info.trigger_unix_ns);
    field(out, "post_until_mono_ns", info.post_until_mono_ns);
    out << ",\"trace_path\":"; jsonString(out, info.trace_path);
    statsJson(out);
    out << "}\n";
    out.close();
    if (!out) {++io_errors; return;}
    std::error_code error;
    std::filesystem::rename(temporary, metadata_path, error);
    if (error) {++io_errors; return;}
    identity_written = true;
  }

  void header(std::ostream & out)
  {
    out << "{\"schema_version\":1,\"type\":\"incident\",\"incident_id\":";
    jsonString(out, info.incident_id);
    field(out, "pid", info.pid); field(out, "control_tid", info.control_tid);
    field(out, "process_start_ticks", process_start_ticks);
    field(out, "trigger_mono_ns", info.trigger_mono_ns);
    field(out, "trigger_unix_ns", info.trigger_unix_ns);
    field(out, "post_until_mono_ns", info.post_until_mono_ns);
    field(out, "pre_window_ns", options.pre_window_ns);
    out << ",\"trigger_reasons\":"; reasons(out, episode_reasons);
    out << ",\"sdk_abi\":\"installed_ROBOT_STATE_PKG\",\"raw_sdk_size\":"
        << sizeof(ROBOT_STATE_PKG) << "}\n";
  }

  void retainFile(const std::string & path)
  {
    // Include older process/reboot artifacts, otherwise a per-process limit
    // would allow the persistent diagnostics directory to grow indefinitely.
    std::error_code error;
    using File = std::pair<std::filesystem::file_time_type, std::string>;
    std::vector<File> retained;
    retained.reserve(options.max_trace_files);
    std::filesystem::directory_iterator entry(options.output_directory, error), end;
    if (error) {++io_errors; return;}
    while (entry != end) {
      const auto candidate = entry->path();
      const std::string name = candidate.filename().string();
      if (name.rfind("hardware-", 0) == 0 && name.size() >= 10 &&
        name.compare(name.size() - 10, 10, ".jsonl.tmp") == 0)
      {
        // Finalization can be interrupted by a process crash/reboot. Remove
        // that abandoned temporary before applying the persistent trace cap.
        std::filesystem::remove(candidate, error);
        if (error) {++io_errors; error.clear();}
      }
      if (candidate.extension() == ".jsonl" && name.rfind("hardware-", 0) == 0 &&
        candidate.string() != path && entry->is_regular_file(error))
      {
        const auto modified = entry->last_write_time(error);
        if (error) {++io_errors; error.clear();} else {
          retained.emplace_back(modified, candidate.string());
          std::sort(retained.begin(), retained.end());
          // The newly-created active incident owns one of the retained slots.
          if (retained.size() >= options.max_trace_files) {
            std::filesystem::remove(retained.front().second, error);
            if (error) {++io_errors; error.clear();}
            retained.erase(retained.begin());
          }
        }
      }
      entry.increment(error);
      if (error) {++io_errors; break;}
    }
  }

  void appendIncident(const TraceEvent & event)
  {
    if (incident.size() < options.incident_capacity) {incident.push_back(event);} else {
      ++incident_dropped;
    }
  }

  void begin(const TraceEvent & event)
  {
    info.incident_id = "hardware-" + std::to_string(pid) + "-" +
      std::to_string(event.stamp.mono_ns) + "-" + std::to_string(++episode_sequence);
    info.pid = pid;
    info.control_tid = identity_tid;
    info.trigger_mono_ns = event.stamp.mono_ns;
    info.trigger_unix_ns = event.stamp.unix_ns;
    info.post_until_mono_ns = event.stamp.mono_ns + options.post_window_ns;
    info.trace_path = (std::filesystem::path(options.output_directory) /
      (info.incident_id + ".jsonl")).string();
    episode_reasons = event.trigger_reasons;
    incident.clear();
    for (size_t index = 0; index < history_size; ++index) {
      const auto & previous = history[(history_start + index) % history.size()];
      if (previous.stamp.mono_ns >= event.stamp.mono_ns - options.pre_window_ns &&
        previous.stamp.mono_ns <= info.post_until_mono_ns) {appendIncident(previous);}
    }
    capturing = true;
    armed = false;
    correlation_eligible = true;
    ++incidents_started;
    if (prepareDirectory(options.output_directory)) {
      std::ofstream out(info.trace_path, std::ios::trunc);
      out.imbue(std::locale::classic());
      out << std::setprecision(17);
      header(out);
      // Persist the pre-window and trigger immediately. A process crash during
      // the post-window still leaves the evidence that caused this incident.
      for (const auto & previous : incident) {eventJson(out, previous, info);}
      eventJson(out, event, info);
      out.close();
      if (!out) {++io_errors;}
      retainFile(info.trace_path);
    }
    metadata("capturing");
    if (callback) {
      try {callback(info);} catch (...) {++io_errors;}
    }
  }

  void finish(bool partial)
  {
    if (!capturing) {return;}
    capturing = false;
    ++incidents_completed;
    const auto temporary = info.trace_path + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    out.imbue(std::locale::classic());
    out << std::setprecision(17);
    header(out);
    for (const auto & event : incident) {eventJson(out, event, info);}
    out << "{\"schema_version\":1,\"type\":\"summary\",\"incident_id\":";
    jsonString(out, info.incident_id);
    out << ",\"partial\":" << (partial ? "true" : "false");
    field(out, "retained_events", incident.size());
    statsJson(out);
    out << "}\n";
    out.close();
    std::error_code error;
    if (!out) {
      ++io_errors;
      std::filesystem::remove(temporary, error);
    } else {
      std::filesystem::rename(temporary, info.trace_path, error);
      if (error) {
        ++io_errors;
        error.clear();
        std::filesystem::remove(temporary, error);
      }
    }
    metadata("complete");
  }

  void trigger(TraceEvent & event)
  {
    if (!event.trigger_reasons) {return;}
    if (capturing) {
      episode_reasons |= event.trigger_reasons;
      ++correlated_triggers;
    } else if (correlation_eligible && !info.incident_id.empty() &&
      event.stamp.mono_ns <= info.post_until_mono_ns + options.correlation_window_ns)
    {
      // Late abort/fault edges keep the same incident ID. Appending one bounded
      // edge does not reopen the fixed post window or start recurring episodes.
      ++correlated_triggers;
      if (correlated_after_complete++ < options.incident_capacity) {
        std::ofstream out(info.trace_path, std::ios::app);
        out.imbue(std::locale::classic());
        out << std::setprecision(17);
        eventJson(out, event, info);
        out.close();
        if (!out) {++io_errors;}
      } else {++incident_dropped;}
    } else {
      correlated_after_complete = 0;
      begin(event);
    }
  }

  void consume(TraceEvent event)
  {
    if (event.control_tid && identity_tid != event.control_tid) {
      identity_tid = event.control_tid;
      info.control_tid = identity_tid;
      metadata(capturing ? "capturing" : (armed ? "armed" : "complete"));
    }
    if (!event.control_tid && event.kind == TraceKind::EXTERNAL_TRIGGER) {
      event.control_tid = identity_tid;
    }
    if (capturing && event.stamp.mono_ns > info.post_until_mono_ns) {finish(false);}
    if (event.kind == TraceKind::RPC_END) {
      const size_t rpc_index = static_cast<size_t>(event.rpc);
      if (event.rc != 0 && (rpc_index >= last_rpc_result.size() ||
        last_rpc_result[rpc_index] != event.rc)) {event.trigger_reasons |= RPC_ERROR;}
      if (rpc_index < last_rpc_result.size()) {last_rpc_result[rpc_index] = event.rc;}
      if (event.rpc_begin_mono_ns > 0 &&
        event.stamp.mono_ns - event.rpc_begin_mono_ns >= options.slow_threshold_ns)
      {
        event.trigger_reasons |= RPC_SLOW;
      }
    }
    if (event.kind == TraceKind::READ_BEGIN || event.kind == TraceKind::WRITE_BEGIN) {
      const size_t index = event.kind == TraceKind::READ_BEGIN ? 0 : 1;
      if (last_cycle_ns[index] > 0) {
        const int64_t elapsed = event.stamp.mono_ns - last_cycle_ns[index];
        if (elapsed >= options.slow_threshold_ns) {
          event.trigger_reasons |= CYCLE_GAP;
          catchup_remaining[index] = 8;
          rapid_cycles[index] = 0;
        } else {
          const double nominal_ns = event.nominal_period_sec * kNsPerSecond;
          if (nominal_ns > 0 && elapsed > 0 && elapsed < nominal_ns * .5) {
            ++rapid_cycles[index];
            if (catchup_remaining[index] > 0 || rapid_cycles[index] >= 2) {
              event.trigger_reasons |= CATCH_UP;
            }
          } else {rapid_cycles[index] = 0;}
          if (catchup_remaining[index] > 0) {--catchup_remaining[index];}
        }
      }
      last_cycle_ns[index] = event.stamp.mono_ns;
    }
    if (event.kind == TraceKind::READ_END || event.kind == TraceKind::WRITE_END) {
      const size_t index = event.kind == TraceKind::READ_END ? 0 : 1;
      if (last_cycle_ns[index] > 0 &&
        event.stamp.mono_ns - last_cycle_ns[index] >= options.slow_threshold_ns)
      {
        event.trigger_reasons |= CYCLE_GAP;
      }
    }
    trigger(event);
    if (capturing && event.stamp.mono_ns >= info.trigger_mono_ns - options.pre_window_ns &&
      event.stamp.mono_ns <= info.post_until_mono_ns) {appendIncident(event);}
    if (history_size < history.size()) {
      history[(history_start + history_size++) % history.size()] = event;
    } else {
      history[history_start] = event;
      history_start = (history_start + 1) % history.size();
      ++history_evicted;
    }
  }

  void poll(int64_t now_ns)
  {
    // Consume the queue before the wall-clock deadline: a temporarily delayed
    // writer must keep already-queued events whose own timestamp is in-window.
    size_t tail = queue_tail.load(std::memory_order_relaxed);
    const size_t head = queue_head.load(std::memory_order_acquire);
    while (tail != head) {
      TraceEvent event = queue[tail % queue.size()];
      queue_tail.store(++tail, std::memory_order_release);
      consume(event);
    }
    std::array<TraceEvent, 16> pending;
    size_t pending_size = 0;
    {
      std::lock_guard<std::mutex> lock(request_mutex);
      pending_size = request_size;
      for (size_t index = 0; index < pending_size; ++index) {pending[index] = requests[index];}
      request_size = 0;
    }
    for (size_t index = 0; index < pending_size; ++index) {consume(pending[index]);}
    const uint64_t dropped = queue_dropped.load(std::memory_order_acquire);
    if (dropped != drops_observed) {
      drops_observed = dropped;
      TraceEvent event{};
      event.kind = TraceKind::EXTERNAL_TRIGGER;
      event.stamp = {drop_mono_ns.load(), drop_unix_ns.load()};
      event.control_tid = identity_tid;
      event.trigger_reasons = DROP;
      consume(event);
    }
    if (capturing && now_ns >= info.post_until_mono_ns) {finish(false);}
  }

  void run() noexcept
  {
    try {
      if (identity_tid) {metadata("armed");}
      while (!stop_requested.load(std::memory_order_acquire)) {
        poll(clockNow().mono_ns);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      poll(0);
      finish(true);
      if (identity_written || !info.incident_id.empty()) {metadata("stopped");}
    } catch (...) {
      ++io_errors;
      try {finish(true); metadata("stopped");} catch (...) {++io_errors;}
    }
  }

  FlightRecorderOptions options;
  const int32_t pid;
  const uint64_t process_start_ticks;
  std::vector<TraceEvent> queue;
  std::vector<TraceEvent> history;
  std::vector<TraceEvent> incident;
  std::atomic<size_t> queue_head{0}, queue_tail{0};
  std::atomic<bool> accepting{true}, stop_requested{false};
  std::atomic<uint64_t> queue_dropped{0}, history_evicted{0}, incident_dropped{0};
  std::atomic<uint64_t> correlated_triggers{0}, io_errors{0};
  std::atomic<uint64_t> incidents_started{0}, incidents_completed{0};
  std::atomic<int64_t> drop_mono_ns{0}, drop_unix_ns{0};
  std::mutex request_mutex;
  std::array<TraceEvent, 16> requests;
  size_t request_size = 0;
  std::thread writer;
  IncidentCallback callback;
  std::string metadata_path;
  IncidentInfo info;
  uint64_t episode_reasons = 0, episode_sequence = 0, drops_observed = 0;
  size_t history_start = 0, history_size = 0, correlated_after_complete = 0;
  std::array<int64_t, 2> last_cycle_ns{};
  std::array<unsigned, 2> catchup_remaining{};
  std::array<unsigned, 2> rapid_cycles{};
  std::array<int32_t, static_cast<size_t>(TraceRpc::RESET) + 1> last_rpc_result{};
  int32_t identity_tid = 0;
  bool identity_written = false, capturing = false, correlation_eligible = false, armed = true;
  bool disk_authorized = false;
};

FlightRecorder::FlightRecorder(FlightRecorderOptions options)
: impl_(std::make_unique<Impl>(std::move(options))) {}
FlightRecorder::~FlightRecorder() {shutdown();}

void FlightRecorder::start(IncidentCallback callback)
{
  if (impl_->writer.joinable()) {return;}
  impl_->callback = std::move(callback);
  impl_->stop_requested.store(false, std::memory_order_release);
  impl_->accepting.store(true, std::memory_order_release);
  impl_->last_cycle_ns = {};
  impl_->catchup_remaining = {};
  impl_->rapid_cycles = {};
  impl_->last_rpc_result = {};
  // Keep the last handoff identity through a fast deactivate/activate. The host
  // may poll less often than these transitions, and must still see the incident.
  // Only correlation eligibility resets: the next trigger starts a new episode.
  impl_->correlation_eligible = false;
  impl_->armed = true;
  impl_->episode_reasons = 0;
  impl_->history_start = 0;
  impl_->history_size = 0;
  impl_->correlated_after_complete = 0;
  impl_->writer = std::thread([this] {impl_->run();});
  impl_->disk_authorized = true;
}

bool FlightRecorder::record(const TraceEvent & event) noexcept
{
  if (!impl_->accepting.load(std::memory_order_relaxed)) {return false;}
  const size_t head = impl_->queue_head.load(std::memory_order_relaxed);
  const size_t tail = impl_->queue_tail.load(std::memory_order_acquire);
  if (head - tail >= impl_->queue.size()) {
    impl_->drop_mono_ns.store(event.stamp.mono_ns, std::memory_order_relaxed);
    impl_->drop_unix_ns.store(event.stamp.unix_ns, std::memory_order_relaxed);
    impl_->queue_dropped.fetch_add(1, std::memory_order_release);
    return false;
  }
  impl_->queue[head % impl_->queue.size()] = event;
  impl_->queue_head.store(head + 1, std::memory_order_release);
  return true;
}

void FlightRecorder::requestIncident(
  uint64_t reason_bits, ClockStamp stamp,
  const char * goal_uuid, int64_t goal_accepted_ros_ns) noexcept
{
  if (!reason_bits || !impl_->accepting.load(std::memory_order_acquire)) {return;}
  std::lock_guard<std::mutex> lock(impl_->request_mutex);
  if (!impl_->accepting.load(std::memory_order_acquire)) {return;}
  if (impl_->request_size == impl_->requests.size()) {
    ++impl_->incident_dropped;
    return;
  }
  auto & event = impl_->requests[impl_->request_size++];
  event = TraceEvent{};
  event.kind = TraceKind::EXTERNAL_TRIGGER;
  event.stamp = stamp;
  event.trigger_reasons = reason_bits;
  if (goal_uuid) {
    const size_t length = strnlen(goal_uuid, sizeof(event.goal_uuid) - 1);
    std::memcpy(event.goal_uuid, goal_uuid, length);
  }
  event.goal_accepted_ros_ns = goal_accepted_ros_ns;
}

void FlightRecorder::shutdown()
{
  const bool was_accepting = impl_->accepting.exchange(false, std::memory_order_acq_rel);
  if (!was_accepting && !impl_->writer.joinable()) {return;}
  impl_->stop_requested.store(true, std::memory_order_release);
  if (impl_->writer.joinable()) {impl_->writer.join();} else if (impl_->disk_authorized) {
    impl_->poll(0);
    impl_->finish(true);
    if (impl_->identity_written || !impl_->info.incident_id.empty()) {impl_->metadata("stopped");}
  } else {
    // A fake/offline producer does not gain filesystem authority by being
    // destroyed. Tests must explicitly opt in with pollForTest or start.
    impl_->queue_tail.store(
      impl_->queue_head.load(std::memory_order_acquire),
      std::memory_order_release);
    std::lock_guard<std::mutex> lock(impl_->request_mutex);
    impl_->request_size = 0;
  }
}
FlightRecorderStats FlightRecorder::stats() const noexcept {return impl_->snapshot();}
void FlightRecorder::pollForTest(int64_t now_mono_ns)
{
  if (impl_->writer.joinable()) {
    throw std::logic_error("pollForTest requires an offline recorder");
  }
  impl_->disk_authorized = true;
  impl_->poll(now_mono_ns);
}

}  // namespace fairino_hardware
