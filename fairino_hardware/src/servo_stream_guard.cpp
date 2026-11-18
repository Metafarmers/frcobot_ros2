// Copyright 2026 Metafarmers
// SPDX-License-Identifier: BSD-3-Clause
#include "fairino_hardware/fairino_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace fairino_hardware {
namespace {
int64_t monotonicNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
constexpr double kHoldTolerance = 0.001;
constexpr int64_t kRequestLifetimeNs = 2000000000LL;
} // namespace

bool FairinoHardwareInterface::streamHealthy() const {
  return rt_valid_ && rt_frame_seen_ &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       rt_frame_advanced_at_)
                 .count() <= kFreshStateSec &&
         rt_snapshot_.rbtEnableState == 1 && rt_snapshot_.main_code == 0 &&
         rt_snapshot_.sub_code == 0 && rt_snapshot_.EmergencyStop == 0 &&
         rt_snapshot_.collisionState == 0 &&
         rt_snapshot_.safety_stop0_state == 0 &&
         rt_snapshot_.safety_stop1_state == 0 &&
         (rt_snapshot_.robot_mode == 0 || rt_snapshot_.robot_mode == 1) &&
         rt_snapshot_.program_state <= 2 &&
         (rt_snapshot_.robot_state == 1 || rt_snapshot_.robot_state == 2) &&
         !_drag_active && _drag_req.load() == 0 && !hard_inhibit_.load();
}

bool FairinoHardwareInterface::streamAtHold(
    const std::array<double, 6> &hold) const {
  if (!streamHealthy() || rt_snapshot_.robot_state != 1 ||
      rt_snapshot_.mc_queue_len != 0 || rt_snapshot_.motion_done != 1)
    return false;
  for (size_t j = 0; j < 6; ++j) {
    if (!std::isfinite(hold[j]) || !std::isfinite(_jnt_position_command[j]) ||
        std::fabs(hold[j] - _jnt_position_state[j]) > kHoldTolerance ||
        std::fabs(hold[j] - _jnt_position_command[j]) > kHoldTolerance)
      return false;
  }
  return true;
}

FairinoHardwareInterface::StreamState
FairinoHardwareInterface::streamMessage(const StreamSnapshot &snapshot) const {
  StreamState out;
  out.enabled = stream_guard_enabled_;
  out.session_id = stream_session_;
  out.operation_id = snapshot.owner.data();
  out.binding_id = snapshot.binding;
  out.generation = snapshot.generation;
  out.revision = snapshot.revision;
  out.phase = snapshot.phase;
  out.reason = snapshot.reason;
  out.recovery_count = snapshot.recoveries;
  out.completed_request_id = snapshot.completed_id;
  out.request_code = snapshot.code;
  const auto begin = stream_rpc_begin_ns_.load();
  out.delay_pending =
      begin > 0 &&
      (monotonicNs() - begin) * 1e-9 >= stream_delay_threshold_sec_ &&
      stream_rpc_begin_ns_.load() == begin;
  out.feedback_age_sec =
      snapshot.age < 0
          ? -1
          : snapshot.age +
                std::max(0.0, (monotonicNs() - snapshot.sampled_ns) * 1e-9);
  out.feedback_valid = snapshot.feedback_valid && out.feedback_age_sec >= 0 &&
                       out.feedback_age_sec <= kFreshStateSec;
  out.delay_threshold_sec = stream_delay_threshold_sec_;
  out.last_delay_sec = snapshot.last_delay;
  out.measured_position = snapshot.measured;
  out.command_position = snapshot.command;
  return out;
}

void FairinoHardwareInterface::commitStreamState() {
  ++stream_control_.revision;
  stream_control_.sampled_ns = monotonicNs();
  stream_control_.feedback_valid = rt_valid_;
  stream_control_.age =
      rt_frame_seen_
          ? std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          rt_frame_advanced_at_)
                .count()
          : -1;
  std::copy_n(_jnt_position_state, 6, stream_control_.measured.begin());
  std::copy_n(_jnt_position_command, 6, stream_control_.command.begin());
  std::unique_lock<std::mutex> lock(stream_mutex_, std::try_to_lock);
  if (lock.owns_lock())
    stream_published_ = stream_control_;
}

void FairinoHardwareInterface::createStreamInterfaces() {
  stream_pub_ = _svc_node->create_publisher<StreamState>(
      "fairino_hw_control/servo_stream_state",
      rclcpp::QoS(1).reliable().transient_local());
  stream_srv_ = _svc_node->create_service<StreamControl>(
      "fairino_hw_control/control_servo_stream",
      [this](const std::shared_ptr<StreamControl::Request> request,
             std::shared_ptr<StreamControl::Response> response) {
        handleStreamControl(*request, *response);
      });
  stream_timer_ = _svc_node->create_wall_timer(
      std::chrono::milliseconds(20), [this]() { publishStreamState(); });
}

void FairinoHardwareInterface::publishStreamState() {
  StreamSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    snapshot = stream_published_;
  }
  const auto message = streamMessage(snapshot);
  if (snapshot.phase != stream_logged_phase_ ||
      snapshot.generation != stream_logged_generation_) {
    if (snapshot.phase == StreamState::STOPPING ||
        snapshot.phase == StreamState::BLOCKED) {
      RCLCPP_WARN(_svc_node->get_logger(),
                  "[servo-stream] session=%s operation=%s generation=%llu "
                  "phase=%u reason=%u "
                  "recoveries=%u delay_ms=%.3f feedback_age_ms=%.3f; normal "
                  "ServoJ blocked",
                  message.session_id.c_str(), message.operation_id.c_str(),
                  static_cast<unsigned long long>(snapshot.generation),
                  snapshot.phase, snapshot.reason, snapshot.recoveries,
                  snapshot.last_delay * 1000.0,
                  message.feedback_age_sec * 1000.0);
    } else if (snapshot.phase == StreamState::QUIESCENT ||
               (snapshot.phase == StreamState::ACTIVE &&
                snapshot.recoveries > 0)) {
      RCLCPP_INFO(
          _svc_node->get_logger(),
          "[servo-stream] operation=%s generation=%llu phase=%u recoveries=%u",
          message.operation_id.c_str(),
          static_cast<unsigned long long>(snapshot.generation), snapshot.phase,
          snapshot.recoveries);
    }
    stream_logged_phase_ = snapshot.phase;
    stream_logged_generation_ = snapshot.generation;
  }
  stream_pub_->publish(message);
}

void FairinoHardwareInterface::handleStreamControl(
    const StreamControl::Request &req, StreamControl::Response &res) {
  std::lock_guard<std::mutex> lock(stream_mutex_);
  res.state = streamMessage(stream_published_);
  if (req.command == StreamControl::Request::QUERY) {
    if (req.session_id == stream_session_ && req.binding_id != 0 &&
        req.binding_id == stream_published_.binding &&
        req.operation_id == stream_published_.owner.data()) {
      stream_lease_binding_.store(req.binding_id);
      stream_lease_seen_ns_.store(monotonicNs());
    }
    res.accepted = res.completed = true;
    return;
  }
  if (!stream_guard_enabled_) {
    res.code = 4;
    res.message = "guard disabled";
    return;
  }
  if (req.session_id != stream_session_ || req.operation_id.empty() ||
      req.operation_id.size() > 64 ||
      req.operation_id.find('\0') != std::string::npos || req.request_id == 0 ||
      req.command > StreamControl::Request::RELEASE ||
      !std::all_of(req.hold_position.begin(), req.hold_position.end(),
                   [](double v) { return std::isfinite(v); })) {
    res.code = 2;
    res.message = "invalid session, owner or request";
    return;
  }
  StreamRequest item;
  item.command = req.command;
  std::copy(req.operation_id.begin(), req.operation_id.end(),
            item.owner.begin());
  item.binding = req.binding_id;
  item.generation = req.generation;
  item.id = req.request_id;
  item.hold = req.hold_position;
  item.deadline_ns = monotonicNs() + kRequestLifetimeNs;
  const auto same = [&](const StreamRequest &previous) {
    return previous.id == item.id && previous.owner == item.owner &&
           previous.command == item.command &&
           previous.generation == item.generation &&
           previous.binding == item.binding && previous.hold == item.hold;
  };
  if (stream_last_request_.id == item.id) {
    if (!same(stream_last_request_)) {
      res.code = 2;
      res.message = "request id reused";
      return;
    }
    res.accepted = true;
    res.completed = stream_published_.completed_id == item.id;
    res.code = res.completed ? stream_published_.code : 1;
    return;
  }
  if (req.command == StreamControl::Request::QUIESCE && req.binding_id != 0 &&
      req.binding_id == stream_published_.binding &&
      item.owner == stream_published_.owner) {
    // Binding is unchanged across faults, but changes for a new owner. This
    // atomic cancellation also wins while REARM's SDK call is blocked.
    stream_abort_binding_.store(req.binding_id);
  }
  if (stream_mailbox_pending_) {
    res.accepted = same(stream_pending_);
    res.code = res.accepted ? 1 : 3;
    res.message = "control request pending";
    return;
  }
  if (req.command != StreamControl::Request::QUIESCE &&
      req.generation != stream_published_.generation) {
    res.code = 2;
    res.message = "stale generation";
    return;
  }
  stream_pending_ = item;
  stream_mailbox_pending_ = true;
  res.accepted = true;
  res.code = 1;
}

void FairinoHardwareInterface::finishStreamRequest(const StreamRequest &req,
                                                   int code) {
  stream_control_.completed_id = req.id;
  stream_control_.code = code;
}

void FairinoHardwareInterface::consumeStreamRequest() {
  if (stream_control_.binding &&
      (stream_control_.reason == StreamState::NONE ||
       (stream_control_.reason == StreamState::DELAY || stream_control_.reason == StreamState::PATH_DEVIATION)) &&
      (stream_lease_binding_.load() != stream_control_.binding ||
       monotonicNs() - stream_lease_seen_ns_.load() >= 1000000000LL)) {
    latchStream(StreamState::OWNER_LOST);
  }
  if (stream_control_.binding &&
      stream_abort_binding_.load() == stream_control_.binding &&
      stream_control_.reason != StreamState::CANCELED) {
    latchStream(StreamState::CANCELED);
  }
  StreamRequest req;
  {
    std::unique_lock<std::mutex> lock(stream_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !stream_mailbox_pending_)
      return;
    req = stream_pending_;
    stream_last_request_ = req;
    stream_mailbox_pending_ = false;
  }
  if (monotonicNs() >= req.deadline_ns) {
    finishStreamRequest(req, 6);
    return;
  }
  const bool owner = req.owner == stream_control_.owner && req.binding != 0 &&
                     req.binding == stream_control_.binding;
  if (req.command == StreamControl::Request::QUIESCE && owner) {
    latchStream(StreamState::CANCELED);
    finishStreamRequest(req, 0);
    return;
  }
  if (req.generation != stream_control_.generation) {
    finishStreamRequest(req, 2);
    return;
  }
  if (req.command == StreamControl::Request::BIND) {
    if (stream_control_.phase != StreamState::IDLE || stream_latched_.load() ||
        flush_inhibit_ || !streamHealthy()) {
      finishStreamRequest(req, 5);
      return;
    }
    stream_control_.owner = req.owner;
    stream_control_.binding = ++stream_control_.generation;
    stream_lease_binding_.store(stream_control_.binding);
    stream_lease_seen_ns_.store(monotonicNs());
    stream_control_.recoveries = 0;
    stream_control_.reason = StreamState::NONE;
    stream_control_.phase = StreamState::ACTIVE;
    finishStreamRequest(req, 0);
    return;
  }
  if (!owner) {
    finishStreamRequest(req, 2);
    return;
  }
  if (req.command == StreamControl::Request::RELEASE) {
    if (stream_control_.phase != StreamState::ACTIVE ||
        stream_latched_.load()) {
      finishStreamRequest(req, 5);
      return;
    }
    stream_control_.owner.fill(0);
    stream_control_.binding = 0;
    ++stream_control_.generation;
    stream_control_.phase = StreamState::IDLE;
    finishStreamRequest(req, 0);
    return;
  }
  if (req.command != StreamControl::Request::REARM ||
      stream_control_.phase != StreamState::QUIESCENT ||
      (stream_control_.reason != StreamState::DELAY && stream_control_.reason != StreamState::PATH_DEVIATION) ||
      stream_control_.recoveries != 0 ||
      stream_abort_binding_.load() == stream_control_.binding ||
      !streamAtHold(req.hold)) {
    finishStreamRequest(req, 5);
    return;
  }
  ++stream_control_.recoveries;
  stream_control_.phase = StreamState::REARMING;
  stream_rearm_step_ = 0;
  stream_rearm_request_ = req;
}

void FairinoHardwareInterface::latchStream(uint8_t reason, double delay) {
  if (!stream_guard_enabled_)
    return;
  if (stream_latched_.load() && (stream_control_.reason != StreamState::DELAY && stream_control_.reason != StreamState::PATH_DEVIATION) &&
      stream_control_.phase != StreamState::REARMING)
    return;
  if (stream_latched_.load() && stream_control_.reason == reason &&
      stream_control_.phase != StreamState::REARMING)
    return;
  if (stream_control_.phase == StreamState::REARMING) {
    finishStreamRequest(stream_rearm_request_, 5);
  }
  progress_state_[0] = 0;
  ++progress_state_[2];
  progress_robot_mode_ = -1;
  stream_latched_.store(true);
  ++stream_control_.generation;
  stream_control_.reason = reason;
  stream_control_.last_delay = delay;
  stream_control_.phase = StreamState::STOPPING;
  stream_guard_flush_ = true;
  stream_stationary_since_ns_ = 0;
  // Reuse the canonical queue-clear state machine, but leave servo ended until
  // a matching owner explicitly rearms after replacing the JTC trajectory.
  flush_resume_requested_.store(false);
  flush_deadline_ns_.store(monotonicNs() +
                           static_cast<int64_t>(kFlushTimeoutSec * 1e9));
  flush_result_.store(static_cast<int>(FlushResult::PENDING));
  flush_requested_.fetch_add(1);
  flush_inhibit_ = true;
  last_servo_target_valid_ = false;
}

bool FairinoHardwareInterface::processStreamRearm() {
  if (stream_control_.phase != StreamState::REARMING)
    return false;
  if (monotonicNs() >= stream_rearm_request_.deadline_ns) {
    latchStream(StreamState::TIMEOUT);
    return true;
  }
  if (stream_abort_binding_.load() == stream_control_.binding) {
    latchStream(StreamState::CANCELED);
    return true;
  }
  if (!streamAtHold(stream_rearm_request_.hold)) {
    latchStream(StreamState::UNSAFE);
    return true;
  }
  if (stream_rearm_step_ == 0) {
    const auto start = std::chrono::steady_clock::now();
    recordRpcBegin(START_RPC, start);
    const int rc = _ptr_robot->ServoMoveStart();
    recordRpc(START_RPC, rc, start);
    const double duration =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    if (rc || (!progress_configured_ && duration >= stream_delay_threshold_sec_)) {
      latchStream(rc ? StreamState::SDK_ERROR : StreamState::DELAY, duration);
      return true;
    }
    stream_rearm_frame_ = rt_snapshot_.frame_cnt;
    stream_rearm_step_ = 1;
    return true;
  }
  if (rt_snapshot_.frame_cnt == stream_rearm_frame_)
    return true;
  if (stream_rearm_step_ == 1) {
    JointPos command{};
    ExaxisPos external{};
    for (size_t j = 0; j < 6; ++j)
      command.jPos[j] = stream_rearm_request_.hold[j] * 180.0 / M_PI;
    const auto start = std::chrono::steady_clock::now();
    stream_rpc_begin_ns_.store(monotonicNs());
    recordRpcBegin(SERVO_RPC, start);
    const int rc =
        _ptr_robot->ServoJ(&command, &external, 0, 0,
                           static_cast<float>(servo_command_period_sec_), 0, 0);
    recordRpc(SERVO_RPC, rc, start);
    stream_rpc_begin_ns_.store(0);
    const double duration =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    if (rc || (!progress_configured_ && duration >= stream_delay_threshold_sec_)) {
      latchStream(rc ? StreamState::SDK_ERROR : StreamState::DELAY, duration);
      return true;
    }
    stream_rearm_frame_ = rt_snapshot_.frame_cnt;
    stream_rearm_step_ = 2;
    return true;
  }
  last_servo_target_ = stream_rearm_request_.hold;
  last_servo_target_valid_ = true;
  cancel_hold_ = false;
  flush_inhibit_ = false;
  stream_guard_flush_ = false;
  progress_robot_mode_ = rt_snapshot_.robot_mode;
  progress_post_reply_sample_ = true;
  stream_control_.phase = StreamState::ACTIVE;
  stream_control_.reason = StreamState::NONE;
  finishStreamRequest(stream_rearm_request_, 0);
  stream_latched_.store(false);
  // No ordinary trajectory command in the cycle that opens admission.
  return true;
}
} // namespace fairino_hardware
