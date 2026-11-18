// Copyright 2026 Metafarmers
// SPDX-License-Identifier: BSD-3-Clause
#include "fairino_hardware/fairino_hardware_interface.hpp"
#include <algorithm>
#include <cmath>
#include <thread>

namespace fairino_hardware
{
bool FairinoHardwareInterface::initializeProgress()
{
  const auto resource = info_.hardware_parameters.find("servo_progress_resource");
  if (resource != info_.hardware_parameters.end()) {progress_resource_ = resource->second;}
  progress_configured_ = std::any_of(
    info_.gpios.begin(), info_.gpios.end(),
    [&](const auto & gpio) {return gpio.name == progress_resource_;});
  if (!progress_configured_) {
    return true;                            // Old controller config retains conservative guard.
  }
  if (!stream_guard_enabled_) {return false;}
  static std::atomic<uint32_t> incarnation{0};
  progress_state_ = {0, 0, static_cast<double>(++incarnation), 0};
  progress_command_ = {0, 0, progress_state_[2], 0};
  return true;
}

bool FairinoHardwareInterface::progressActive() const
{
  return progress_configured_ && (progress_command_[0] == 1 || progress_command_[0] == 2);
}

void FairinoHardwareInterface::updateProgress()
{
  if (!progress_configured_) {return;}
  progress_state_[3] =
    (!stream_latched_.load() && flush_requested_.load() != flush_completed_.load()) ?
    StreamState::STOPPING : stream_control_.phase;
  progress_state_[0] = 0;
  if (!streamHealthy() || stream_latched_.load() || flush_inhibit_ ||
    flush_requested_.load() != flush_completed_.load()) {return;}
  // A delayed call may have left a cached packet from before the reply. Observe
  // one post-reply cache sample, then demand a subsequent advancing frame.
  if (progress_post_reply_sample_) {
    progress_reply_frame_ = rt_snapshot_.frame_cnt;
    progress_wait_frame_ = true; progress_post_reply_sample_ = false;
    return;
  }
  if (progress_wait_frame_) {
    if (rt_snapshot_.frame_cnt == progress_reply_frame_ ||
      std::chrono::steady_clock::now() < progress_next_send_) {return;}
    progress_wait_frame_ = false;
  }
  progress_state_[0] = 1;
}

bool FairinoHardwareInterface::admitProgressTarget(const double * target)
{
  if (!progressActive()) {return !progress_configured_;}
  if (progress_command_[2] != progress_state_[2] ||
    !std::isfinite(progress_command_[1]) ||
    progress_command_[1] != std::floor(progress_command_[1]) ||
    progress_command_[1] > 9007199254740990.0)
  {
    latchStream(StreamState::UNSAFE); return false;
  }
  if (progress_command_[1] == progress_state_[1]) {return false;}
  if (progress_command_[1] != progress_state_[1] + 1) {
    latchStream(StreamState::UNSAFE); return false;
  }
  if (progress_state_[0] != 1) {return false;}
  // read() runs before the wire call. Quantizing this deadline to read ticks
  // halves nominal throughput when read is a fraction of a millisecond early.
  // Keep strict wire pacing here instead: at most one configured period, usually
  // only the read/update phase difference. Never accumulate catch-up credit.
  if (std::chrono::steady_clock::now() < progress_next_send_) {
    std::this_thread::sleep_until(progress_next_send_);
    consumeStreamRequest();
  }
  if (stream_latched_.load() || flush_inhibit_ ||
    flush_requested_.load() != flush_completed_.load()) {return false;}
  if (!streamHealthy() || progress_command_[2] != progress_state_[2] ||
    progress_command_[1] != progress_state_[1] + 1)
  {
    latchStream(StreamState::UNSAFE); return false;
  }
  if (progress_robot_mode_ >= 0 && rt_snapshot_.robot_mode != progress_robot_mode_) {
    latchStream(StreamState::UNSAFE); return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(target[i]) || !std::isfinite(rt_snapshot_.actual_qd[i])) {
      latchStream(StreamState::UNSAFE);
      return false;
    }
  }
  progress_robot_mode_ = rt_snapshot_.robot_mode;
  return true;
}

void FairinoHardwareInterface::acknowledgeProgress(double duration)
{
  if (!progressActive()) {return;}
  progress_state_[1] = progress_command_[1];
  progress_state_[0] = 0;
  const auto now = std::chrono::steady_clock::now();
  // No catch-up burst. A late return starts a new full command period; an
  // ordinary call preserves the configured send-to-send period.
  const double wait = duration >= servo_command_period_sec_ ? servo_command_period_sec_ :
    servo_command_period_sec_ - duration;
  progress_next_send_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(wait));
  progress_post_reply_sample_ = duration >= servo_command_period_sec_;
}
}  // namespace fairino_hardware
