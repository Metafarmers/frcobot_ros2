// Copyright 2026 Metafarmers
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Test-only controller model. No sockets, robot enable, or SDK implementation.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

namespace fairino_hardware::test
{
struct ServoDelayFakeController
{
  using Clock = std::chrono::steady_clock;
  struct Call
  {
    double start;
    double end;
    std::string name;
    double target;
    double measured;
  };
  explicit ServoDelayFakeController(int delay)
  : delay_ms(delay) {}
  int delay_ms;
  bool injected = false;
  int injections_remaining = 1;
  int injection_count = 0;
  double injection_start_rad = 0.05;
  std::array<double, 6> target_deg{};
  Clock::time_point updated = Clock::now();
  std::vector<Call> calls;
  static double now()
  {
    return std::chrono::duration<double>(Clock::now().time_since_epoch())
           .count();
  }
  void advance(ROBOT_STATE_PKG & state)
  {
    const auto stamp = Clock::now();
    const double dt = std::chrono::duration<double>(stamp - updated).count();
    updated = stamp;
    bool moving = false;
    for (size_t j = 0; j < 6; ++j) {
      const double step = std::clamp(
        target_deg[j] - state.jt_cur_pos[j],
        -35.0 * dt, 35.0 * dt);
      state.jt_cur_pos[j] += step;
      state.actual_qd[j] = dt > 0 ? step / dt : 0.0;
      moving |= std::fabs(target_deg[j] - state.jt_cur_pos[j]) > 1e-6;
    }
    state.robot_state = moving ? 2 : 1;
    state.motion_done = moving ? 0 : 1;
    state.mc_queue_len = 0;
  }
  int beginServo(const JointPos & command, ROBOT_STATE_PKG & state)
  {
    advance(state);
    std::copy_n(command.jPos, 6, target_deg.begin());
    calls.push_back(
      {now(), 0.0, "ServoJ", target_deg[2] * M_PI / 180.0,
        state.jt_cur_pos[2] * M_PI / 180.0});
    // Delay successful replies without changing the requested path.
    if (injections_remaining > 0 && delay_ms > 0 &&
      std::fabs(target_deg[2]) >= injection_start_rad * 180.0 / M_PI)
    {
      injected = true;
      --injections_remaining;
      ++injection_count;
      return delay_ms;
    }
    return 0;
  }
  void endServo() {calls.back().end = now();}
  void record(const std::string & name, ROBOT_STATE_PKG & state)
  {
    advance(state);
    calls.push_back(
      {now(), now(), name, target_deg[2] * M_PI / 180.0,
        state.jt_cur_pos[2] * M_PI / 180.0});
  }
  void stop(ROBOT_STATE_PKG & state)
  {
    record("StopMotion", state);
    std::copy_n(state.jt_cur_pos, 6, target_deg.begin());
    std::fill_n(state.actual_qd, 6, 0.0);
    state.motion_done = 1;
    state.robot_state = 1;
  }
  void save(const std::string & directory) const
  {
    std::ofstream out(directory + "/sdk_calls.csv");
    out << std::setprecision(12)
        << "start_s,end_s,operation,target_j3_rad,measured_j3_rad\n";
    for (const auto & call : calls) {
      out << call.start << ',' << call.end << ',' << call.name << ','
          << call.target << ',' << call.measured << '\n';
    }
  }
};
} // namespace fairino_hardware::test
