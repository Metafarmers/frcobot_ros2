// Copyright 2026 Metafarmers
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware {
namespace test {

// The test target compiles the real driver with these SDK definitions instead
// of linking libfairino. No SDK socket, robot connection, or ROS executor
// starts.
struct SdkState {
  ROBOT_STATE_PKG state{};
  std::vector<JointPos> servo_commands;
  std::vector<uint8_t> drag_requests;
  int stop_calls = 0;
  int servo_start_calls = 0;
  int servo_end_calls = 0;
  int reset_calls = 0;
  int servo_result = 0;
  int state_result = 0;
  int stop_result = 0;
  int end_result = 0;
  int clear_result = 0;
  int queue_result = 0;
  int start_result = 0;
  int queue_length = 0;
  int state_calls = 0;
  int clear_calls = 0;
  bool advance_frame = true;
  std::vector<std::string> operations;
  float command_period = 0.0F;
};

SdkState sdk;

}  // namespace test
}  // namespace fairino_hardware

// Only the public SDK signatures referenced by the driver are substituted.
FRRobot::FRRobot() = default;
FRRobot::~FRRobot() = default;

errno_t FRRobot::GetRobotRealTimeState(ROBOT_STATE_PKG* state) {
  auto& sdk = fairino_hardware::test::sdk;
  ++sdk.state_calls;
  if (sdk.advance_frame) ++sdk.state.frame_cnt;
  *state = sdk.state;
  return sdk.state_result;
}

errno_t FRRobot::ServoJ(JointPos* joints, ExaxisPos*, float, float,
                        float command_period, float, float, int) {
  auto& sdk = fairino_hardware::test::sdk;
  sdk.servo_commands.push_back(*joints);
  sdk.operations.push_back("servo");
  sdk.command_period = command_period;
  return sdk.servo_result;
}

errno_t FRRobot::DragTeachSwitch(uint8_t state) {
  fairino_hardware::test::sdk.drag_requests.push_back(state);
  return 0;
}

errno_t FRRobot::StopMotion() {
  auto& sdk = fairino_hardware::test::sdk;
  ++sdk.stop_calls;
  sdk.operations.push_back("stop");
  return sdk.stop_result;
}

errno_t FRRobot::MotionQueueClear() {
  auto& sdk = fairino_hardware::test::sdk;
  ++sdk.clear_calls;
  sdk.operations.push_back("clear");
  return sdk.clear_result;
}

errno_t FRRobot::GetMotionQueueLength(int* length) {
  auto& sdk = fairino_hardware::test::sdk;
  sdk.operations.push_back("queue");
  *length = sdk.queue_length;
  return sdk.queue_result;
}

errno_t FRRobot::ServoMoveStart() {
  auto& sdk = fairino_hardware::test::sdk;
  ++sdk.servo_start_calls;
  sdk.operations.push_back("start");
  return sdk.start_result;
}

errno_t FRRobot::ServoMoveEnd() {
  auto& sdk = fairino_hardware::test::sdk;
  ++sdk.servo_end_calls;
  sdk.operations.push_back("end");
  return sdk.end_result;
}

errno_t FRRobot::ResetAllError() {
  ++fairino_hardware::test::sdk.reset_calls;
  return 0;
}

errno_t FRRobot::RPC(const char*) {
  ADD_FAILURE() << "write() tests must not activate the hardware or connect";
  return -1;
}

errno_t FRRobot::CloseRPC() {
  ADD_FAILURE() << "write() tests must not enter connection lifecycle methods";
  return -1;
}

errno_t FRRobot::RobotEnable(uint8_t) {
  ADD_FAILURE() << "write() tests must not enable hardware";
  return -1;
}

errno_t FRRobot::GetActualJointPosDegree(uint8_t, JointPos*) {
  ADD_FAILURE() << "write() tests inject measured joint states directly";
  return -1;
}

namespace fairino_hardware {

enum class StopSource { FLANGE_BUTTON, DRAG_SERVICE, PENDANT_DRAG, SI0, SI1 };

class FairinoHardwareInterfaceTest
    : public ::testing::TestWithParam<StopSource> {
 protected:
  using Joints = std::array<double, 6>;

  void SetUp() override {
    test::sdk = test::SdkState{};
    // The button is active LOW; the idle value must not request a drag.
    test::sdk.state.tl_dgt_input_l = 1;
    test::sdk.state.robot_state = 1;
    hardware_ = std::make_unique<FairinoHardwareInterface>();
    hardware_->_ptr_robot = std::make_unique<FRRobot>();
    hardware_->_control_mode = 0;
    setCommand(Joints{});
    setMeasured(Joints{});
  }

  void writeCycle() {
    EXPECT_EQ(hardware_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
              hardware_interface::return_type::OK);
    EXPECT_EQ(
        hardware_->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
        hardware_interface::return_type::OK);
  }

  void setCommand(const Joints& joints) {
    std::copy(joints.begin(), joints.end(), hardware_->_jnt_position_command);
  }

  void setMeasured(const Joints& joints) {
    std::copy(joints.begin(), joints.end(), hardware_->_jnt_position_state);
    for (size_t j = 0; j < joints.size(); ++j) {
      test::sdk.state.jt_cur_pos[j] = joints[j] * 180.0 / std::acos(-1.0);
    }
  }

  void requestFlush(double timeout = 2.0) {
    hardware_->flush_deadline_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
        (std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout))).time_since_epoch()).count());
    hardware_->flush_result_.store(static_cast<int>(FairinoHardwareInterface::FlushResult::PENDING));
    hardware_->flush_requested_.fetch_add(1);
  }

  bool cancelHolding() const { return hardware_->cancel_hold_; }
  bool flushBlocked() const { return hardware_->flush_inhibit_; }
  bool hardInhibited() const { return hardware_->hard_inhibit_.load(); }
  uint64_t flushCompleted() const { return hardware_->flush_completed_.load(); }
  uint64_t flushRequested() const { return hardware_->flush_requested_.load(); }
  bool flushSucceeded() const {
    return hardware_->flush_result_.load() == static_cast<int>(FairinoHardwareInterface::FlushResult::SUCCESS);
  }
  void expireFlush() { hardware_->flush_deadline_ns_.store(0); }
  void staleFrame() {
    hardware_->rt_frame_advanced_at_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  }
  hardware_interface::return_type readOnly() {
    return hardware_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02));
  }
  hardware_interface::return_type writeOnly() {
    return hardware_->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02));
  }
  Joints measured() const {
    Joints result;
    std::copy_n(hardware_->_jnt_position_state, 6, result.begin());
    return result;
  }
  auto callFlushService() {
    return std::async(std::launch::async, [this]() {
      std_srvs::srv::Trigger::Response response;
      hardware_->requestStopAndFlush(response, true);
      return response;
    });
  }
  std_srvs::srv::Trigger::Response callFlushServiceSync() {
    std_srvs::srv::Trigger::Response response;
    hardware_->requestStopAndFlush(response, true);
    return response;
  }
  auto callStopOnlyService() {
    return std::async(std::launch::async, [this]() {
      std_srvs::srv::Trigger::Response response;
      hardware_->requestStopAndFlush(response, false);
      return response;
    });
  }
  std_srvs::srv::Trigger::Response callStopOnlyServiceSync() {
    std_srvs::srv::Trigger::Response response;
    hardware_->requestStopAndFlush(response, false);
    return response;
  }
  bool initPeriod(const std::string& value) {
    hardware_interface::HardwareInfo info;
    for (int j = 0; j < 6; ++j) {
      hardware_interface::ComponentInfo joint;
      joint.name = "j" + std::to_string(j + 1);
      hardware_interface::InterfaceInfo position;
      position.name = "position";
      joint.command_interfaces.push_back(position);
      joint.state_interfaces.push_back(position);
      info.joints.push_back(joint);
    }
    if (!value.empty()) info.hardware_parameters["servo_command_period_sec"] = value;
    return hardware_->on_init(info) == hardware_interface::CallbackReturn::SUCCESS;
  }
  auto diagnostics() const { return hardware_->published_diagnostics_; }

  bool isEstop() const {
    return GetParam() == StopSource::SI0 || GetParam() == StopSource::SI1;
  }

  bool isHolding() const {
    return isEstop() ? hardware_->_post_estop_hold : hardware_->_post_drag_hold;
  }

  void beginStop() {
    switch (GetParam()) {
      case StopSource::FLANGE_BUTTON:
        test::sdk.state.tl_dgt_input_l = 0;
        break;
      case StopSource::DRAG_SERVICE:
        hardware_->_drag_req.store(1);
        break;
      case StopSource::PENDANT_DRAG:
        test::sdk.state.robot_state = 4;
        break;
      case StopSource::SI0:
        test::sdk.state.safety_stop0_state = 1;
        break;
      case StopSource::SI1:
        test::sdk.state.safety_stop1_state = 1;
        break;
    }
    writeCycle();
  }

  void endStop() {
    switch (GetParam()) {
      case StopSource::FLANGE_BUTTON:
        test::sdk.state.tl_dgt_input_l = 1;
        break;
      case StopSource::DRAG_SERVICE:
        hardware_->_drag_req.store(-1);
        break;
      case StopSource::PENDANT_DRAG:
        test::sdk.state.robot_state = 1;
        break;
      case StopSource::SI0:
        test::sdk.state.safety_stop0_state = 0;
        break;
      case StopSource::SI1:
        test::sdk.state.safety_stop1_state = 0;
        break;
    }
    writeCycle();
  }

  void expectTarget(const Joints& radians) {
    ASSERT_FALSE(test::sdk.servo_commands.empty());
    const auto& degrees = test::sdk.servo_commands.back();
    for (size_t joint = 0; joint < radians.size(); ++joint) {
      EXPECT_NEAR(degrees.jPos[joint], radians[joint] * 180.0 / std::acos(-1.0),
                  1e-10)
          << "joint " << joint;
    }
    EXPECT_FLOAT_EQ(test::sdk.command_period, 0.02F);
  }

  void enterDisplacedHold() {
    beginStop();
    setMeasured(displaced_);
    writeCycle();
    endStop();
    ASSERT_TRUE(isHolding());
    expectTarget(displaced_);
  }

  const Joints displaced_{0.3, -0.2, 0.4, -0.5, 0.6, -0.7};
  std::unique_ptr<FairinoHardwareInterface> hardware_;
};

TEST_F(FairinoHardwareInterfaceTest, ReadUsesOneStateRpcAndWriteReusesTheSnapshot) {
  setMeasured(displaced_);
  setCommand(displaced_);
  EXPECT_EQ(readOnly(), hardware_interface::return_type::OK);
  EXPECT_EQ(test::sdk.state_calls, 1);
  // SDK backing state changes after read, but write must use read's coherent
  // position + safety + button snapshot, not issue another read RPC.
  test::sdk.state.safety_stop0_state = 1;
  test::sdk.state.tl_dgt_input_l = 0;
  std::fill_n(test::sdk.state.jt_cur_pos, 6, 99.0);
  EXPECT_EQ(writeOnly(), hardware_interface::return_type::OK);
  EXPECT_EQ(test::sdk.state_calls, 1);
  expectTarget(displaced_);
  EXPECT_EQ(test::sdk.stop_calls, 0);
  const auto actual = measured();
  for (size_t j = 0; j < actual.size(); ++j) EXPECT_NEAR(actual[j], displaced_[j], 1e-12);
  EXPECT_EQ(writeOnly(), hardware_interface::return_type::ERROR);
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
}

TEST_F(FairinoHardwareInterfaceTest, InvalidSixthJointAndStaleFrameBlockServo) {
  writeCycle();
  test::sdk.advance_frame = false;
  staleFrame();
  EXPECT_EQ(readOnly(), hardware_interface::return_type::ERROR);
  EXPECT_EQ(writeOnly(), hardware_interface::return_type::ERROR);
  test::sdk.advance_frame = true;
  test::sdk.state.jt_cur_pos[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(readOnly(), hardware_interface::return_type::ERROR);
  EXPECT_EQ(writeOnly(), hardware_interface::return_type::ERROR);
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
}

TEST_F(FairinoHardwareInterfaceTest, InvalidSixthCommandBlocksServo) {
  Joints command{};
  command[5] = std::numeric_limits<double>::infinity();
  setCommand(command);
  EXPECT_EQ(readOnly(), hardware_interface::return_type::OK);
  EXPECT_EQ(writeOnly(), hardware_interface::return_type::ERROR);
  EXPECT_TRUE(test::sdk.servo_commands.empty());
}

TEST_F(FairinoHardwareInterfaceTest, PeriodDefaultsToTwentyMsAndAcceptsValidatedOverride) {
  ASSERT_TRUE(initPeriod(""));
  writeCycle();
  EXPECT_FLOAT_EQ(test::sdk.command_period, 0.02F);
  ASSERT_TRUE(initPeriod("0.025"));
  writeCycle();
  EXPECT_FLOAT_EQ(test::sdk.command_period, 0.025F);
  EXPECT_DOUBLE_EQ(diagnostics().command_period_sec, 0.025);
}

TEST_F(FairinoHardwareInterfaceTest, PeriodRejects125HzNonFiniteAndJunk) {
  for (const std::string value : {"0.008", "0", "-0.02", "nan", "inf", "0.02junk", "0.2"}) {
    EXPECT_FALSE(initPeriod(value)) << value;
  }
}

TEST_F(FairinoHardwareInterfaceTest, FlushWaitsForQueueStationaryWindowAndRestartFrame) {
  setCommand(displaced_);
  requestFlush();
  writeCycle();
  EXPECT_EQ(test::sdk.operations, (std::vector<std::string>{"stop", "end", "clear"}));
  EXPECT_EQ(flushCompleted(), 0u);
  test::sdk.queue_length = 3;
  writeCycle();
  EXPECT_EQ(flushCompleted(), 0u);
  test::sdk.queue_length = 0;
  writeCycle();
  test::sdk.advance_frame = false;
  writeCycle();
  EXPECT_EQ(flushCompleted(), 0u);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  test::sdk.advance_frame = true;
  Joints stopped{};
  stopped[3] = 0.1;
  setMeasured(stopped);
  for (unsigned frame = 0; frame < 2; ++frame) {
    writeCycle();
    EXPECT_EQ(flushCompleted(), 0u);
    EXPECT_EQ(test::sdk.servo_start_calls, 0);
  }
  writeCycle();
  EXPECT_EQ(test::sdk.servo_start_calls, 1);
  EXPECT_EQ(flushCompleted(), 0u);
  writeCycle();
  EXPECT_TRUE(flushSucceeded());
  EXPECT_EQ(flushCompleted(), 1u);
  EXPECT_TRUE(cancelHolding());
  EXPECT_FALSE(flushBlocked());
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  writeCycle();
  expectTarget(stopped);
  EXPECT_EQ(diagnostics().queue_length, 0);
  EXPECT_EQ(diagnostics().flush_generation, 1u);
}

TEST_F(FairinoHardwareInterfaceTest, CancelHoldRejectsEvolvingDistantCommandAndReleasesNearActual) {
  setMeasured(displaced_);
  requestFlush();
  for (int i = 0; i < 6; ++i) writeCycle();
  ASSERT_TRUE(flushSucceeded());
  Joints stale{};
  for (int i = 1; i <= 10; ++i) {
    stale[0] = i * 0.001;
    setCommand(stale);
    writeCycle();
    EXPECT_TRUE(cancelHolding());
    expectTarget(displaced_);
  }
  Joints fresh = displaced_;
  fresh[5] += 0.001;
  setCommand(fresh);
  writeCycle();
  EXPECT_FALSE(cancelHolding());
  expectTarget(fresh);
}

TEST_F(FairinoHardwareInterfaceTest, FlushRpcFailureNeverRestartsOrReleasesHold) {
  test::sdk.clear_result = 14;
  requestFlush();
  writeCycle();
  EXPECT_EQ(flushCompleted(), 1u);
  EXPECT_FALSE(flushSucceeded());
  setCommand(displaced_);
  for (int i = 0; i < 3; ++i) writeCycle();
  EXPECT_TRUE(flushBlocked());
  EXPECT_TRUE(cancelHolding());
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, FlushTimeoutWithQueueStillOccupiedBlocksMotion) {
  requestFlush();
  writeCycle();
  test::sdk.queue_length = 2;
  writeCycle();
  expireFlush();
  writeCycle();
  EXPECT_EQ(flushCompleted(), 1u);
  EXPECT_FALSE(flushSucceeded());
  EXPECT_TRUE(flushBlocked());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, FailedRestartCanOnlyRecoverThroughAnotherCompletedFlush) {
  test::sdk.start_result = 14;
  requestFlush();
  for (int i = 0; i < 5; ++i) writeCycle();
  EXPECT_FALSE(flushSucceeded());
  EXPECT_TRUE(flushBlocked());
  setCommand(displaced_);
  writeCycle();
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  test::sdk.start_result = 0;
  requestFlush();
  for (int i = 0; i < 6; ++i) writeCycle();
  EXPECT_EQ(flushCompleted(), 2u);
  EXPECT_TRUE(flushSucceeded());
  EXPECT_FALSE(flushBlocked());
  EXPECT_TRUE(cancelHolding());
}

TEST_F(FairinoHardwareInterfaceTest, FeedbackFailureStillConsumesStopButCannotCompleteFlush) {
  requestFlush();
  test::sdk.state_result = 7;
  EXPECT_EQ(readOnly(), hardware_interface::return_type::ERROR);
  EXPECT_EQ(writeOnly(), hardware_interface::return_type::OK);
  EXPECT_EQ(test::sdk.stop_calls, 1);
  EXPECT_EQ(writeOnly(), hardware_interface::return_type::OK);  // queue query, no fresh actual
  EXPECT_EQ(writeOnly(), hardware_interface::return_type::OK);
  EXPECT_EQ(flushCompleted(), 0u);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  expireFlush();
  EXPECT_EQ(writeOnly(), hardware_interface::return_type::OK);
  EXPECT_EQ(flushCompleted(), 1u);
  EXPECT_FALSE(flushSucceeded());
  EXPECT_TRUE(flushBlocked());
}

TEST_F(FairinoHardwareInterfaceTest, FlushDoesNotRestartDuringSafetyStop) {
  requestFlush();
  writeCycle();
  writeCycle();
  test::sdk.state.safety_stop1_state = 1;
  writeCycle();
  EXPECT_FALSE(flushSucceeded());
  EXPECT_EQ(flushCompleted(), 1u);
  EXPECT_TRUE(flushBlocked());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  EXPECT_TRUE(test::sdk.servo_commands.empty());
}

TEST_F(FairinoHardwareInterfaceTest, FlushServiceWaitsForControlThreadCompletionAndRejectsOverlap) {
  auto response = callFlushService();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (flushRequested() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  ASSERT_EQ(flushRequested(), 1u);
  EXPECT_TRUE(test::sdk.operations.empty());
  EXPECT_FALSE(callFlushServiceSync().success);
  EXPECT_EQ(response.wait_for(std::chrono::milliseconds(1)), std::future_status::timeout);
  for (int i = 0; i < 6; ++i) writeCycle();
  EXPECT_TRUE(response.get().success);
}

TEST_F(FairinoHardwareInterfaceTest, MovingFeedbackCannotCompleteFlush) {
  requestFlush();
  writeCycle();
  writeCycle();
  test::sdk.state.robot_state = 2;
  test::sdk.state.actual_qd[2] = 1.0;
  for (int i = 0; i < 5; ++i) writeCycle();
  EXPECT_EQ(flushCompleted(), 0u);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  test::sdk.state.robot_state = 1;
  test::sdk.state.actual_qd[2] = 0.0;
  for (int i = 0; i < 4; ++i) writeCycle();
  EXPECT_TRUE(flushSucceeded());
}

TEST_F(FairinoHardwareInterfaceTest, StopOnlyHardInhibitsUntilHardwareRestart) {
  auto response = callStopOnlyService();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (flushRequested() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  ASSERT_EQ(flushRequested(), 1u);
  for (int i = 0; i < 5; ++i) writeCycle();
  EXPECT_TRUE(response.get().success);
  EXPECT_TRUE(flushBlocked());
  EXPECT_TRUE(hardInhibited());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  setCommand(displaced_);
  writeCycle();
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  EXPECT_TRUE(callStopOnlyServiceSync().success);
  EXPECT_FALSE(callFlushServiceSync().success);
}

TEST_F(FairinoHardwareInterfaceTest, FailedStopOnlyCanRetryUntilPhysicallyConfirmed) {
  test::sdk.clear_result = 14;
  auto failed = callStopOnlyService();
  const auto first_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (flushRequested() == 0 && std::chrono::steady_clock::now() < first_deadline) {
    std::this_thread::yield();
  }
  ASSERT_EQ(flushRequested(), 1u);
  writeCycle();
  EXPECT_FALSE(failed.get().success);
  EXPECT_TRUE(hardInhibited());
  EXPECT_EQ(flushCompleted(), 1u);

  test::sdk.clear_result = 0;
  auto retry = callStopOnlyService();
  const auto retry_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (flushRequested() < 2 && std::chrono::steady_clock::now() < retry_deadline) {
    std::this_thread::yield();
  }
  ASSERT_EQ(flushRequested(), 2u);
  for (int i = 0; i < 5; ++i) writeCycle();
  EXPECT_TRUE(retry.get().success);
  EXPECT_TRUE(flushSucceeded());
  EXPECT_EQ(flushCompleted(), 2u);
  EXPECT_EQ(test::sdk.clear_calls, 2);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, FlushServiceTimeoutCannotLaterRestartMotion) {
  const auto begin = std::chrono::steady_clock::now();
  const auto response = callFlushServiceSync();
  EXPECT_FALSE(response.success);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count(), 2.5);
  EXPECT_TRUE(test::sdk.operations.empty());
  EXPECT_FALSE(callFlushServiceSync().success);
  writeCycle();
  EXPECT_EQ(flushCompleted(), 1u);
  EXPECT_TRUE(flushBlocked());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  EXPECT_TRUE(test::sdk.servo_commands.empty());
}

TEST_F(FairinoHardwareInterfaceTest, DiagnosticsTrackReturnCodesAndMeasuredServoTargetError) {
  test::sdk.servo_result = 14;
  test::sdk.state.servoJCmdNum = 73;
  setCommand(displaced_);
  writeCycle();
  const auto snapshot = diagnostics();
  EXPECT_EQ(snapshot.servo_command_count, 73);
  EXPECT_NEAR(snapshot.target_state_error_rad, 0.7, 1e-12);
  EXPECT_DOUBLE_EQ(snapshot.loop_period_sec, 0.02);
  EXPECT_EQ(snapshot.rpc[0].calls, 1u);
  EXPECT_EQ(snapshot.rpc[1].rc, 14);
  EXPECT_EQ(snapshot.rpc[1].calls, 1u);
  EXPECT_GE(snapshot.rpc[1].duration_ms, 0.0);
}

TEST_P(FairinoHardwareInterfaceTest, ActiveStopSkipsServoAndErrorRecovery) {
  // Any unintended ServoJ would fail and trigger the driver's recovery path.
  test::sdk.servo_result = 14;
  beginStop();
  for (int cycle = 0; cycle < 3; ++cycle) {
    setCommand(displaced_);
    writeCycle();
  }
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  EXPECT_EQ(test::sdk.reset_calls, 0);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  EXPECT_EQ(test::sdk.stop_calls, isEstop() ? 1 : 0);
}

TEST_P(FairinoHardwareInterfaceTest, OldCommandRetainsMovedPoseAfterRelease) {
  enterDisplacedHold();
  const auto first_command_count = test::sdk.servo_commands.size();
  for (int cycle = 0; cycle < 3; ++cycle) {
    setCommand(Joints{});
    writeCycle();
    EXPECT_TRUE(isHolding());
    expectTarget(displaced_);
  }
  EXPECT_EQ(test::sdk.servo_commands.size(), first_command_count + 3);
  EXPECT_EQ(test::sdk.stop_calls, isEstop() ? 1 : 0);
}

TEST_P(FairinoHardwareInterfaceTest, RepeatedSmallOffsetStaysHeld) {
  enterDisplacedHold();
  Joints command{};
  command[0] = 0.001;
  for (int cycle = 0; cycle < 10; ++cycle) {
    setCommand(command);
    writeCycle();
    EXPECT_TRUE(isHolding());
    expectTarget(displaced_);
  }
}

TEST_P(FairinoHardwareInterfaceTest,
       CumulativeSmallStepsReleaseAtSnapshotLimit) {
  enterDisplacedHold();
  Joints command{};
  // Every step is only 0.001 rad. The comparison is against a fixed snapshot,
  // so accumulated displacement releases hold even though each step is small.
  for (int step = 1; step < 5; ++step) {
    command[5] = -0.001 * step;
    setCommand(command);
    writeCycle();
    EXPECT_TRUE(isHolding());
    expectTarget(displaced_);
  }
  command[5] = -0.005;
  setCommand(command);
  writeCycle();
  EXPECT_FALSE(isHolding());
  expectTarget(command);
}

TEST_P(FairinoHardwareInterfaceTest, CommandNearDisplacedPoseReleasesHold) {
  enterDisplacedHold();
  Joints new_command = displaced_;
  new_command[0] += 0.0001;
  setCommand(new_command);
  writeCycle();
  EXPECT_FALSE(isHolding());
  expectTarget(new_command);
}

TEST_P(FairinoHardwareInterfaceTest, EvolvingOldTrajectoryAlsoReleasesHold) {
  enterDisplacedHold();
  // Characterization of a limitation, not a desired safety guarantee: write()
  // has command samples but no goal identity. A stale trajectory that keeps
  // evolving is indistinguishable from a new goal once it crosses 0.005 rad.
  Joints stale_command{};
  stale_command[0] = 0.006;
  setCommand(stale_command);
  writeCycle();
  EXPECT_FALSE(isHolding());
  expectTarget(stale_command);
}

INSTANTIATE_TEST_SUITE_P(StopSources, FairinoHardwareInterfaceTest,
                         ::testing::Values(StopSource::FLANGE_BUTTON,
                                           StopSource::DRAG_SERVICE,
                                           StopSource::PENDANT_DRAG,
                                           StopSource::SI0, StopSource::SI1),
                         [](const ::testing::TestParamInfo<StopSource>& info) {
                           switch (info.param) {
                             case StopSource::FLANGE_BUTTON:
                               return std::string("FlangeButton");
                             case StopSource::DRAG_SERVICE:
                               return std::string("DragService");
                             case StopSource::PENDANT_DRAG:
                               return std::string("PendantDrag");
                             case StopSource::SI0:
                               return std::string("SI0");
                             case StopSource::SI1:
                               return std::string("SI1");
                           }
                           return std::string("Unknown");
                         });

}  // namespace fairino_hardware
