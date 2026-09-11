// Copyright 2026 Metafarmers
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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
  float command_period = 0.0F;
};

SdkState sdk;

}  // namespace test
}  // namespace fairino_hardware

// Only the public SDK signatures referenced by the driver are substituted.
FRRobot::FRRobot() = default;
FRRobot::~FRRobot() = default;

errno_t FRRobot::GetRobotRealTimeState(ROBOT_STATE_PKG* state) {
  *state = fairino_hardware::test::sdk.state;
  return 0;
}

errno_t FRRobot::ServoJ(JointPos* joints, ExaxisPos*, float, float,
                        float command_period, float, float, int) {
  auto& sdk = fairino_hardware::test::sdk;
  sdk.servo_commands.push_back(*joints);
  sdk.command_period = command_period;
  return sdk.servo_result;
}

errno_t FRRobot::DragTeachSwitch(uint8_t state) {
  fairino_hardware::test::sdk.drag_requests.push_back(state);
  return 0;
}

errno_t FRRobot::StopMotion() {
  ++fairino_hardware::test::sdk.stop_calls;
  return 0;
}

errno_t FRRobot::ServoMoveStart() {
  ++fairino_hardware::test::sdk.servo_start_calls;
  return 0;
}

errno_t FRRobot::ServoMoveEnd() {
  ++fairino_hardware::test::sdk.servo_end_calls;
  return 0;
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
    EXPECT_EQ(
        hardware_->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
        hardware_interface::return_type::OK);
  }

  void setCommand(const Joints& joints) {
    std::copy(joints.begin(), joints.end(), hardware_->_jnt_position_command);
  }

  void setMeasured(const Joints& joints) {
    std::copy(joints.begin(), joints.end(), hardware_->_jnt_position_state);
  }

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
