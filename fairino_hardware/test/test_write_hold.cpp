// Copyright 2026 Metafarmers
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <future>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "servo_delay_fake_controller.hpp"
#include <mf_manipulator_controllers/progress_controller.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cstdlib>

namespace fairino_hardware {
namespace test {

// The test target compiles the real driver with these SDK definitions instead
// of linking libfairino. No SDK socket or robot connection starts. Logging
// callbacks use an isolated ROS domain; one test spins only that executor.
struct SdkState {
  ROBOT_STATE_PKG state{};
  std::unique_ptr<ServoDelayFakeController> demo;
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
  std::vector<int> end_results;
  int clear_result = 0;
  int queue_result = 0;
  int start_result = 0;
  int drag_result = 0;
  int queue_length = 0;
  int state_calls = 0;
  int clear_calls = 0;
  bool advance_frame = true;
  bool allow_deactivate = false;
  int servo_delay_ms = 0;
  std::function<void()> before_servo_return;
  std::function<void()> after_close;
  std::vector<std::string> operations;
  float command_period = 0.0F;
};

SdkState sdk;

class FaultLogCapture {
 public:
  FaultLogCapture() : previous_(rcutils_logging_get_output_handler()) {
    active_ = this;
    rcutils_logging_set_output_handler(capture);
  }
  ~FaultLogCapture() {
    rcutils_logging_set_output_handler(previous_);
    active_ = nullptr;
  }
  std::vector<std::string> messages;
  std::vector<std::string> operations_at_first_message;

 private:
  static void capture(const rcutils_log_location_t*, int severity, const char* name,
                      rcutils_time_point_value_t, const char* format, va_list* args) {
    if (!active_ || severity < RCUTILS_LOG_SEVERITY_ERROR ||
        std::string(name) != "FairinoHardwareInterface") return;
    char text[8192];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(text, sizeof(text), format, copy);
    va_end(copy);
    if (active_->messages.empty()) active_->operations_at_first_message = sdk.operations;
    active_->messages.emplace_back(text);
  }
  inline static FaultLogCapture* active_ = nullptr;
  rcutils_logging_output_handler_t previous_;
};

}  // namespace test
}  // namespace fairino_hardware

// Only the public SDK signatures referenced by the driver are substituted.
FRRobot::FRRobot() = default;
FRRobot::~FRRobot() = default;

errno_t FRRobot::GetRobotRealTimeState(ROBOT_STATE_PKG* state) {
  auto& sdk = fairino_hardware::test::sdk;
  ++sdk.state_calls;
  if (sdk.demo) sdk.demo->advance(sdk.state);
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
  const int delay = sdk.demo ? sdk.demo->beginServo(*joints, sdk.state) : sdk.servo_delay_ms;
  if (delay != 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  if (delay > 0 && sdk.before_servo_return) sdk.before_servo_return();
  if (sdk.demo) sdk.demo->endServo();
  return sdk.servo_result;
}

errno_t FRRobot::DragTeachSwitch(uint8_t state) {
  fairino_hardware::test::sdk.drag_requests.push_back(state);
  return fairino_hardware::test::sdk.drag_result;
}

errno_t FRRobot::StopMotion() {
  auto& sdk = fairino_hardware::test::sdk;
  ++sdk.stop_calls;
  if (sdk.demo) sdk.demo->stop(sdk.state);
  sdk.operations.push_back("stop");
  return sdk.stop_result;
}

errno_t FRRobot::MotionQueueClear() {
  auto& sdk = fairino_hardware::test::sdk;
  ++sdk.clear_calls;
  if (sdk.demo) sdk.demo->record("MotionQueueClear", sdk.state);
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
  if (sdk.demo) sdk.demo->record("ServoMoveStart", sdk.state);
  sdk.operations.push_back("start");
  return sdk.start_result;
}

errno_t FRRobot::ServoMoveEnd() {
  auto& sdk = fairino_hardware::test::sdk;
  ++sdk.servo_end_calls;
  if (sdk.demo) sdk.demo->record("ServoMoveEnd", sdk.state);
  sdk.operations.push_back("end");
  if (static_cast<size_t>(sdk.servo_end_calls) <= sdk.end_results.size()) {
    return sdk.end_results[sdk.servo_end_calls - 1];
  }
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
  if (fairino_hardware::test::sdk.allow_deactivate) {
    fairino_hardware::test::sdk.operations.push_back("close");
    if (fairino_hardware::test::sdk.after_close) fairino_hardware::test::sdk.after_close();
    return 0;
  }
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
    test::sdk.state.rbtEnableState = 1;
    test::sdk.state.motion_done = 1;
    hardware_ = std::make_unique<FairinoHardwareInterface>();
    hardware_->_ptr_robot = std::make_unique<FRRobot>();
    hardware_->_control_mode = 0;
    setCommand(Joints{});
    setMeasured(Joints{});
    prepareFlightRecorder({}, false);
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

  using StreamState = mf_manipulator_msgs::msg::ServoStreamState;
  using StreamControl = mf_manipulator_msgs::srv::ControlServoStream;
  StreamState streamState() const { return hardware_->streamMessage(hardware_->stream_published_); }
  StreamControl::Request request(uint8_t command) {
    const auto state = streamState();
    StreamControl::Request req;
    req.command = command;
    req.session_id = state.session_id;
    req.operation_id = "unit-owner";
    req.binding_id = state.binding_id;
    req.generation = state.generation;
    req.request_id = ++request_sequence_;
    return req;
  }
  StreamControl::Response control(const StreamControl::Request& req) {
    StreamControl::Response response;
    hardware_->handleStreamControl(req, response);
    return response;
  }
  void bindStream() {
    ASSERT_TRUE(initPeriod("0.02"));
    writeCycle();
    const auto req = request(StreamControl::Request::BIND);
    ASSERT_TRUE(control(req).accepted);
    writeCycle();
    ASSERT_EQ(streamState().phase, StreamState::ACTIVE);
  }
  void delayStream(int milliseconds = 65) {
    test::sdk.servo_delay_ms = milliseconds;
    setCommand({0.1, 0, 0, 0, 0, 0});
    writeCycle();
    test::sdk.servo_delay_ms = 0;
    ASSERT_EQ(streamState().reason, StreamState::DELAY);
  }
  void stationaryWindow() {
    for (int i = 0; i < 28; ++i) {
      control(request(StreamControl::Request::QUERY));
      writeCycle();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  uint64_t request_sequence_ = 100;

  void progressCommand(double sequence, double epoch_offset = 0) {
    hardware_->progress_command_ = {1, sequence, hardware_->progress_state_[2] + epoch_offset, 0};
  }
  auto progressState() const { return hardware_->progress_state_; }

  void runProgressAction(int delay_ms, int robot_mode, int repeats = 1,
                         bool delay_endpoint = false, bool streaming_controller = false) {
    using Action = control_msgs::action::FollowJointTrajectory;
    ASSERT_TRUE(initPeriod("0.02", "", "", true));
    test::sdk.state.robot_mode = robot_mode;
    test::sdk.demo = std::make_unique<test::ServoDelayFakeController>(delay_ms);
    test::sdk.demo->injections_remaining = repeats;
    if (delay_endpoint) test::sdk.demo->injection_start_rad = 0.1 - 1e-9;
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    mf_manipulator_controllers::ProgressController controller;
    rclcpp::NodeOptions options;
    std::vector<std::string> names{"j1", "j2", "j3", "j4", "j5", "j6"};
    options.parameter_overrides({
      rclcpp::Parameter("joints", names),
      rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position"}),
      rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position"}),
      rclcpp::Parameter("constraints.goal_time", 1.0),
      rclcpp::Parameter("progress.streaming", streaming_controller),
      rclcpp::Parameter("constraints.j3.goal", delay_endpoint ? 0.01 : 0.0001)});
    ASSERT_EQ(controller.init("progress_integration", "", options), controller_interface::return_type::OK);
    ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    auto commands = hardware_->export_command_interfaces();
    auto states = hardware_->export_state_interfaces();
    std::vector<hardware_interface::LoanedCommandInterface> loan_commands;
    std::vector<hardware_interface::LoanedStateInterface> loan_states;
    for (auto &command : commands) loan_commands.emplace_back(command);
    for (auto &state : states) loan_states.emplace_back(state);
    controller.assign_interfaces(std::move(loan_commands), std::move(loan_states));
    ASSERT_EQ(readOnly(), hardware_interface::return_type::OK);
    ASSERT_EQ(controller.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    auto node = std::make_shared<rclcpp::Node>("progress_client");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(controller.get_node()->get_node_base_interface());
    auto client = rclcpp_action::create_client<Action>(node, "/progress_integration/follow_joint_trajectory");
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
    Action::Goal goal;
    goal.trajectory.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint start, end;
    start.positions.resize(6, 0); end.positions.resize(6, 0);
    end.positions[2] = 0.1; end.time_from_start.nanosec = 600000000;
    goal.trajectory.points = {start, end};
    auto accepted = client->async_send_goal(goal);
    auto previous = std::chrono::steady_clock::now();
    auto tick = [&]() {
      auto now = std::chrono::steady_clock::now();
      const auto period = rclcpp::Duration::from_seconds(std::chrono::duration<double>(now - previous).count());
      previous = now;
      executor.spin_some();
      EXPECT_EQ(hardware_->read(node->now(), period), hardware_interface::return_type::OK);
      EXPECT_EQ(controller.update(node->now(), period), controller_interface::return_type::OK);
      EXPECT_EQ(hardware_->write(node->now(), period), hardware_interface::return_type::OK);
      executor.spin_some();
      std::this_thread::sleep_until(now + std::chrono::milliseconds(20));
    };
    auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(6);
    while (accepted.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
           std::chrono::steady_clock::now() < limit) tick();
    ASSERT_EQ(accepted.wait_for(std::chrono::seconds(0)), std::future_status::ready);
    const auto handle = accepted.get(); ASSERT_NE(handle, nullptr);
    auto result = client->async_get_result(handle);
    // Process the action monitor while ServoJ is still in flight. Even when
    // the endpoint is already within tolerance, success requires its reply.
    test::sdk.before_servo_return = [&]() {
      for (int i = 0; i < 3; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      EXPECT_NE(result.wait_for(std::chrono::seconds(0)), std::future_status::ready);
    };
    while (result.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
           std::chrono::steady_clock::now() < limit) tick();
    ASSERT_EQ(result.wait_for(std::chrono::seconds(0)), std::future_status::ready);
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(result.get().goal_id, handle->get_goal_id());
    EXPECT_EQ(test::sdk.demo->injected, delay_ms > 0);
    EXPECT_EQ(test::sdk.demo->injection_count, delay_ms > 0 ? repeats : 0);
    EXPECT_EQ(test::sdk.stop_calls, 0);
    EXPECT_EQ(streamState().reason, StreamState::NONE);
    double previous_target = 0, previous_end = 0, first_send = 0;
    for (const auto &call : test::sdk.demo->calls) {
      if (call.name != "ServoJ") continue;
      if (first_send == 0) first_send = call.start;
      EXPECT_LE(std::abs(call.target - previous_target), 0.003335);
      if (previous_end > 0) { EXPECT_GE(call.start - previous_end, 0.018); }
      previous_target = call.target; previous_end = call.end;
    }
    if (delay_ms == 0) {
      // A 0.6s trajectory should retain nominal speed when the SDK is fast.
      // Allow action scheduling jitter without accepting a systematic 25Hz gate.
      EXPECT_LT(previous_end - first_send, 0.9);
    }
    EXPECT_NEAR(measured()[2], 0.1, 0.0001);
    EXPECT_DOUBLE_EQ(hardware_->progress_state_[1], hardware_->progress_command_[1]);
    test::sdk.before_servo_return = {};
    controller.get_node()->deactivate(); controller.release_interfaces();
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
  bool initPeriod(const std::string& value, const std::string& button = "",
                  const std::string& guard = "", bool progress = false) {
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
    if (progress) {
      hardware_interface::ComponentInfo gpio;
      gpio.name = "servo_progress";
      info.gpios.push_back(gpio);
    }
    if (!guard.empty()) info.hardware_parameters["servo_delay_guard_enabled"] = guard;
    if (!value.empty()) info.hardware_parameters["servo_command_period_sec"] = value;
    if (!button.empty()) info.hardware_parameters["flange_drag_button_enabled"] = button;
    return hardware_->on_init(info) == hardware_interface::CallbackReturn::SUCCESS;
  }
  auto diagnostics() const { return hardware_->published_diagnostics_; }
  void prepareDiagnosticPublisher(const std::string& node_namespace = "", size_t domain = 231) {
    ASSERT_EQ(rcutils_logging_initialize(), RCUTILS_RET_OK);
    diagnostic_context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(domain);
    init_options.auto_initialize_logging(false);
    diagnostic_context_->init(0, nullptr, init_options);
    rclcpp::NodeOptions options;
    options.context(diagnostic_context_).enable_rosout(false).start_parameter_services(false);
    hardware_->_svc_node = std::make_shared<rclcpp::Node>(
        "fairino_fault_log_test", node_namespace, options);
    hardware_->createDiagnosticInterfaces();
  }
  void runDelayRecoveryDemo() {
    const char* output = std::getenv("MF_SERVO_DEMO_OUTPUT");
    const char* domain_text = std::getenv("MF_SERVO_DEMO_DOMAIN");
    ASSERT_NE(output, nullptr);
    ASSERT_NE(domain_text, nullptr);
    const char* namespace_text = std::getenv("MF_SERVO_DEMO_NAMESPACE");
    const char* delay_text = std::getenv("MF_SERVO_DEMO_DELAY_MS");
    const char* guarded_text = std::getenv("MF_SERVO_DEMO_GUARDED");
    ASSERT_NE(namespace_text, nullptr);
    ASSERT_NE(delay_text, nullptr);
    ASSERT_NE(guarded_text, nullptr);
    ASSERT_EQ(std::string(namespace_text).find("/servo_delay_demo_"), 0u);
    const int domain = std::stoi(domain_text);
    ASSERT_GE(domain, 220);  // Refuse the robot's operational domain (171).
    ASSERT_LE(domain, 232);
    const int delay = std::stoi(delay_text);
    ASSERT_GE(delay, 0);
    ASSERT_LE(delay, 1500);
    const bool guarded = std::string(guarded_text) == "1";
    ASSERT_TRUE(initPeriod("0.02", "", guarded ? "true" : "false"));
    test::sdk.demo = std::make_unique<test::ServoDelayFakeController>(delay);
    prepareDiagnosticPublisher(namespace_text, domain);
    hardware_->createStreamInterfaces();
    std::mutex commands_mutex;
    Joints commands{};
    std::atomic<bool> finished{false};
    auto subscription = hardware_->_svc_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "sim_joint_command", 1, [&](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          if (msg->data.size() != 6 || !std::all_of(msg->data.begin(), msg->data.end(),
              [](double x) { return std::isfinite(x); })) return;
          std::lock_guard<std::mutex> lock(commands_mutex);
          std::copy_n(msg->data.begin(), 6, commands.begin());
        });
    auto done = hardware_->_svc_node->create_service<std_srvs::srv::Trigger>(
        "finish_demo", [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          finished.store(true);
          response->success = true;
        });
    hardware_->startServiceThread();
    std::ofstream trace(std::string(output) + "/driver_cycles.csv");
    trace << std::setprecision(12)
          << "monotonic_s,phase,reason,recoveries,command_j3_rad,measured_j3_rad,servo_calls\n";
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (!finished.load() && std::chrono::steady_clock::now() < end) {
      {
        std::lock_guard<std::mutex> lock(commands_mutex);
        setCommand(commands);
      }
      writeCycle();
      const auto state = hardware_->stream_control_;
      trace << test::ServoDelayFakeController::now() << ',' << static_cast<int>(state.phase)
            << ',' << static_cast<int>(state.reason) << ',' << static_cast<int>(state.recoveries)
            << ',' << hardware_->_jnt_position_command[2] << ',' << measured()[2]
            << ',' << test::sdk.servo_commands.size() << '\n';
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    hardware_->stopServiceThread();
    test::sdk.demo->save(output);
    EXPECT_TRUE(finished.load()) << "Python recovery client did not finish within 30 seconds";
    EXPECT_EQ(test::sdk.reset_calls, 0);
  }
  void publishDiagnosticSnapshot() { hardware_->publishDiagnostics(); }
  std::string trajectoryStatusTopic() const {
    return hardware_->trajectory_status_sub_->get_topic_name();
  }
  void trajectoryStatus(int8_t state, uint8_t id = 1) {
    action_msgs::msg::GoalStatusArray statuses;
    action_msgs::msg::GoalStatus status;
    status.status = state;
    status.goal_info.goal_id.uuid[15] = id;
    statuses.status_list.push_back(status);
    hardware_->logTrajectoryAborts(statuses);
  }
  void publishStaleSnapshot(double age_sec) {
    hardware_->logPendingFaults(diagnostics(), age_sec);
  }
  void startBackgroundDiagnostics() {
    // Make the final drain deterministic: no timer may consume the event.
    hardware_->diagnostics_timer_->cancel();
    hardware_->startServiceThread();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!hardware_->_svc_exec->is_spinning() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(hardware_->_svc_exec->is_spinning());
  }
  void stopBackgroundDiagnostics() {
    test::sdk.allow_deactivate = true;
    EXPECT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()),
              hardware_interface::CallbackReturn::SUCCESS);
  }
  void requestDrag(bool enable) { hardware_->_drag_req.store(enable ? 1 : -1); }
  bool dragActive() const { return hardware_->_drag_active; }
  void latchHardInhibit() { hardware_->hard_inhibit_.store(true); }
  void requestResumableFlush() {
    hardware_->flush_resume_requested_.store(true);
    requestFlush();
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
        hardware_->flange_drag_button_enabled_ = true;
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

  void prepareFlightRecorder(FlightRecorder::IncidentCallback callback = {}, bool start = true) {
    // Both paths are test-private; fake metadata cannot attach the live host recorder.
    if (trace_directory_.empty()) {
      char path[] = "/tmp/mf-fairino-driver-flight-test-XXXXXX";
      ASSERT_NE(mkdtemp(path), nullptr);
      trace_directory_ = path;
    }
    FlightRecorderOptions options;
    options.output_directory = trace_directory_ + "/traces";
    options.metadata_directory = trace_directory_ + "/metadata";
    options.post_window_ns = 20000000;
    options.correlation_window_ns = 1000000000;
    options.queue_capacity = 256;
    options.history_capacity = 256;
    options.incident_capacity = 512;
    hardware_->flight_recorder_ = std::make_unique<FlightRecorder>(options);
    if (start) hardware_->flight_recorder_->start(std::move(callback));
  }

  std::string finishFlightRecorder() {
    hardware_->flight_recorder_->shutdown();
    std::string result;
    const auto directory = std::filesystem::path(trace_directory_) / "traces";
    if (std::filesystem::exists(directory)) {
      for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() != ".jsonl") continue;
        std::ifstream input(entry.path());
        result.append(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
      }
    }
    return result;
  }
  FlightRecorderStats flightStats() const { return hardware_->flight_recorder_->stats(); }

  void TearDown() override {
    // Join the independent writer before removing its test-private files.
    hardware_.reset();
    if (!trace_directory_.empty()) std::filesystem::remove_all(trace_directory_);
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
  std::shared_ptr<rclcpp::Context> diagnostic_context_;
  std::unique_ptr<FairinoHardwareInterface> hardware_;
  std::string trace_directory_;
};

// Opt-in component integration harness, launched only by servo_delay_recovery_demo.py.
TEST_F(FairinoHardwareInterfaceTest, DISABLED_RosServoDelayRecoveryDemo) {
  runDelayRecoveryDemo();
}

TEST_F(FairinoHardwareInterfaceTest, GuardSafetyStopReleaseDoesNotResumeOwnedOldTrajectory) {
  bindStream();
  const auto count = test::sdk.servo_commands.size();
  test::sdk.state.safety_stop0_state = 1;
  writeCycle();
  test::sdk.state.safety_stop0_state = 0;
  setCommand({0.2, 0, 0, 0, 0, 0});
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), count);
  EXPECT_EQ(streamState().reason, StreamState::UNSAFE);
  EXPECT_TRUE(flushBlocked());
}

TEST_F(FairinoHardwareInterfaceTest, GuardDelaySamplesNeverSendCatchupOrResetErrors) {
  for (int delay : {65, 200, 1020}) {
    SetUp();
    ASSERT_TRUE(initPeriod("0.02"));
    test::sdk.servo_delay_ms = delay;
    writeCycle();
    ASSERT_EQ(test::sdk.servo_commands.size(), 1u);
    test::sdk.servo_delay_ms = 0;
    setCommand({0.3, 0, 0, 0, 0, 0});
    writeCycle();
    EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
    EXPECT_EQ(test::sdk.reset_calls, 0);
    EXPECT_TRUE(flushBlocked());
    EXPECT_EQ(streamState().reason, StreamState::DELAY);
  }
}

TEST_F(FairinoHardwareInterfaceTest, GuardSubthresholdCallAndIdleHoldGapContinue) {
  ASSERT_TRUE(initPeriod("0.02"));
  test::sdk.servo_delay_ms = 30;
  writeCycle();
  test::sdk.servo_delay_ms = 0;
  setCommand({0.1, 0, 0, 0, 0, 0});
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), 2u);
  std::this_thread::sleep_for(std::chrono::milliseconds(65));
  writeCycle();  // Unchanged target while idle does not constitute a missed stream.
  EXPECT_FALSE(flushBlocked());
}

TEST_F(FairinoHardwareInterfaceTest, GuardMissedActiveCadenceBlocksBeforeSending) {
  bindStream();
  const auto count = test::sdk.servo_commands.size();
  std::this_thread::sleep_for(std::chrono::milliseconds(65));
  setCommand({0.2, 0, 0, 0, 0, 0});
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), count);
  EXPECT_EQ(streamState().reason, StreamState::DELAY);
}

TEST_F(FairinoHardwareInterfaceTest, GuardQueueZeroStillRequiresMeasuredRest) {
  bindStream();
  delayStream();
  test::sdk.state.robot_state = 2;
  test::sdk.state.motion_done = 0;
  stationaryWindow();
  EXPECT_EQ(streamState().phase, StreamState::STOPPING);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  test::sdk.state.robot_state = 1;
  test::sdk.state.motion_done = 1;
  stationaryWindow();
  EXPECT_EQ(streamState().phase, StreamState::QUIESCENT);
  EXPECT_TRUE(flushBlocked());
  auto stop = std::find(test::sdk.operations.begin(), test::sdk.operations.end(), "stop");
  ASSERT_NE(stop, test::sdk.operations.end());
  ASSERT_GE(std::distance(stop, test::sdk.operations.end()), 3);
  EXPECT_EQ(*std::next(stop), "clear");
  EXPECT_EQ(*std::next(stop, 2), "end");
}

TEST_F(FairinoHardwareInterfaceTest, GuardMissingVelocityUsesPositionWindowButMixedNaNBlocks) {
  bindStream();
  delayStream();
  for (auto& velocity : test::sdk.state.actual_qd) velocity = std::numeric_limits<double>::quiet_NaN();
  stationaryWindow();
  EXPECT_EQ(streamState().phase, StreamState::QUIESCENT);
  // A second independent owner fixture tests malformed partial feedback.
  SetUp();
  bindStream();
  delayStream();
  test::sdk.state.actual_qd[0] = std::numeric_limits<double>::quiet_NaN();
  stationaryWindow();
  EXPECT_EQ(streamState().phase, StreamState::BLOCKED);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, GuardMeasuredMotionWithMissingVelocityDoesNotRearm) {
  bindStream();
  delayStream();
  for (auto& velocity : test::sdk.state.actual_qd) velocity = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 30; ++i) {
    setMeasured({i * 0.002, 0, 0, 0, 0, 0});
    writeCycle();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(streamState().phase, StreamState::STOPPING);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, GuardRearmFirstTargetIsMeasuredHoldAndDuplicateIsIdempotent) {
  bindStream();
  delayStream();
  setMeasured({0.07, 0, 0, 0, 0, 0});
  stationaryWindow();
  ASSERT_EQ(streamState().phase, StreamState::QUIESCENT);
  setCommand(measured()); // JTC hold replacement acknowledged by upper owner.
  auto req = request(StreamControl::Request::REARM);
  req.hold_position = measured();
  ASSERT_TRUE(control(req).accepted);
  const auto count = test::sdk.servo_commands.size();
  for (int i = 0; i < 3; ++i) writeCycle();
  ASSERT_EQ(streamState().phase, StreamState::ACTIVE);
  EXPECT_EQ(streamState().recovery_count, 1);
  EXPECT_EQ(streamState().completed_request_id, req.request_id);
  ASSERT_EQ(test::sdk.servo_commands.size(), count + 1);
  EXPECT_NEAR(test::sdk.servo_commands.back().jPos[0], 0.07 * 180 / M_PI, 1e-6);
  ASSERT_TRUE(control(req).completed);
  writeCycle();
  EXPECT_EQ(test::sdk.servo_start_calls, 1);
  test::sdk.servo_delay_ms = 65;
  setCommand({0.08, 0, 0, 0, 0, 0});
  writeCycle();
  test::sdk.servo_delay_ms = 0;
  stationaryWindow();
  EXPECT_EQ(streamState().phase, StreamState::BLOCKED);
  EXPECT_EQ(test::sdk.servo_start_calls, 1);
}

TEST_F(FairinoHardwareInterfaceTest, GuardStaleJtcCommandRejectsRearm) {
  bindStream();
  delayStream();
  stationaryWindow();
  auto req = request(StreamControl::Request::REARM);
  req.hold_position = measured();
  ASSERT_TRUE(control(req).accepted);
  writeCycle();
  EXPECT_NE(streamState().request_code, 0);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  EXPECT_TRUE(flushBlocked());
}

TEST_F(FairinoHardwareInterfaceTest, GuardCancelWinsOverPendingRearmAndOldSession) {
  bindStream();
  delayStream();
  stationaryWindow();
  setCommand(measured());
  auto req = request(StreamControl::Request::REARM);
  req.hold_position = measured();
  auto invalid = req;
  invalid.session_id = "old-session";
  EXPECT_FALSE(control(invalid).accepted);
  ASSERT_TRUE(control(req).accepted);
  auto cancel = request(StreamControl::Request::QUIESCE);
  control(cancel); // Atomic cancel intent wins even if mailbox is occupied.
  writeCycle();
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  EXPECT_EQ(streamState().reason, StreamState::CANCELED);
  EXPECT_TRUE(flushBlocked());
}

TEST_F(FairinoHardwareInterfaceTest, GuardOwnerDisappearanceBlocksWithoutReset) {
  bindStream();
  std::this_thread::sleep_for(std::chrono::milliseconds(1050));
  writeCycle();
  EXPECT_EQ(streamState().reason, StreamState::OWNER_LOST);
  EXPECT_TRUE(flushBlocked());
  EXPECT_EQ(test::sdk.reset_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, SlowSuccessfulServoMustNotSendCatchupTarget) {
  ASSERT_TRUE(initPeriod("0.02"));
  test::sdk.state.rbtEnableState = 1;
  test::sdk.state.motion_done = 1;
  test::sdk.servo_delay_ms = 65;
  writeCycle();
  test::sdk.servo_delay_ms = 0;
  setCommand(Joints{0.3, 0, 0, 0, 0, 0});
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), 1U);
  EXPECT_EQ(test::sdk.reset_calls, 0);
  EXPECT_TRUE(flushBlocked());
}

TEST_F(FairinoHardwareInterfaceTest, FlightRecorderHealthyCyclesStaySilentWithoutExtraSdk) {
  std::atomic<int> incidents{0};
  prepareFlightRecorder([&](const IncidentInfo&) { ++incidents; });
  auto next_cycle = std::chrono::steady_clock::now();
  for (int cycle = 0; cycle < 10; ++cycle) {
    // An unpaced fake loop really is a burst and must trigger the recorder.
    std::this_thread::sleep_until(next_cycle);
    next_cycle += std::chrono::milliseconds(20);
    setCommand(Joints{cycle * .01, 0, 0, 0, 0, 0});
    writeCycle();
  }
  EXPECT_TRUE(finishFlightRecorder().empty());
  EXPECT_EQ(incidents, 0);
  EXPECT_EQ(test::sdk.state_calls, 10);
  EXPECT_EQ(test::sdk.servo_commands.size(), 10u);
  EXPECT_EQ(test::sdk.stop_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, FlightRecorderSuccessfulSlowServoKeepsFullStateAndBothTargets) {
  prepareFlightRecorder();
  test::sdk.state.actual_qdd[0] = 123;
  test::sdk.state.jointDriverTorque[5] = 19;
  test::sdk.state.cl_dgt_input_h = 18;
  test::sdk.state.rbtEnableState = 1;
  writeCycle();
  test::sdk.servo_delay_ms = 60;
  setCommand(displaced_);
  writeCycle();
  const std::string trace = finishFlightRecorder();
  EXPECT_NE(trace.find("RPC_SLOW"), std::string::npos);
  EXPECT_NE(trace.find("\"rpc\":\"SERVO\",\"rc\":0"), std::string::npos);
  EXPECT_NE(trace.find("\"actual_qdd\":[123,"), std::string::npos);
  EXPECT_NE(trace.find("\"cl_dgt_input_h\":18"), std::string::npos);
  EXPECT_NE(trace.find("\"rbtEnableState\":1"), std::string::npos);
  EXPECT_NE(trace.find("\"raw_sdk_hex\":"), std::string::npos);
  EXPECT_NE(trace.find("\"attempted_target_valid\":true"), std::string::npos);
  EXPECT_NE(trace.find("\"successful_target_valid\":true"), std::string::npos);
  EXPECT_EQ(flightStats().incidents_started, 1u);
  EXPECT_EQ(test::sdk.state_calls, 2);
  EXPECT_EQ(test::sdk.servo_commands.size(), 2u);
  EXPECT_EQ(test::sdk.stop_calls, 0);
  EXPECT_EQ(test::sdk.reset_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, FlightRecorderInterpolationPeriodDoesNotChangeCadenceDetection) {
  ASSERT_TRUE(initPeriod("0.1"));
  std::atomic<int> incidents{0};
  prepareFlightRecorder([&](const IncidentInfo&) { ++incidents; });
  auto next_cycle = std::chrono::steady_clock::now();
  for (int cycle = 0; cycle < 5; ++cycle) {
    std::this_thread::sleep_until(next_cycle);
    next_cycle += std::chrono::milliseconds(20);
    setCommand(Joints{cycle * .01, 0, 0, 0, 0, 0});
    writeCycle();
  }
  EXPECT_TRUE(finishFlightRecorder().empty());
  EXPECT_EQ(incidents, 0);
  EXPECT_FLOAT_EQ(test::sdk.command_period, 0.1F);
  EXPECT_EQ(test::sdk.state_calls, 5);
  EXPECT_EQ(test::sdk.servo_commands.size(), 5u);
}

TEST_F(FairinoHardwareInterfaceTest, FlightRecorderSafetyAndActionAbortShareOneIncident) {
  prepareFlightRecorder();
  writeCycle();
  test::sdk.state.safety_stop0_state = 1;
  writeCycle();
  trajectoryStatus(action_msgs::msg::GoalStatus::STATUS_ABORTED, 9);
  const auto trace = finishFlightRecorder();
  EXPECT_NE(trace.find("SAFETY_FAULT"), std::string::npos);
  EXPECT_NE(trace.find("ACTION_ABORT"), std::string::npos);
  EXPECT_NE(trace.find("00000000000000000000000000000009"), std::string::npos);
  EXPECT_EQ(flightStats().incidents_started, 1u);
  EXPECT_EQ(test::sdk.stop_calls, 1);
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
}

TEST_F(FairinoHardwareInterfaceTest, FlightRecorderSlowWriterCannotDelaySdkStopAndClose) {
  std::promise<void> callback_started;
  std::promise<void> release_writer;
  auto release = release_writer.get_future().share();
  prepareFlightRecorder([&](const IncidentInfo&) {
    callback_started.set_value();
    release.wait();
  });
  test::sdk.servo_delay_ms = 60;
  writeCycle();
  const bool writer_blocked = callback_started.get_future().wait_for(std::chrono::seconds(2)) ==
      std::future_status::ready;
  EXPECT_TRUE(writer_blocked);
  std::promise<void> sdk_closed;
  auto closed = sdk_closed.get_future();
  test::sdk.allow_deactivate = true;
  test::sdk.after_close = [&]() { sdk_closed.set_value(); };
  auto deactivate = std::async(std::launch::async, [&]() {
    return hardware_->on_deactivate(rclcpp_lifecycle::State());
  });
  const bool stopped_first = closed.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  EXPECT_TRUE(stopped_first);
  // Always release, even on failure, so a failed test cannot deadlock teardown.
  release_writer.set_value();
  EXPECT_EQ(deactivate.get(), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(test::sdk.operations, (std::vector<std::string>{"servo", "end", "stop", "close"}));
}

TEST_F(FairinoHardwareInterfaceTest, HealthyMotionDoesNotWriteFaultLogsOrCallExtraSdk) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  for (int cycle = 0; cycle < 10; ++cycle) {
    setCommand(Joints{cycle * 0.01, 0, 0, 0, 0, 0});
    writeCycle();
    publishDiagnosticSnapshot();
  }
  EXPECT_TRUE(logs.messages.empty());
  EXPECT_EQ(test::sdk.state_calls, 10);
  EXPECT_EQ(test::sdk.servo_commands.size(), 10u);
  EXPECT_EQ(test::sdk.stop_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, TransientControllerFaultIsLoggedAfterItHasCleared) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  test::sdk.state.main_code = 14;
  test::sdk.state.sub_code = 2;
  test::sdk.state.collisionState = 1;
  test::sdk.state.EmergencyStop = 1;
  test::sdk.state.mc_queue_len = 5;
  test::sdk.state.servoJCmdNum = 73;
  test::sdk.state.actual_qd[0] = 12.5;
  setMeasured(displaced_);
  setCommand(Joints{});
  writeCycle();
  EXPECT_TRUE(logs.messages.empty()) << "RT read/write must not format or emit logs";
  test::sdk.state.main_code = 0;
  test::sdk.state.sub_code = 0;
  test::sdk.state.collisionState = 0;
  test::sdk.state.EmergencyStop = 0;
  writeCycle();
  publishDiagnosticSnapshot();
  ASSERT_EQ(logs.messages.size(), 1u);
  const auto& message = logs.messages.front();
  for (const auto* field : {"event=sdk_fault", "main_code=14", "sub_code=2",
                            "collision=1", "emergency_stop=1", "mc_queue_len=5",
                            "servoJCmdNum=73", "sdk_joint_position_deg=[",
                            "sdk_joint_velocity_deg_sec=[12.500000",
                            "command_rad=[", "last_servo_target_rad=[",
                            "measured_rad=[", "snapshot_mono_ns=", "frame_age_sec="}) {
    EXPECT_NE(message.find(field), std::string::npos) << field << " in " << message;
  }
  EXPECT_EQ(test::sdk.state_calls, 2);
}

TEST_F(FairinoHardwareInterfaceTest, PersistentFaultLogsOnceAndRecurrenceLogsAgain) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  test::sdk.state.main_code = 14;
  for (int cycle = 0; cycle < 20; ++cycle) {
    writeCycle();
    publishDiagnosticSnapshot();
  }
  ASSERT_EQ(logs.messages.size(), 1u);
  test::sdk.state.sub_code = 8;
  writeCycle();
  publishDiagnosticSnapshot();
  ASSERT_EQ(logs.messages.size(), 2u);
  test::sdk.state.main_code = 0;
  test::sdk.state.sub_code = 0;
  writeCycle();
  publishDiagnosticSnapshot();
  EXPECT_EQ(logs.messages.size(), 2u) << "recovery must not add routine logs";
  test::sdk.state.main_code = 14;
  writeCycle();
  publishDiagnosticSnapshot();
  EXPECT_EQ(logs.messages.size(), 3u);
}

TEST_F(FairinoHardwareInterfaceTest, FailedStateReadLogsLastObservedStateWithoutExtraRpc) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  setMeasured(displaced_);
  writeCycle();
  test::sdk.state_result = -2;
  EXPECT_EQ(readOnly(), hardware_interface::return_type::ERROR);
  publishDiagnosticSnapshot();
  ASSERT_EQ(logs.messages.size(), 1u);
  EXPECT_NE(logs.messages[0].find("state_rpc_rc=-2"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("state_valid=0"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("sdk_state_observed=1"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("sdk_sample_age_sec="), std::string::npos);
  EXPECT_EQ(test::sdk.state_calls, 2);
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
}

TEST_F(FairinoHardwareInterfaceTest, ServoFailureKeepsAttemptedTargetAndReturnCode) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  test::sdk.servo_result = 14;
  setCommand(displaced_);
  writeCycle();
  EXPECT_TRUE(logs.messages.empty());
  publishDiagnosticSnapshot();
  ASSERT_EQ(logs.messages.size(), 1u);
  EXPECT_NE(logs.messages[0].find("ServoJ_rc=14"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("last_servo_target_rad=[0.300000,-0.200000"),
            std::string::npos);
  EXPECT_NE(logs.messages[0].find("ServoJ_duration_ms="), std::string::npos);
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
}

TEST_F(FairinoHardwareInterfaceTest, TrajectoryAbortLogsOncePerGoalWithHealthySdkState) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  writeCycle();
  for (int8_t status : {action_msgs::msg::GoalStatus::STATUS_ACCEPTED,
                        action_msgs::msg::GoalStatus::STATUS_EXECUTING,
                        action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
                        action_msgs::msg::GoalStatus::STATUS_CANCELED}) {
    trajectoryStatus(status);
  }
  EXPECT_TRUE(logs.messages.empty());
  trajectoryStatus(action_msgs::msg::GoalStatus::STATUS_ABORTED);
  trajectoryStatus(action_msgs::msg::GoalStatus::STATUS_ABORTED);
  ASSERT_EQ(logs.messages.size(), 1u);
  EXPECT_NE(logs.messages[0].find("event=trajectory_aborted"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("goal_uuid=00000000000000000000000000000001"),
            std::string::npos);
  EXPECT_NE(logs.messages[0].find("main_code=0"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("snapshot_age_sec="), std::string::npos);
  trajectoryStatus(action_msgs::msg::GoalStatus::STATUS_ABORTED, 2);
  EXPECT_EQ(logs.messages.size(), 2u);
  EXPECT_EQ(test::sdk.state_calls, 1);
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
}

TEST_F(FairinoHardwareInterfaceTest, GuardAdmitsManualModeWithoutChangingRobotMode) {
  ASSERT_TRUE(initPeriod("0.02"));
  test::sdk.state.robot_mode = 1;
  test::sdk.state.program_state = 1;
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
  EXPECT_EQ(test::sdk.stop_calls, 0);
  EXPECT_EQ(streamState().reason, StreamState::NONE);
}

TEST_F(FairinoHardwareInterfaceTest, RealJtcRetainsGoalThrough200msReplyInManualMode) {
  runProgressAction(200, 1);
}

TEST_F(FairinoHardwareInterfaceTest, RealJtcFastRepliesRetainNominalTrajectorySpeed) {
  runProgressAction(0, 1);
}

TEST_F(FairinoHardwareInterfaceTest, RealJtcRetainsGoalThroughOneSecondReplyInAutomaticMode) {
  runProgressAction(1000, 0);
}

TEST_F(FairinoHardwareInterfaceTest, RealJtcRetainsGoalThroughRepeatedDelayedReplies) {
  runProgressAction(200, 1, 3);
}

TEST_F(FairinoHardwareInterfaceTest, RealJtcWaitsForDelayedEndpointReplyBeforeSuccess) {
  runProgressAction(200, 1, 1, true);
}

TEST_F(FairinoHardwareInterfaceTest, RealJtcEndpointDelayUsesExtensionBeforeSettlingBudget) {
  runProgressAction(1200, 1, 1, true);
}

TEST_F(FairinoHardwareInterfaceTest, RealJtcFixedGoalOnServoControllerDoesNotSkipAfterDelay) {
  runProgressAction(200, 1, 1, false, true);
}

TEST_F(FairinoHardwareInterfaceTest, DelayedAcknowledgementRequiresNewFeedbackAndPacing) {
  ASSERT_TRUE(initPeriod("0.02", "", "", true));
  ASSERT_EQ(readOnly(), hardware_interface::return_type::OK);
  progressCommand(1);
  setCommand({0.01, 0, 0, 0, 0, 0});
  test::sdk.servo_delay_ms = 200;
  ASSERT_EQ(writeOnly(), hardware_interface::return_type::OK);
  EXPECT_EQ(progressState()[1], 1);
  EXPECT_EQ(streamState().reason, StreamState::NONE);
  EXPECT_EQ(test::sdk.stop_calls, 0);
  progressCommand(2);
  setCommand({0.02, 0, 0, 0, 0, 0});
  test::sdk.servo_delay_ms = 0;
  writeCycle();  // First cache observation after reply cannot authorize motion.
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
  test::sdk.advance_frame = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
  test::sdk.advance_frame = true;
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), 2u);
  EXPECT_EQ(progressState()[1], 2);
}

TEST_F(FairinoHardwareInterfaceTest, FreshReadMayPrepareNextPointBeforeWireSlot) {
  ASSERT_TRUE(initPeriod("0.02", "", "", true));
  test::sdk.demo = std::make_unique<test::ServoDelayFakeController>(0);
  ASSERT_EQ(readOnly(), hardware_interface::return_type::OK);
  progressCommand(1);
  setCommand({0.01, 0, 0, 0, 0, 0});
  writeOnly();
  ASSERT_EQ(progressState()[1], 1);
  // read() precedes update()/write(). Gating this fresh read on the previous
  // wire timestamp rounds a small phase difference up to an entire 20ms tick.
  ASSERT_EQ(readOnly(), hardware_interface::return_type::OK);
  ASSERT_EQ(progressState()[0], 1);
  progressCommand(2);
  setCommand({0.02, 0, 0, 0, 0, 0});
  writeOnly();
  ASSERT_EQ(progressState()[1], 2);
  ASSERT_EQ(test::sdk.demo->calls.size(), 2u);
  EXPECT_GE(test::sdk.demo->calls[1].start - test::sdk.demo->calls[0].start, 0.02);
}

TEST_F(FairinoHardwareInterfaceTest, StopRequestedDuringWirePacingBlocksNextServo) {
  ASSERT_TRUE(initPeriod("0.02", "", "", true));
  ASSERT_EQ(readOnly(), hardware_interface::return_type::OK);
  progressCommand(1);
  setCommand({0.01, 0, 0, 0, 0, 0});
  writeOnly();
  ASSERT_EQ(readOnly(), hardware_interface::return_type::OK);
  progressCommand(2);
  setCommand({0.02, 0, 0, 0, 0, 0});
  auto stopped = std::async(std::launch::async, [this]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    requestFlush();
  });
  writeOnly();
  stopped.get();
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
  EXPECT_EQ(test::sdk.stop_calls, 1);
  EXPECT_EQ(progressState()[1], 1);
}

TEST_F(FairinoHardwareInterfaceTest, UnacknowledgedOrOldEpochCannotReplayTargets) {
  ASSERT_TRUE(initPeriod("0.02", "", "", true));
  ASSERT_EQ(readOnly(), hardware_interface::return_type::OK);
  progressCommand(0);
  setCommand({0.01, 0, 0, 0, 0, 0});
  writeOnly();
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  progressCommand(1, -1);
  writeCycle();
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  EXPECT_EQ(streamState().reason, StreamState::UNSAFE);
}

TEST_F(FairinoHardwareInterfaceTest, NormalReplyContinuesThroughCapturedJ2TrackingSample) {
  ASSERT_TRUE(initPeriod("0.02", "", "", true));
  // J2 samples from the stop on the first test_e move, 2026-11-18 15:42:28.
  constexpr double previous = -1.8353575508;
  constexpr double next = -1.8024577318;
  setMeasured({0, previous, 0, 0, 0, 0});
  progressCommand(1);
  setCommand({0, previous, 0, 0, 0, 0});
  writeCycle();
  ASSERT_EQ(progressState()[1], 1);

  setMeasured({0, -110.93602753 * M_PI / 180.0, 0, 0, 0, 0});
  test::sdk.state.actual_qd[1] = 70.16021426;
  progressCommand(2);
  setCommand({0, next, 0, 0, 0, 0});
  writeCycle();
  ASSERT_EQ(test::sdk.servo_commands.size(), 2u);
  EXPECT_NEAR(test::sdk.servo_commands.back().jPos[1], next * 180.0 / M_PI, 1e-6);
  EXPECT_EQ(progressState()[1], 2);
  EXPECT_EQ(test::sdk.stop_calls, 0);
  EXPECT_EQ(streamState().reason, StreamState::NONE);
}

TEST_F(FairinoHardwareInterfaceTest, SdkErrorNeverAcknowledgesOrAutomaticallyResets) {
  ASSERT_TRUE(initPeriod("0.02", "", "", true));
  ASSERT_EQ(readOnly(), hardware_interface::return_type::OK);
  progressCommand(1);
  test::sdk.servo_result = -7;
  writeOnly();
  EXPECT_EQ(progressState()[1], 0);
  EXPECT_EQ(streamState().reason, StreamState::SDK_ERROR);
  EXPECT_EQ(test::sdk.reset_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, RunningServoProgramIsHealthyInBothModes) {
  ASSERT_TRUE(initPeriod("0.02"));
  test::sdk.state.program_state = 2;
  test::sdk.state.robot_state = 2;
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), 1u);
  EXPECT_EQ(test::sdk.stop_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, SafetyFaultIsReportedButNormalStopAndManualModeAreQuiet) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  test::sdk.state.robot_mode = 1;
  test::sdk.state.program_state = 1;
  test::sdk.state.motion_done = 1;
  writeCycle();
  publishDiagnosticSnapshot();
  EXPECT_TRUE(logs.messages.empty());
  test::sdk.state.safety_stop0_state = 1;
  writeCycle();
  publishDiagnosticSnapshot();
  ASSERT_EQ(logs.messages.size(), 1u);
  EXPECT_NE(logs.messages[0].find("safety_stop0=1"), std::string::npos);
  writeCycle();
  publishDiagnosticSnapshot();
  EXPECT_EQ(logs.messages.size(), 1u);
  EXPECT_EQ(test::sdk.stop_calls, 1);
}

TEST_F(FairinoHardwareInterfaceTest, InvalidRawFeedbackStillRetainsItsControllerFault) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  test::sdk.state.main_code = 23;
  test::sdk.state.jt_cur_pos[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(readOnly(), hardware_interface::return_type::ERROR);
  publishDiagnosticSnapshot();
  ASSERT_EQ(logs.messages.size(), 1u);
  EXPECT_NE(logs.messages[0].find("main_code=23"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("state_valid=0"), std::string::npos);
  EXPECT_EQ(test::sdk.state_calls, 1);
  EXPECT_TRUE(test::sdk.servo_commands.empty());
}

TEST_F(FairinoHardwareInterfaceTest, FaultSnapshotOverflowIsVisibleAndDrainingIsBounded) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  for (int code = 1; code <= 100; ++code) {
    test::sdk.state.main_code = code;
    writeCycle();
  }
  EXPECT_TRUE(logs.messages.empty());
  publishDiagnosticSnapshot();
  ASSERT_GT(logs.messages.size(), 1u);
  EXPECT_LT(logs.messages.size(), 100u);
  EXPECT_NE(logs.messages[0].find("event=sdk_fault_log_overflow"), std::string::npos);
  const size_t previous_count = logs.messages.size();
  publishDiagnosticSnapshot();
  EXPECT_EQ(logs.messages.size(), previous_count);
  test::sdk.state.main_code = 101;
  writeCycle();
  publishDiagnosticSnapshot();
  EXPECT_EQ(logs.messages.size(), previous_count + 1);
}

TEST_F(FairinoHardwareInterfaceTest, StaleControlSnapshotLogsOnceUntilSamplingResumes) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  writeCycle();
  publishStaleSnapshot(0.3);
  publishStaleSnapshot(0.4);
  ASSERT_EQ(logs.messages.size(), 1u);
  EXPECT_NE(logs.messages[0].find("event=control_snapshot_stale"), std::string::npos);
  publishStaleSnapshot(0.0);
  EXPECT_EQ(logs.messages.size(), 1u);
  publishStaleSnapshot(0.3);
  EXPECT_EQ(logs.messages.size(), 2u);
}

TEST_F(FairinoHardwareInterfaceTest, AbortObserverUsesFollowJointTrajectoryActionInArmNamespace) {
  prepareDiagnosticPublisher("/manipulator");
  EXPECT_EQ(trajectoryStatusTopic(),
            "/manipulator/joint_trajectory_controller/follow_joint_trajectory/_action/status");
}

TEST_F(FairinoHardwareInterfaceTest, ModeFailureKeepsFirstRpcErrorEvenWhenStopRetrySucceeds) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  test::sdk.end_results = {14, 0};
  requestDrag(true);
  writeCycle();
  publishDiagnosticSnapshot();
  ASSERT_EQ(logs.messages.size(), 1u);
  EXPECT_NE(logs.messages[0].find("ServoMoveEnd_last_error_rc=14"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("ServoMoveEnd_rc=0"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("drag_transition_failed=1"), std::string::npos);
  EXPECT_EQ(test::sdk.servo_end_calls, 2);
  EXPECT_EQ(test::sdk.stop_calls, 1);
  EXPECT_EQ(test::sdk.clear_calls, 1);
}

TEST_F(FairinoHardwareInterfaceTest, BackgroundShutdownDrainsFaultBeforeFirstTimerTick) {
  prepareDiagnosticPublisher();
  test::FaultLogCapture logs;
  test::sdk.state_result = -2;
  EXPECT_EQ(readOnly(), hardware_interface::return_type::ERROR);
  EXPECT_TRUE(logs.messages.empty());
  startBackgroundDiagnostics();
  stopBackgroundDiagnostics();
  ASSERT_EQ(logs.messages.size(), 1u);
  EXPECT_NE(logs.messages[0].find("event=sdk_fault"), std::string::npos);
  EXPECT_NE(logs.messages[0].find("state_rpc_rc=-2"), std::string::npos);
  EXPECT_EQ(logs.operations_at_first_message,
            (std::vector<std::string>{"end", "stop", "close"}));
  EXPECT_EQ(test::sdk.state_calls, 1);
}

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

TEST_F(FairinoHardwareInterfaceTest, GeneralToolDiZeroIsNotADragButtonByDefault) {
  test::sdk.state.tl_dgt_input_l = 0;
  writeCycle();
  EXPECT_TRUE(test::sdk.drag_requests.empty());
  EXPECT_EQ(test::sdk.servo_end_calls, 0);
  EXPECT_FALSE(dragActive());
}

TEST_F(FairinoHardwareInterfaceTest, FailedServoEndCannotEnterDrag) {
  test::sdk.end_result = 99;
  requestDrag(true);
  writeCycle();
  EXPECT_FALSE(dragActive());
  EXPECT_TRUE(test::sdk.drag_requests.empty());
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  EXPECT_TRUE(hardInhibited());
  EXPECT_EQ(test::sdk.stop_calls, 1);
  EXPECT_EQ(test::sdk.clear_calls, 1);
}

TEST_F(FairinoHardwareInterfaceTest, SafetyAfterFailedModeStillSendsPhysicalStop) {
  test::sdk.end_result = -1;
  requestDrag(true);
  writeCycle();
  const int stopped_before = test::sdk.stop_calls;
  test::sdk.state.safety_stop0_state = 1;
  writeCycle();
  EXPECT_EQ(test::sdk.stop_calls, stopped_before + 1);
  EXPECT_TRUE(test::sdk.servo_commands.empty());
}

TEST_F(FairinoHardwareInterfaceTest, PendingResumableFlushCannotRearmModeFault) {
  test::sdk.end_result = -1;
  requestDrag(true);
  writeCycle();
  test::sdk.end_result = 0;
  requestResumableFlush();
  for (int cycle = 0; cycle < 10; ++cycle) writeCycle();
  EXPECT_TRUE(hardInhibited());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  EXPECT_TRUE(test::sdk.servo_commands.empty());
}

TEST_F(FairinoHardwareInterfaceTest, FailedDragEnableCannotPretendModeChanged) {
  test::sdk.drag_result = -1;
  requestDrag(true);
  writeCycle();
  EXPECT_FALSE(dragActive());
  EXPECT_TRUE(hardInhibited());
  setCommand(displaced_);
  writeCycle();
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, FailedDragDisableCannotRestartServo) {
  requestDrag(true);
  writeCycle();
  ASSERT_TRUE(dragActive());
  test::sdk.drag_result = -1;
  requestDrag(false);
  writeCycle();
  EXPECT_TRUE(dragActive());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  EXPECT_TRUE(hardInhibited());
  EXPECT_TRUE(test::sdk.servo_commands.empty());
}

TEST_F(FairinoHardwareInterfaceTest, FailedServoRestartCannotReleaseDragHold) {
  requestDrag(true);
  writeCycle();
  test::sdk.start_result = 99;
  requestDrag(false);
  writeCycle();
  EXPECT_TRUE(hardInhibited());
  setCommand(displaced_);
  writeCycle();
  EXPECT_TRUE(test::sdk.servo_commands.empty());
}

TEST_F(FairinoHardwareInterfaceTest, SafetyStopPreemptsDragRequests) {
  test::sdk.state.safety_stop0_state = 1;
  requestDrag(true);
  writeCycle();
  EXPECT_EQ(test::sdk.stop_calls, 1);
  EXPECT_TRUE(test::sdk.drag_requests.empty());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  EXPECT_TRUE(test::sdk.servo_commands.empty());
  test::sdk.state.safety_stop0_state = 0;
  writeCycle();
  EXPECT_TRUE(test::sdk.drag_requests.empty());
}

TEST_F(FairinoHardwareInterfaceTest, GeneralToolDiDoesNotPreventConfirmedStop) {
  test::sdk.state.tl_dgt_input_l = 0;
  requestFlush();
  for (int cycle = 0; cycle < 10 && !flushSucceeded(); ++cycle) writeCycle();
  EXPECT_TRUE(flushSucceeded());
}

TEST_F(FairinoHardwareInterfaceTest, WiredFlangeButtonRequiresExplicitOptIn) {
  ASSERT_TRUE(initPeriod("", "true"));
  test::sdk.state.tl_dgt_input_l = 0;
  writeCycle();
  ASSERT_EQ(test::sdk.drag_requests, std::vector<uint8_t>{1});
  EXPECT_TRUE(dragActive());
  test::sdk.state.tl_dgt_input_l = 1;
  writeCycle();
  EXPECT_EQ(test::sdk.drag_requests, (std::vector<uint8_t>{1, 0}));
  EXPECT_FALSE(dragActive());
}

TEST_F(FairinoHardwareInterfaceTest, ExplicitlyWiredPressedButtonStillBlocksFlush) {
  ASSERT_TRUE(initPeriod("", "1"));
  test::sdk.state.tl_dgt_input_l = 0;
  requestFlush();
  for (int cycle = 0; cycle < 8; ++cycle) writeCycle();
  EXPECT_FALSE(flushSucceeded());
  EXPECT_TRUE(flushBlocked());
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
}

TEST_F(FairinoHardwareInterfaceTest, InvalidFlangeButtonSettingIsRejected) {
  for (const std::string value : {"yes", "automatic", "2"}) {
    EXPECT_FALSE(initPeriod("", value));
  }
  EXPECT_TRUE(initPeriod("", "false"));
  EXPECT_TRUE(initPeriod("", "0"));
}

TEST_F(FairinoHardwareInterfaceTest, SafetyStopDefersDragOffWithoutRestartingMotion) {
  requestDrag(true);
  writeCycle();
  ASSERT_TRUE(dragActive());
  test::sdk.state.safety_stop1_state = 1;
  requestDrag(false);
  writeCycle();
  EXPECT_EQ(test::sdk.stop_calls, 1);
  EXPECT_EQ(test::sdk.drag_requests, std::vector<uint8_t>{1});
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  test::sdk.state.safety_stop1_state = 0;
  writeCycle();
  EXPECT_EQ(test::sdk.drag_requests, (std::vector<uint8_t>{1, 0}));
  EXPECT_FALSE(dragActive());
}

TEST_F(FairinoHardwareInterfaceTest, UnchangedHoldDoesNotQueueRepeatedServoTargets) {
  for (int cycle = 0; cycle < 255; ++cycle) writeCycle();
  ASSERT_EQ(test::sdk.servo_commands.size(), 1u);
  expectTarget(Joints{});
}

TEST_F(FairinoHardwareInterfaceTest, IdleQueueCannotDelayHomeBeyondFiveSecondSettlingBudget) {
  // Replay the observed 255-entry backlog with a FIFO consumer at 50 Hz.
  // The actual write() emits the points; only the SDK/robot receiver is modeled.
  for (int cycle = 0; cycle < 255; ++cycle) writeCycle();
  size_t consumed = 0;
  Joints actual{};
  constexpr double goal = 0.1;
  for (int cycle = 1; cycle <= 25 + 250; ++cycle) {
    Joints command{};
    command[5] = goal * std::min(cycle, 25) / 25;
    setCommand(command);
    writeCycle();
    if (consumed < test::sdk.servo_commands.size()) {
      const auto& next = test::sdk.servo_commands[consumed++];
      for (size_t j = 0; j < actual.size(); ++j) {
        actual[j] = next.jPos[j] * std::acos(-1.0) / 180.0;
      }
    }
    setMeasured(actual);
  }
  EXPECT_LE(std::fabs(goal - actual[5]), std::acos(-1.0) / 180.0);
  EXPECT_EQ(consumed, test::sdk.servo_commands.size());
}

TEST_F(FairinoHardwareInterfaceTest, EveryChangedTargetIsSentWithoutADeadband) {
  for (int cycle = 0; cycle < 20; ++cycle) {
    Joints command{};
    command[5] = cycle * 1e-8;
    setCommand(command);
    writeCycle();
  }
  EXPECT_EQ(test::sdk.servo_commands.size(), 20u);
}

TEST_F(FairinoHardwareInterfaceTest, FailedSendCannotSuppressRetryOfTheSameTarget) {
  test::sdk.servo_result = 14;
  writeCycle();
  test::sdk.servo_result = 0;
  writeCycle();
  ASSERT_EQ(test::sdk.servo_commands.size(), 2u);
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), 2u);
}

TEST_F(FairinoHardwareInterfaceTest, SuccessfulFlushResendsItsFirstUnchangedHoldTarget) {
  writeCycle();
  requestFlush();
  for (int cycle = 0; cycle < 10 && !flushSucceeded(); ++cycle) writeCycle();
  ASSERT_TRUE(flushSucceeded());
  ASSERT_EQ(test::sdk.servo_commands.size(), 1u);
  writeCycle();
  ASSERT_EQ(test::sdk.servo_commands.size(), 2u);
  writeCycle();
  EXPECT_EQ(test::sdk.servo_commands.size(), 2u);
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

TEST_F(FairinoHardwareInterfaceTest, ResumableFlushReportsFailureWhenHardInhibitedAfterAdmission) {
  auto response = callFlushService();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (flushRequested() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  ASSERT_EQ(flushRequested(), 1u);
  latchHardInhibit();
  for (int i = 0; i < 6; ++i) writeCycle();
  const auto result = response.get();
  EXPECT_TRUE(flushSucceeded());
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("restart hardware stack"), std::string::npos);
  EXPECT_EQ(test::sdk.servo_start_calls, 0);
  EXPECT_TRUE(test::sdk.servo_commands.empty());
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
  // The already-sent safe hold target remains selected without requeueing it.
  EXPECT_EQ(test::sdk.servo_commands.size(), first_command_count);
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
