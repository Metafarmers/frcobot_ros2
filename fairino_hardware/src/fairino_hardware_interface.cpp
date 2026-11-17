#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <iomanip>
#include <locale>
#include <sstream>
#include <random>

#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware{

FairinoHardwareInterface::~FairinoHardwareInterface() {
    stopServiceThread();
    if (flight_recorder_) flight_recorder_->shutdown();
}

void FairinoHardwareInterface::startFlightRecorder() {
    // Lifecycle/background only. Failure to start telemetry must not change
    // robot enable, retry, hold, or hardware return policies.
    try {
        flight_recorder_->start([this](const IncidentInfo& info) {
            const auto stats = flight_recorder_->stats();
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                "event=flight_recorder_incident incident_id=%s pid=%d control_tid=%d "
                "trigger_mono_ns=%lld trigger_unix_ns=%lld post_until_mono_ns=%lld trace_path=%s "
                "io_errors=%llu queue_dropped=%llu incident_dropped=%llu",
                info.incident_id.c_str(), info.pid, info.control_tid,
                static_cast<long long>(info.trigger_mono_ns),
                static_cast<long long>(info.trigger_unix_ns),
                static_cast<long long>(info.post_until_mono_ns), info.trace_path.c_str(),
                static_cast<unsigned long long>(stats.io_errors),
                static_cast<unsigned long long>(stats.queue_dropped),
                static_cast<unsigned long long>(stats.incident_dropped));
        });
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
            "event=flight_recorder_start_failed reason=%s", error.what());
    }
}

void FairinoHardwareInterface::recordTrace(
    TraceKind kind, TraceRpc rpc, int rc, int64_t rpc_begin_mono_ns, uint64_t reasons) {
    TraceEvent event;
    event.stamp = clockNow();
    event.kind = kind;
    event.rpc = rpc;
    event.control_tid = controlThreadId();
    event.rc = rc;
    event.cycle_index = trace_cycle_index_;
    event.write_index = control_diagnostics_.write_cycles;
    event.rpc_begin_mono_ns = rpc_begin_mono_ns;
    // This deployment's ros2_control cadence is fixed at50Hz. ServoJ cmdT is
    // interpolation duration; the passed ROS period is measured iteration time.
    // Neither is the expected cadence used to recognize missed-deadline catch-up.
    event.nominal_period_sec = 0.02;
    event.command_period_sec = servo_command_period_sec_;
    event.reported_period_sec = trace_reported_period_sec_;
    event.actual_period_sec = control_diagnostics_.actual_loop_period_sec;
    event.frame_age_sec = rt_frame_seen_
        ? (event.stamp.mono_ns - std::chrono::duration_cast<std::chrono::nanoseconds>(
               rt_frame_advanced_at_.time_since_epoch()).count()) * 1e-9 : -1.0;
    event.trigger_reasons = reasons;
    event.sdk_observed = control_diagnostics_.sdk.observed;
    event.sdk_observed_mono_ns = control_diagnostics_.sdk.observed_at_ns;
    event.state_valid = rt_valid_;
    if (event.sdk_observed) event.raw_sdk = trace_raw_sdk_;
    event.command_valid = true;
    std::copy_n(_jnt_position_command, 6, event.command_rad);
    event.attempted_target_valid = control_diagnostics_.servo_target_observed;
    std::copy_n(control_diagnostics_.last_servo_target_rad.begin(), 6, event.attempted_target_rad);
    event.successful_target_valid = trace_successful_target_observed_;
    std::copy_n(last_servo_target_.begin(), 6, event.successful_target_rad);
    event.flags = static_cast<uint64_t>(cancel_hold_) |
        (static_cast<uint64_t>(flush_inhibit_) << 1) |
        (static_cast<uint64_t>(hard_inhibit_.load()) << 2) |
        (static_cast<uint64_t>(_post_drag_hold) << 3) |
        (static_cast<uint64_t>(_post_estop_hold) << 4) |
        (static_cast<uint64_t>(_drag_active) << 5) |
        (static_cast<uint64_t>(drag_transition_failed_) << 6);
    flight_recorder_->record(event);
}

void FairinoHardwareInterface::recordRpcBegin(
    RpcIndex index, std::chrono::steady_clock::time_point start) {
    static constexpr TraceRpc kinds[] = {TraceRpc::STATE, TraceRpc::SERVO,
        TraceRpc::STOP, TraceRpc::END, TraceRpc::CLEAR, TraceRpc::QUEUE,
        TraceRpc::START, TraceRpc::DRAG, TraceRpc::RESET};
    recordTrace(TraceKind::RPC_BEGIN, kinds[index], 0,
        std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count());
}

void FairinoHardwareInterface::startServiceThread() {
    rclcpp::ExecutorOptions options;
    options.context = _svc_node->get_node_base_interface()->get_context();
    _svc_exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(options, 2);
    _svc_exec->add_node(_svc_node);
    _svc_spin_thread = std::thread([this]() { _svc_exec->spin(); });
}

void FairinoHardwareInterface::drainFaultLogsOnDeactivation() {
    if (fault_write_index_.load() == fault_read_index_.load() &&
        fault_snapshots_dropped_.load() == 0) return;
    // Called after service callbacks have joined AND SDK stop/close calls have
    // run. A slow log sink must not postpone issuing those existing stop calls.
    std::thread writer([this]() {
        DiagnosticSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(diagnostics_mutex_);
            snapshot = published_diagnostics_;
        }
        logPendingFaults(snapshot, 0.0);
    });
    writer.join();
}

void FairinoHardwareInterface::stopServiceThread() {
    if (_svc_exec) _svc_exec->cancel();
    if (_svc_spin_thread.joinable()) _svc_spin_thread.join();
    if (_svc_exec && _svc_node) _svc_exec->remove_node(_svc_node);
    _svc_exec.reset();
    stream_timer_.reset();
    stream_srv_.reset();
    stream_pub_.reset();
    diagnostics_timer_.reset();
    trajectory_status_sub_.reset();
    diagnostics_pub_.reset();
    flush_srv_.reset();
    stop_hold_srv_.reset();
    flush_callback_group_.reset();
    _drag_srv.reset();
    _svc_node.reset();
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    last_servo_target_valid_ = false;
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量

    stream_guard_enabled_ = true;
    stream_delay_threshold_sec_ = 0.06;
    try {
        const auto enabled = info_.hardware_parameters.find("servo_delay_guard_enabled");
        if (enabled != info_.hardware_parameters.end()) {
            if (enabled->second != "true" && enabled->second != "false") throw std::invalid_argument("enabled");
            stream_guard_enabled_ = enabled->second == "true";
        }
        const auto threshold = info_.hardware_parameters.find("servo_delay_threshold_sec");
        if (threshold != info_.hardware_parameters.end()) {
            size_t used = 0;
            stream_delay_threshold_sec_ = std::stod(threshold->second, &used);
            if (used != threshold->second.size()) throw std::invalid_argument("threshold");
        }
        if (!std::isfinite(stream_delay_threshold_sec_) || stream_delay_threshold_sec_ < 0.02 ||
            stream_delay_threshold_sec_ > 1.0) throw std::invalid_argument("threshold");
    } catch (const std::exception&) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Invalid ServoJ delay guard parameters");
        return hardware_interface::CallbackReturn::ERROR;
    }
    std::random_device random;
    stream_session_ = std::to_string(random()) + "-" + std::to_string(random()) + "-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

    flange_drag_button_enabled_ = false;
    const auto button_parameter = info_.hardware_parameters.find("flange_drag_button_enabled");
    if (button_parameter != info_.hardware_parameters.end()) {
        const auto& value = button_parameter->second;
        if (value == "true" || value == "1") flange_drag_button_enabled_ = true;
        else if (value != "false" && value != "0") {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "flange_drag_button_enabled must be true/false or 1/0");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    servo_command_period_sec_ = 0.02;
    const auto period_parameter = info_.hardware_parameters.find("servo_command_period_sec");
    if (period_parameter != info_.hardware_parameters.end()) {
        try {
            size_t parsed = 0;
            const double value = std::stod(period_parameter->second, &parsed);
            // Deployment keeps a 50 Hz controller. Reject shorter interpolation
            // periods (including the old 8 ms value), junk, NaN and infinity.
            if (parsed != period_parameter->second.size() || !std::isfinite(value) ||
                value < 0.02 || value > 0.1) throw std::invalid_argument("period");
            servo_command_period_sec_ = value;
        } catch (const std::exception&) {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "servo_command_period_sec must be finite and in [0.02, 0.1] seconds");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    if (info_.joints.size() != 6) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Exactly six joints are required");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // read robot_ip from URDF <ros2_control> hardware parameters
    auto it = info_.hardware_parameters.find("robot_ip");
    if (it != info_.hardware_parameters.end()) {
        _controller_ip = it->second;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Robot IP from parameter: %s", _controller_ip.c_str());
    } else {
        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                    "No 'robot_ip' parameter found, using default: %s", _controller_ip.c_str());
    }

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {

        //指令部分
        if (joint.command_interfaces.size() != 1) {//开放servoJ
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        //关节状态部分
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //                 "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //                 "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

    }
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state.at(i)));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State&)
{
    using namespace std::chrono_literals;
    stream_session_ += "-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    stream_control_ = StreamSnapshot{};
    stream_published_ = StreamSnapshot{};
    stream_pending_ = StreamRequest{};
    stream_last_request_ = StreamRequest{};
    stream_mailbox_pending_ = false;
    stream_latched_.store(false);
    stream_abort_binding_.store(0);
    stream_rpc_begin_ns_.store(0);
    stream_lease_binding_.store(0);
    stream_guard_flush_ = false;
    control_diagnostics_ = DiagnosticSnapshot{};
    published_diagnostics_ = DiagnosticSnapshot{};
    fault_write_index_.store(0);
    fault_read_index_.store(0);
    fault_snapshots_dropped_.store(0);
    previous_fault_active_ = false;
    snapshot_stale_logged_ = false;
    reported_aborted_goals_.clear();
    // Keep the recorder's last incident handoff across fast deactivate/activate;
    // the host may not poll the short-lived capturing/stopped phases in time.
    trace_raw_sdk_ = ROBOT_STATE_PKG{};
    trace_cycle_index_ = 0;
    trace_successful_target_observed_ = false;
    trace_reported_period_sec_ = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "[servo-diagnostics] cmdT=%.6f s cancel_hold_release_tolerance=0.005000 rad",
                servo_command_period_sec_);
    //做变量的初始化工作
    _ptr_robot = std::make_unique<FRRobot>();//创建机器人实例
    for(int i=0;i<6;i++){//初始化变量
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
    }
    _control_mode = 0;//默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Connecting to robot at %s ...", _controller_ip.c_str());
    errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str());//建立xmlrpc连接
    rclcpp::sleep_for(200ms);//等待一段时间让控制器的rpc连接建立完毕
    if(returncode != 0){
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                     "SDK connection failed (error=%d) to %s", returncode, _controller_ip.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "SDK connected to %s", _controller_ip.c_str());
    }
    // Clear any existing errors and enable the robot
    _ptr_robot->ResetAllError();
    rclcpp::sleep_for(100ms);
    returncode = _ptr_robot->RobotEnable(1);
    if(returncode != 0){
        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "RobotEnable failed (error=%d), may already be enabled", returncode);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Robot enabled");
    }
    rclcpp::sleep_for(100ms);
    //做第一步的工作，读取当前状态数据
    JointPos jntpos;
    returncode = _ptr_robot->GetActualJointPosDegree(0,&jntpos);
    /*
    获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
    因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
    */
    if(returncode == 0){
        for(int j=0;j<6;j++){
            _jnt_position_command[j] = jntpos.jPos[j]/180.0*M_PI;
            _jnt_position_state[j] = _jnt_position_command[j];
        }
        rt_valid_ = false;
        rt_available_for_write_ = false;
        rt_frame_seen_ = false;
        last_write_at_ = {};
        last_servo_target_valid_ = false;
        flush_phase_ = FlushPhase::IDLE;
        flush_active_generation_ = 0;
        flush_requested_.store(0);
        flush_completed_.store(0);
        flush_result_.store(static_cast<int>(FlushResult::NONE));
        flush_resume_requested_.store(true);
        hard_inhibit_.store(false);
        cancel_hold_ = false;
        flush_inhibit_ = false;
        drag_transition_failed_ = false;
        _drag_active = false;
        _drag_req.store(0);
        _drag_btn_prev = false;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"初始指令位置: %f,%f,%f,%f,%f,%f",_jnt_position_command[0],\
        _jnt_position_command[1],_jnt_position_command[2],_jnt_position_command[3],_jnt_position_command[4],_jnt_position_command[5]);
        // Enter servo mode before ServoJ commands
        returncode = _ptr_robot->ServoMoveStart();
        if(returncode != 0){
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "ServoMoveStart failed (error=%d)", returncode);
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoMoveStart OK. Hardware activated.");
        // ★손교시 SW 언락 서비스 — ~/set_drag_teach(SetBool): true=드래그ON, false=OFF.
        //   전원재시작 없이 서비스 한 번으로 손교시 언락(DragTeachSwitch → robot_state=4 → ServoJ 스킵).
        _svc_node = std::make_shared<rclcpp::Node>("fairino_hw_drag");
        _drag_srv = _svc_node->create_service<std_srvs::srv::SetBool>(
            "~/set_drag_teach",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> resp){
                _drag_req.store(req->data ? 1 : -1);   // 원자 세팅만(백그라운드 스레드) → write()가 소비
                resp->success = true;
                resp->message = req->data ? "drag ON requested" : "drag OFF requested";
            });
        flush_callback_group_ = _svc_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        flush_srv_ = _svc_node->create_service<std_srvs::srv::Trigger>(
            "/fairino_hw_control/stop_and_flush",
            [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                requestStopAndFlush(*response, true);
            }, rmw_qos_profile_services_default, flush_callback_group_);
        stop_hold_srv_ = _svc_node->create_service<std_srvs::srv::Trigger>(
            "/fairino_hw_control/stop_and_hold",
            [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                requestStopAndFlush(*response, false);
            }, rmw_qos_profile_services_default, flush_callback_group_);
        createStreamInterfaces();
        createDiagnosticInterfaces();
        startFlightRecorder();
        // ★서비스 spin 을 별도 스레드로 — 실시간 write() 루프서 spin_some 을 부르던 걸 대체(지터 제거).
        startServiceThread();
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "손교시 언락 서비스 준비: /fairino_hw_drag/set_drag_teach (std_srvs/SetBool, 백그라운드 spin)");
        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
    last_servo_target_valid_ = false;
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    // ★백그라운드 서비스 spin 정지: executor cancel → 스레드 join → 리소스 해제(순서 중요).
    stopServiceThread();
    _ptr_robot->ServoMoveEnd();//退出伺服模式
    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    _ptr_robot.reset();
    // Joining/flushing recorder disk I/O must follow the existing physical
    // stop and SDK close calls, never delay their issuance.
    flight_recorder_->shutdown();
    drainFaultLogsOnDeactivation();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



void FairinoHardwareInterface::recordRpc(
    RpcIndex index, int rc, std::chrono::steady_clock::time_point start) {
    auto& sample = control_diagnostics_.rpc[index];
    sample.rc = rc;
    sample.duration_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    ++sample.calls;
    if (rc != 0) {
        sample.last_error_rc = rc;
        sample.last_error_duration_ms = sample.duration_ms;
        sample.last_error_at_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        ++sample.errors;
    }
    static constexpr TraceRpc kinds[] = {TraceRpc::STATE, TraceRpc::SERVO,
        TraceRpc::STOP, TraceRpc::END, TraceRpc::CLEAR, TraceRpc::QUEUE,
        TraceRpc::START, TraceRpc::DRAG, TraceRpc::RESET};
    recordTrace(TraceKind::RPC_END, kinds[index], rc,
        std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count());
}

void FairinoHardwareInterface::commitDiagnostics() {
    control_diagnostics_.sampled_at_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    control_diagnostics_.command_period_sec = servo_command_period_sec_;
    control_diagnostics_.state_valid = rt_valid_;
    control_diagnostics_.cancel_hold = cancel_hold_;
    control_diagnostics_.flush_inhibit = flush_inhibit_;
    control_diagnostics_.hard_inhibit = hard_inhibit_.load();
    control_diagnostics_.drag_hold = _post_drag_hold;
    control_diagnostics_.estop_hold = _post_estop_hold;
    control_diagnostics_.drag_active = _drag_active;
    control_diagnostics_.drag_transition_failed = drag_transition_failed_;
    control_diagnostics_.flange_drag_button_enabled = flange_drag_button_enabled_;
    control_diagnostics_.raw_tool_di = rt_snapshot_.tl_dgt_input_l;
    control_diagnostics_.main_error = rt_snapshot_.main_code;
    control_diagnostics_.sub_error = rt_snapshot_.sub_code;
    control_diagnostics_.flush_generation = flush_requested_.load();
    control_diagnostics_.flush_completed_generation = flush_completed_.load();
    control_diagnostics_.flush_result = flush_result_.load();
    control_diagnostics_.flush_phase = static_cast<int>(flush_phase_);
    std::copy_n(_jnt_position_command, 6, control_diagnostics_.command_rad.begin());
    std::copy_n(_jnt_position_state, 6, control_diagnostics_.measured_rad.begin());
    control_diagnostics_.frame_age_sec = rt_frame_seen_
        ? (control_diagnostics_.sampled_at_ns -
           std::chrono::duration_cast<std::chrono::nanoseconds>(
               rt_frame_advanced_at_.time_since_epoch()).count()) * 1e-9
        : -1.0;
    retainFaultSnapshot();
    // The timer copies this POD under the mutex, then releases it before doing
    // any allocation, formatting or ROS publication. Never block the RT loop.
    std::unique_lock<std::mutex> lock(diagnostics_mutex_, std::try_to_lock);
    if (lock.owns_lock()) published_diagnostics_ = control_diagnostics_;
}

void FairinoHardwareInterface::retainFaultSnapshot() {
    const auto& snapshot = control_diagnostics_;
    const auto& sdk = snapshot.sdk;
    FaultKey key{};
    if (sdk.observed) {
        key[0] = sdk.main_code;
        key[1] = sdk.sub_code;
        key[2] = sdk.emergency_stop;
        key[3] = sdk.collision;
        key[4] = sdk.safety0;
        key[5] = sdk.safety1;
    }
    key[6] = snapshot.rpc[STATE_RPC].calls != 0 && !snapshot.state_valid;
    key[7] = snapshot.drag_transition_failed;
    key[8] = snapshot.flush_result >= static_cast<int>(FlushResult::RPC_ERROR)
        ? snapshot.flush_result : 0;
    for (size_t i = 0; i < RPC_COUNT; ++i) {
        key[9 + i] = snapshot.rpc[i].calls != 0 ? snapshot.rpc[i].rc : 0;
    }
    const bool active = std::any_of(key.begin(), key.end(), [](int value) { return value != 0; });
    if (!active) {
        previous_fault_active_ = false;
        return;
    }
    if (previous_fault_active_ && key == previous_fault_key_) return;
    previous_fault_active_ = true;
    previous_fault_key_ = key;
    uint64_t reasons = TRIGGER_NONE;
    if (key[0] || key[1]) reasons |= CONTROLLER_FAULT;
    if (key[2] || key[3] || key[4] || key[5]) reasons |= SAFETY_FAULT;
    if (key[6]) {
        if (!sdk.observed || snapshot.rpc[STATE_RPC].rc != 0) reasons |= MISSING_STATE;
        else if (std::any_of(sdk.joint_position_deg.begin(), sdk.joint_position_deg.end(),
                            [](double value) { return !std::isfinite(value); })) {
            reasons |= NONFINITE_STATE;
        } else reasons |= STALE_STATE;
    }
    if (key[7] || key[8]) reasons |= CONTROLLER_FAULT;
    recordTrace(TraceKind::STATE, TraceRpc::NONE, 0, 0, reasons);
    const uint64_t write = fault_write_index_.load(std::memory_order_relaxed);
    const uint64_t read = fault_read_index_.load(std::memory_order_acquire);
    if (write - read >= kFaultSnapshotCapacity) {
        fault_snapshots_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    fault_snapshots_[write % kFaultSnapshotCapacity] = snapshot;
    fault_write_index_.store(write + 1, std::memory_order_release);
}

void FairinoHardwareInterface::createDiagnosticInterfaces() {
    diagnostics_pub_ = _svc_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));
    trajectory_status_sub_ = _svc_node->create_subscription<action_msgs::msg::GoalStatusArray>(
        "joint_trajectory_controller/follow_joint_trajectory/_action/status",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr statuses) {
            logTrajectoryAborts(*statuses);
        });
    diagnostics_timer_ = _svc_node->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { publishDiagnostics(); });
}

void FairinoHardwareInterface::logFaultSnapshot(
    const char* event, const DiagnosticSnapshot& snapshot, const std::string& goal_id,
    int64_t goal_accepted_ros_ns) {
    // Called only by background callbacks, never from read()/write(). SDK state
    // is the last successful getter result; it may precede a failing ServoJ.
    const auto& sdk = snapshot.sdk;
    const int64_t logged_at_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::ostringstream text;
    text.imbue(std::locale::classic());
    text << std::fixed << std::setprecision(6) << "event=" << event;
    if (!goal_id.empty()) {
        text << " goal_uuid=" << goal_id << " goal_accepted_ros_ns=" << goal_accepted_ros_ns;
    }
    const char* robot_id = std::getenv("MF_ROBOT_ID");
    text << " robot_id=" << (robot_id && *robot_id ? robot_id : _controller_ip.c_str())
         << " snapshot_mono_ns=" << snapshot.sampled_at_ns
         << " logged_mono_ns=" << logged_at_ns
         << " snapshot_age_sec=" << (snapshot.sampled_at_ns != 0
              ? (logged_at_ns - snapshot.sampled_at_ns) * 1e-9 : -1.0)
         << " sdk_state_observed=" << sdk.observed
         << " sdk_observed_mono_ns=" << sdk.observed_at_ns
         << " sdk_sample_age_sec=" << (sdk.observed
              ? (logged_at_ns - sdk.observed_at_ns) * 1e-9 : -1.0)
         << " state_valid=" << snapshot.state_valid
         << " frame=" << sdk.frame << " frame_age_sec=" << snapshot.frame_age_sec
         << " main_code=" << sdk.main_code << " sub_code=" << sdk.sub_code
         << " robot_state=" << sdk.robot_state << " robot_mode=" << sdk.robot_mode
         << " program_state=" << sdk.program_state
         << " emergency_stop=" << sdk.emergency_stop << " collision=" << sdk.collision
         << " safety_stop0=" << sdk.safety0 << " safety_stop1=" << sdk.safety1
         << " motion_done=" << sdk.motion_done << " mc_queue_len=" << sdk.motion_queue_length
         << " servoJCmdNum=" << sdk.servo_command_count
         << " actual_loop_period_sec=" << snapshot.actual_loop_period_sec
         << " cmdT_sec=" << snapshot.command_period_sec
         << " write_cycles=" << snapshot.write_cycles
         << " servo_target_observed=" << snapshot.servo_target_observed;
    const auto joints = [&text](const char* label, const std::array<double, 6>& values) {
        text << ' ' << label << "=[";
        for (size_t j = 0; j < values.size(); ++j) {
            if (j != 0) text << ',';
            text << values[j];
        }
        text << ']';
    };
    joints("command_rad", snapshot.command_rad);
    joints("last_servo_target_rad", snapshot.last_servo_target_rad);
    joints("measured_rad", snapshot.measured_rad);
    joints("sdk_joint_position_deg", sdk.joint_position_deg);
    joints("sdk_joint_velocity_deg_sec", sdk.joint_velocity_deg_sec);
    text << " cancel_hold=" << snapshot.cancel_hold << " hard_inhibit=" << snapshot.hard_inhibit
         << " flush_inhibit=" << snapshot.flush_inhibit << " flush_phase=" << snapshot.flush_phase
         << " flush_result=" << snapshot.flush_result << " drag_active=" << snapshot.drag_active
         << " flush_generation=" << snapshot.flush_generation
         << " flush_completed_generation=" << snapshot.flush_completed_generation
         << " drag_transition_failed=" << snapshot.drag_transition_failed
         << " drag_hold=" << snapshot.drag_hold << " estop_hold=" << snapshot.estop_hold
         << " flange_drag_button_enabled=" << snapshot.flange_drag_button_enabled
         << " raw_tool_di=" << snapshot.raw_tool_di;
    static constexpr const char* names[] = {
        "state_rpc", "ServoJ", "StopMotion", "ServoMoveEnd", "MotionQueueClear",
        "GetMotionQueueLength", "ServoMoveStart", "DragTeachSwitch", "ResetAllError"};
    for (size_t i = 0; i < RPC_COUNT; ++i) {
        text << ' ' << names[i] << "_rc=" << snapshot.rpc[i].rc
             << ' ' << names[i] << "_duration_ms=" << snapshot.rpc[i].duration_ms
             << ' ' << names[i] << "_calls=" << snapshot.rpc[i].calls;
        if (snapshot.rpc[i].errors != 0) {
            text << ' ' << names[i] << "_last_error_rc=" << snapshot.rpc[i].last_error_rc
                 << ' ' << names[i] << "_last_error_duration_ms="
                 << snapshot.rpc[i].last_error_duration_ms
                 << ' ' << names[i] << "_last_error_mono_ns=" << snapshot.rpc[i].last_error_at_ns
                 << ' ' << names[i] << "_errors=" << snapshot.rpc[i].errors;
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "%s", text.str().c_str());
}

void FairinoHardwareInterface::logPendingFaults(
    const DiagnosticSnapshot& latest, double snapshot_age_sec) {
    const uint64_t dropped = fault_snapshots_dropped_.exchange(0, std::memory_order_relaxed);
    if (dropped != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                     "event=sdk_fault_log_overflow dropped_snapshots=%llu",
                     static_cast<unsigned long long>(dropped));
    }
    // Bound each drain even if faults are changing while the callback runs.
    const uint64_t end = fault_write_index_.load(std::memory_order_acquire);
    uint64_t read = fault_read_index_.load(std::memory_order_relaxed);
    while (read != end) {
        const auto snapshot = fault_snapshots_[read % kFaultSnapshotCapacity];
        fault_read_index_.store(++read, std::memory_order_release);
        logFaultSnapshot("sdk_fault", snapshot);
    }
    const bool stale = latest.sampled_at_ns != 0 && snapshot_age_sec > 0.2;
    if (stale && !snapshot_stale_logged_) {
        logFaultSnapshot("control_snapshot_stale", latest);
    }
    snapshot_stale_logged_ = stale;
}

void FairinoHardwareInterface::logTrajectoryAborts(
    const action_msgs::msg::GoalStatusArray& statuses) {
    std::vector<std::array<uint8_t, 16>> current;
    for (const auto& status : statuses.status_list) {
        if (status.status != action_msgs::msg::GoalStatus::STATUS_ABORTED) continue;
        const auto& id = status.goal_info.goal_id.uuid;
        current.push_back(id);
        if (std::find(reported_aborted_goals_.begin(), reported_aborted_goals_.end(), id) !=
            reported_aborted_goals_.end()) continue;
        DiagnosticSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(diagnostics_mutex_);
            snapshot = published_diagnostics_;
        }
        std::ostringstream goal_id;
        goal_id << std::hex << std::setfill('0');
        for (uint8_t byte : id) goal_id << std::setw(2) << static_cast<unsigned>(byte);
        const auto& stamp = status.goal_info.stamp;
        flight_recorder_->requestIncident(ACTION_ABORT, clockNow(), goal_id.str().c_str(),
            static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec);
        logFaultSnapshot("trajectory_aborted", snapshot, goal_id.str(),
                         static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec);
    }
    reported_aborted_goals_ = std::move(current);
}

void FairinoHardwareInterface::publishDiagnostics() {
    DiagnosticSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        snapshot = published_diagnostics_;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = _svc_node->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fairino_hardware/control";
    const char* robot_id = std::getenv("MF_ROBOT_ID");
    status.hardware_id = robot_id && *robot_id ? robot_id : _controller_ip;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "servo control";
    const double sample_age_sec = (std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() - snapshot.sampled_at_ns) * 1e-9;
    logPendingFaults(snapshot, sample_age_sec);
    if (snapshot.drag_transition_failed) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "drag/servo mode switch failed; hardware restart required";
    } else if (!snapshot.state_valid || sample_age_sec > 0.2 || snapshot.flush_inhibit ||
        snapshot.rpc[SERVO_RPC].rc != 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "state/servo/flush blocked";
    } else if (snapshot.cancel_hold || snapshot.drag_hold || snapshot.estop_hold || snapshot.drag_active ||
               snapshot.safety0 || snapshot.safety1 || snapshot.robot_state == 4) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "holding or safety/drag stop";
    }
    const auto add = [&status](const char* key, const auto value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = std::to_string(value);
        status.values.push_back(std::move(item));
    };
    add("servo_command_period_sec", snapshot.command_period_sec);
    add("cmdT_sec", snapshot.command_period_sec);
    add("loop_period_sec", snapshot.loop_period_sec);
    add("actual_loop_period_sec", snapshot.actual_loop_period_sec);
    add("state_age_sec", snapshot.state_age_sec);
    add("diagnostic_snapshot_age_sec", sample_age_sec);
    add("write_cycles", snapshot.write_cycles);
    add("servoJCmdNum", snapshot.servo_command_count);
    add("target_state_error_rad", snapshot.target_state_error_rad);
    add("queue_length", snapshot.queue_length);
    add("rt_motion_queue_length", snapshot.rt_motion_queue_length);
    add("duplicate_servo_targets_skipped", snapshot.duplicate_servo_targets_skipped);
    add("flush_generation", snapshot.flush_generation);
    add("flush_completed_generation", snapshot.flush_completed_generation);
    add("flush_result", snapshot.flush_result);
    add("flush_phase", snapshot.flush_phase);
    add("cancel_hold", snapshot.cancel_hold);
    add("flush_inhibit", snapshot.flush_inhibit);
    add("hard_inhibit", snapshot.hard_inhibit);
    add("post_drag_hold", snapshot.drag_hold);
    add("post_estop_hold", snapshot.estop_hold);
    add("drag_active", snapshot.drag_active);
    add("drag_transition_failed", snapshot.drag_transition_failed);
    add("flange_drag_button_enabled", snapshot.flange_drag_button_enabled);
    add("raw_tool_di", snapshot.raw_tool_di);
    add("controller_main_error", snapshot.main_error);
    add("controller_sub_error", snapshot.sub_error);
    add("robot_state", snapshot.robot_state);
    add("safety_stop0_state", snapshot.safety0);
    add("safety_stop1_state", snapshot.safety1);
    add("rt_frame", snapshot.frame);
    static constexpr const char* names[] = {
        "state_rpc", "ServoJ", "StopMotion", "ServoMoveEnd", "MotionQueueClear",
        "GetMotionQueueLength", "ServoMoveStart", "DragTeachSwitch", "ResetAllError"};
    for (size_t i = 0; i < RPC_COUNT; ++i) {
        add((std::string(names[i]) + "_rc").c_str(), snapshot.rpc[i].rc);
        add((std::string(names[i]) + "_duration_ms").c_str(), snapshot.rpc[i].duration_ms);
        add((std::string(names[i]) + "_calls").c_str(), snapshot.rpc[i].calls);
    }
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
}

void FairinoHardwareInterface::requestStopAndFlush(
    std_srvs::srv::Trigger::Response& response, bool resume_after_stop) {
    response.success = false;
    if (flush_callback_busy_.exchange(true)) {
        response.message = "stop_and_flush already waiting";
        return;
    }
    if (!resume_after_stop && hard_inhibit_.load() &&
        flush_requested_.load() == flush_completed_.load() &&
        flush_result_.load() == static_cast<int>(FlushResult::SUCCESS)) {
        response.success = true;
        response.message = "motion already stopped and hard-inhibited until hardware restart";
        flush_callback_busy_.store(false);
        return;
    }
    if (resume_after_stop && stream_latched_.load()) {
        response.message = "ServoJ delay guard requires a matching explicit rearm";
        flush_callback_busy_.store(false);
        return;
    }
    if (resume_after_stop && hard_inhibit_.load()) {
        response.message = "motion is hard-inhibited after an unconfirmed goal; restart hardware stack";
        flush_callback_busy_.store(false);
        return;
    }
    if (flush_requested_.load() != flush_completed_.load()) {
        response.message = "previous stop_and_flush is still pending; motion remains blocked";
        flush_callback_busy_.store(false);
        return;
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(kFlushTimeoutSec));
    flush_deadline_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline.time_since_epoch()).count());
    flush_result_.store(static_cast<int>(FlushResult::PENDING));
    flush_resume_requested_.store(resume_after_stop);
    const uint64_t generation = flush_requested_.fetch_add(1) + 1;
    // SDK calls are exclusively on the control thread. This background wait
    // is bounded even if an SDK call or the controller manager itself stalls.
    while (flush_completed_.load() < generation &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const bool completed = flush_completed_.load() >= generation;
    const int result = flush_result_.load();
    const bool restart_required = resume_after_stop && hard_inhibit_.load();
    response.success = completed && result == static_cast<int>(FlushResult::SUCCESS) &&
        !restart_required;
    response.message = std::string(resume_after_stop ? "stop_and_flush" : "stop_and_hold") +
        " generation=" + std::to_string(generation) +
        (completed ? " result=" + std::to_string(result)
                   : " timeout; completion unconfirmed, motion remains blocked");
    if (restart_required) response.message += "; motion hard-inhibited, restart hardware stack";
    flush_callback_busy_.store(false);
}

void FairinoHardwareInterface::finishFlush(FlushResult result) {
    flush_phase_ = FlushPhase::IDLE;
    if (result != FlushResult::SUCCESS) flush_inhibit_ = true;
    flush_result_.store(static_cast<int>(result));
    flush_completed_.store(flush_active_generation_);
    if (stream_guard_flush_) {
        stream_control_.phase = result == FlushResult::SUCCESS && streamHealthy() &&
            stream_control_.reason == StreamState::DELAY && stream_control_.binding != 0 &&
            stream_control_.recoveries == 0 ? StreamState::QUIESCENT : StreamState::BLOCKED;
        if (result != FlushResult::SUCCESS && stream_control_.reason == StreamState::DELAY) {
            stream_control_.reason = result == FlushResult::TIMEOUT ? StreamState::TIMEOUT : StreamState::UNSAFE;
        }
    }
}

bool FairinoHardwareInterface::processFlush() {
    const auto now_ns = []() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };
    const auto expired = [&]() { return now_ns() >= flush_deadline_ns_.load(); };
    const uint64_t requested = flush_requested_.load();
    if (requested > flush_active_generation_) {
        last_servo_target_valid_ = false;
        flush_active_generation_ = requested;
        flush_resume_active_ = flush_resume_requested_.load() && !hard_inhibit_.load() &&
            !stream_latched_.load();
        if (!flush_resume_active_ && !stream_guard_flush_) hard_inhibit_.store(true);
        cancel_hold_ = true;
        flush_inhibit_ = true;
        flush_stationary_frames_ = 0;
        for (size_t j = 0; j < 6; ++j) {
            _hold_target[j] = _jnt_position_state[j];
            cancel_command_at_flush_[j] = _jnt_position_command[j];
        }
        // Best-effort stop/end/clear even if one operation fails. A request
        // already timed out still stops stale motion, but must never restart it.
        auto start = std::chrono::steady_clock::now();
        recordRpcBegin(STOP_RPC, start);
        const int stop_rc = _ptr_robot->StopMotion();
        recordRpc(STOP_RPC, stop_rc, start);
        const auto end_servo = [&]() {
            const auto call_start = std::chrono::steady_clock::now();
            recordRpcBegin(END_RPC, call_start);
            const int code = _ptr_robot->ServoMoveEnd();
            recordRpc(END_RPC, code, call_start);
            return code;
        };
        const auto clear_queue = [&]() {
            const auto call_start = std::chrono::steady_clock::now();
            recordRpcBegin(CLEAR_RPC, call_start);
            const int code = _ptr_robot->MotionQueueClear();
            recordRpc(CLEAR_RPC, code, call_start);
            return code;
        };
        int end_rc, clear_rc;
        if (stream_guard_flush_) { clear_rc = clear_queue(); end_rc = end_servo(); }
        else { end_rc = end_servo(); clear_rc = clear_queue(); }
        control_diagnostics_.queue_length = -1;
        if (stop_rc || end_rc || clear_rc) finishFlush(FlushResult::RPC_ERROR);
        else if (expired()) finishFlush(FlushResult::TIMEOUT);
        else flush_phase_ = FlushPhase::WAIT_QUEUE;
        return true;
    }
    if (flush_phase_ == FlushPhase::IDLE) return flush_inhibit_;
    if (expired()) {
        finishFlush(FlushResult::TIMEOUT);
        return true;
    }
    if (flush_phase_ == FlushPhase::WAIT_QUEUE) {
        int length = -1;
        const auto start = std::chrono::steady_clock::now();
        recordRpcBegin(QUEUE_RPC, start);
        const int rc = _ptr_robot->GetMotionQueueLength(&length);
        recordRpc(QUEUE_RPC, rc, start);
        control_diagnostics_.queue_length = length;
        if (rc || length < 0) finishFlush(FlushResult::RPC_ERROR);
        else if (length == 0) {
            // A zero SDK queue does not prove that the arm has stopped.
            // Start a fresh-frame stationary window after the queue sample.
            flush_queue_empty_frame_ = rt_snapshot_.frame_cnt;
            flush_stationary_frame_ = flush_queue_empty_frame_;
            flush_stationary_frames_ = 0;
            flush_phase_ = FlushPhase::WAIT_STATIONARY;
        }
        return true;
    }
    if (!rt_valid_) return true;
    if (_drag_active || _drag_req.load() != 0 || rt_snapshot_.robot_state == 4 ||
        (flange_drag_button_enabled_ && !(rt_snapshot_.tl_dgt_input_l & 1)) || rt_snapshot_.safety_stop0_state ||
        rt_snapshot_.safety_stop1_state) {
        finishFlush(FlushResult::UNSAFE_STATE);
        return true;
    }
    if (flush_phase_ == FlushPhase::WAIT_STATIONARY) {
        if (rt_snapshot_.frame_cnt == flush_stationary_frame_) return true;
        flush_stationary_frame_ = rt_snapshot_.frame_cnt;
        if (stream_guard_flush_) {
            if (!streamHealthy()) { finishFlush(FlushResult::UNSAFE_STATE); return true; }
            bool velocities_missing = true, velocities_valid = true, stationary = true;
            for (double velocity : rt_snapshot_.actual_qd) velocities_missing &= std::isnan(velocity);
            if (!velocities_missing) {
                for (double velocity : rt_snapshot_.actual_qd) {
                    velocities_valid &= std::isfinite(velocity);
                    stationary &= std::fabs(velocity) <= 0.01 * 180.0 / M_PI;
                }
            }
            if (!velocities_valid) { finishFlush(FlushResult::UNSAFE_STATE); return true; }
            stationary &= rt_snapshot_.robot_state == 1 && rt_snapshot_.motion_done == 1 &&
                rt_snapshot_.mc_queue_len == 0;
            const int64_t stamp = now_ns();
            if (!stationary) { stream_stationary_since_ns_ = 0; return true; }
            if (stream_stationary_since_ns_ == 0) {
                stream_stationary_since_ns_ = stamp;
                std::copy_n(_jnt_position_state, 6, stream_stationary_min_.begin());
                stream_stationary_max_ = stream_stationary_min_;
            }
            for (size_t j = 0; j < 6; ++j) {
                stream_stationary_min_[j] = std::min(stream_stationary_min_[j], _jnt_position_state[j]);
                stream_stationary_max_[j] = std::max(stream_stationary_max_[j], _jnt_position_state[j]);
                if (stream_stationary_max_[j] - stream_stationary_min_[j] > 0.001) {
                    stream_stationary_since_ns_ = 0; return true;
                }
            }
            if (stamp - stream_stationary_since_ns_ < 200000000LL) return true;
            std::copy_n(_jnt_position_state, 6, _hold_target);
            finishFlush(FlushResult::SUCCESS);
            return true;
        }
        double max_velocity_deg_sec = 0.0;
        bool velocity_valid = true;
        for (double velocity : rt_snapshot_.actual_qd) {
            velocity_valid = velocity_valid && std::isfinite(velocity);
            max_velocity_deg_sec = std::max(max_velocity_deg_sec, std::fabs(velocity));
        }
        if (!velocity_valid) {
            finishFlush(FlushResult::UNSAFE_STATE);
            return true;
        }
        if (rt_snapshot_.robot_state == 1 &&
            max_velocity_deg_sec <= kStoppedVelocityDegSec) {
            ++flush_stationary_frames_;
        } else {
            flush_stationary_frames_ = 0;
        }
        if (flush_stationary_frames_ < kStoppedFramesRequired) return true;

        for (size_t j = 0; j < 6; ++j) {
            _hold_target[j] = _jnt_position_state[j];
            cancel_command_at_flush_[j] = _jnt_position_command[j];
        }
        if (!flush_resume_active_) {
            // Unknown/unterminated upstream goals may still be accepted late.
            // Leave servo mode ended and keep the hard inhibit until restart.
            finishFlush(FlushResult::SUCCESS);
            return true;
        }
        if (expired()) {
            finishFlush(FlushResult::TIMEOUT);
            return true;
        }
        const auto start = std::chrono::steady_clock::now();
        recordRpcBegin(START_RPC, start);
        const int rc = _ptr_robot->ServoMoveStart();
        recordRpc(START_RPC, rc, start);
        if (rc) {
            finishFlush(FlushResult::RPC_ERROR);
        } else if (expired()) {
            finishFlush(FlushResult::TIMEOUT);
        } else {
            flush_restart_frame_ = rt_snapshot_.frame_cnt;
            flush_phase_ = FlushPhase::WAIT_RESTART_FRESH;
        }
        return true;
    }
    if (rt_snapshot_.frame_cnt == flush_restart_frame_) return true;
    double restart_velocity_deg_sec = 0.0;
    for (double velocity : rt_snapshot_.actual_qd) {
        if (!std::isfinite(velocity)) {
            finishFlush(FlushResult::UNSAFE_STATE);
            return true;
        }
        restart_velocity_deg_sec = std::max(restart_velocity_deg_sec, std::fabs(velocity));
    }
    if (rt_snapshot_.robot_state != 1 ||
        restart_velocity_deg_sec > kStoppedVelocityDegSec) {
        finishFlush(FlushResult::UNSAFE_STATE);
        return true;
    }
    for (size_t j = 0; j < 6; ++j) {
        // Resync against a feedback frame sampled after ServoMoveStart.
        _hold_target[j] = _jnt_position_state[j];
        cancel_command_at_flush_[j] = _jnt_position_command[j];
    }
    if (expired()) {
        finishFlush(FlushResult::TIMEOUT);
        return true;
    }
    flush_inhibit_ = false;
    _servo_error_count = 0;
    finishFlush(FlushResult::SUCCESS);
    // Never issue a ServoJ in the same cycle that completes the flush.
    return true;
}

hardware_interface::return_type FairinoHardwareInterface::read(
    const rclcpp::Time&, const rclcpp::Duration& period) {
    trace_reported_period_sec_ = period.seconds();
    ++trace_cycle_index_;
    recordTrace(TraceKind::READ_BEGIN);
    ROBOT_STATE_PKG state{};
    const auto start = std::chrono::steady_clock::now();
    recordRpcBegin(STATE_RPC, start);
    const int rc = _ptr_robot->GetRobotRealTimeState(&state);
    recordRpc(STATE_RPC, rc, start);
    if (rc == 0) {
        trace_raw_sdk_ = state;
        auto& sdk = control_diagnostics_.sdk;
        sdk.observed = true;
        sdk.observed_at_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        sdk.frame = state.frame_cnt;
        sdk.main_code = state.main_code;
        sdk.sub_code = state.sub_code;
        sdk.program_state = state.program_state;
        sdk.robot_state = state.robot_state;
        sdk.robot_mode = state.robot_mode;
        sdk.emergency_stop = state.EmergencyStop;
        sdk.collision = state.collisionState;
        sdk.safety0 = state.safety_stop0_state;
        sdk.safety1 = state.safety_stop1_state;
        sdk.motion_done = state.motion_done;
        sdk.motion_queue_length = state.mc_queue_len;
        sdk.servo_command_count = state.servoJCmdNum;
        std::copy_n(state.jt_cur_pos, 6, sdk.joint_position_deg.begin());
        std::copy_n(state.actual_qd, 6, sdk.joint_velocity_deg_sec.begin());
    }
    rt_valid_ = rc == 0;
    for (size_t j = 0; j < 6 && rt_valid_; ++j) {
        rt_valid_ = std::isfinite(state.jt_cur_pos[j]);
    }
    const auto now = std::chrono::steady_clock::now();
    if (rt_valid_) {
        if (!rt_frame_seen_ || state.frame_cnt != rt_snapshot_.frame_cnt) {
            rt_frame_advanced_at_ = now;
            rt_frame_seen_ = true;
        }
        control_diagnostics_.state_age_sec =
            std::chrono::duration<double>(now - rt_frame_advanced_at_).count();
        rt_valid_ = control_diagnostics_.state_age_sec <= kFreshStateSec;
        if (rt_valid_) {
            rt_snapshot_ = state;
            for (size_t j = 0; j < 6; ++j) {
                _jnt_position_state[j] = state.jt_cur_pos[j] * M_PI / 180.0;
            }
            control_diagnostics_.servo_command_count = state.servoJCmdNum;
            control_diagnostics_.rt_motion_queue_length = state.mc_queue_len;
            control_diagnostics_.robot_state = state.robot_state;
            control_diagnostics_.safety0 = state.safety_stop0_state;
            control_diagnostics_.safety1 = state.safety_stop1_state;
            control_diagnostics_.frame = state.frame_cnt;
        }
    }
    rt_available_for_write_ = rt_valid_;
    recordTrace(TraceKind::READ_END, TraceRpc::NONE, rc);
    commitDiagnostics();
    if (!rt_valid_ && stream_guard_enabled_) {
        latchStream(StreamState::FEEDBACK_LOST);
        // Keep read/update/write alive so the serialized stop can be consumed.
        return hardware_interface::return_type::OK;
    }
    return rt_valid_ ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type FairinoHardwareInterface::write(
    const rclcpp::Time&, const rclcpp::Duration& period) {
    trace_reported_period_sec_ = period.seconds();
    const auto now = std::chrono::steady_clock::now();
    control_diagnostics_.loop_period_sec = period.seconds();
    control_diagnostics_.actual_loop_period_sec = last_write_at_.time_since_epoch().count() == 0
        ? 0.0 : std::chrono::duration<double>(now - last_write_at_).count();
    last_write_at_ = now;
    ++control_diagnostics_.write_cycles;
    recordTrace(TraceKind::WRITE_BEGIN);
    // Stack scope guard performs only a bounded POD copy, including early exits.
    struct CommitOnExit {
        FairinoHardwareInterface* hardware;
        ~CommitOnExit() {
            hardware->commitDiagnostics();
            hardware->commitStreamState();
            hardware->recordTrace(TraceKind::WRITE_END);
        }
    } commit{this};
    const bool fresh_read = rt_valid_ && rt_available_for_write_ &&
        std::chrono::duration<double>(now - rt_frame_advanced_at_).count() <= kFreshStateSec;
    rt_available_for_write_ = false;
    consumeStreamRequest();
    // Always consume a stop request even when feedback is unavailable.
    if (!fresh_read) {
        last_servo_target_valid_ = false;
        rt_valid_ = false;
        if (processFlush()) return hardware_interface::return_type::OK;
        return hardware_interface::return_type::ERROR;
    }
    const auto& state = rt_snapshot_;

    const bool button = flange_drag_button_enabled_ && !(state.tl_dgt_input_l & 0x01);
    if (button && !_drag_btn_prev) _drag_req.store(1);
    else if (!button && _drag_btn_prev) _drag_req.store(-1);
    _drag_btn_prev = button;
    // An owned trajectory must not resume after physical stop/teach release.
    // Latch before legacy safety/drag early returns can bypass admission.
    if (stream_control_.binding != 0 && !streamHealthy()) {
        latchStream(StreamState::UNSAFE);
    }
    // Physical stop takes precedence even if a drag RPC failed or is active.
    const bool estop = state.safety_stop0_state || state.safety_stop1_state;
    if (estop) {
        last_servo_target_valid_ = false;
        // Never replay a queued ON after stop release. Preserve an explicit OFF.
        if (_drag_req.exchange(0) < 0) _drag_req.store(-1);
        if (!_prev_estop) {
            const auto start = std::chrono::steady_clock::now();
            recordRpcBegin(STOP_RPC, start);
            const int rc = _ptr_robot->StopMotion();
            recordRpc(STOP_RPC, rc, start);
        }
        std::copy_n(_jnt_position_state, 6, _hold_target);
        std::copy_n(_jnt_position_command, 6, _pre_estop_cmd);
        _prev_estop = true;
        _post_estop_hold = true;
        _servo_error_count = 0;
        processFlush();
        return hardware_interface::return_type::OK;
    }
    _prev_estop = false;
    if (processStreamRearm()) return hardware_interface::return_type::OK;
    if (processFlush()) return hardware_interface::return_type::OK;
    const int drag_command = _drag_req.exchange(0);
    const auto block_mode_transition = [&]() {
        drag_transition_failed_ = true;
        hard_inhibit_.store(true);
        // Use the existing verified stop-only path, including queue clearance
        // and fresh stationary feedback; never rearm an uncertain mode switch.
        flush_resume_requested_.store(false);
        flush_deadline_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count() +
            static_cast<int64_t>(kFlushTimeoutSec * 1e9));
        flush_requested_.fetch_add(1);
        processFlush();
        return hardware_interface::return_type::OK;
    };
    if (drag_command == 1 && !_drag_active) {
        last_servo_target_valid_ = false;
        auto start = std::chrono::steady_clock::now();
        recordRpcBegin(END_RPC, start);
        const int end_rc = _ptr_robot->ServoMoveEnd();
        recordRpc(END_RPC, end_rc, start);
        if (end_rc != 0) return block_mode_transition();
        start = std::chrono::steady_clock::now();
        recordRpcBegin(DRAG_RPC, start);
        const int drag_rc = _ptr_robot->DragTeachSwitch(1);
        recordRpc(DRAG_RPC, drag_rc, start);
        if (drag_rc != 0) return block_mode_transition();
        _drag_active = true;
        _servo_error_count = 0;
    } else if (drag_command == -1 && _drag_active) {
        last_servo_target_valid_ = false;
        auto start = std::chrono::steady_clock::now();
        recordRpcBegin(DRAG_RPC, start);
        const int drag_rc = _ptr_robot->DragTeachSwitch(0);
        recordRpc(DRAG_RPC, drag_rc, start);
        if (drag_rc != 0) return block_mode_transition();
        start = std::chrono::steady_clock::now();
        recordRpcBegin(START_RPC, start);
        const int start_rc = _ptr_robot->ServoMoveStart();
        recordRpc(START_RPC, start_rc, start);
        if (start_rc != 0) return block_mode_transition();
        for (size_t j = 0; j < 6; ++j) {
            _hold_target[j] = _jnt_position_state[j];
            _pre_drag_cmd[j] = _jnt_position_command[j];
        }
        _post_drag_hold = true;
        _drag_active = false;
        _servo_error_count = 0;
    }
    if (_drag_active) {
        std::copy_n(_jnt_position_state, 6, _hold_target);
        return hardware_interface::return_type::OK;
    }
    if (_control_mode != 0) {
        last_servo_target_valid_ = false;
        if (_control_mode == 1 && std::none_of(_jnt_torque_command, _jnt_torque_command + 6,
                [](double value) { return !std::isfinite(value); })) {
            return hardware_interface::return_type::OK;  // Existing reserved torque-mode no-op.
        }
        return hardware_interface::return_type::ERROR;
    }
    if (std::any_of(_jnt_position_command, _jnt_position_command + 6,
                    [](double value) { return !std::isfinite(value); })) {
        last_servo_target_valid_ = false;
        return hardware_interface::return_type::ERROR;
    }
    const bool drag = state.robot_state == 4;
    if (drag) {
        last_servo_target_valid_ = false;
        if (!_prev_drag) std::copy_n(_jnt_position_command, 6, _pre_drag_cmd);
        std::copy_n(_jnt_position_state, 6, _hold_target);
        _prev_drag = true;
        _post_drag_hold = true;
        _servo_error_count = 0;
        return hardware_interface::return_type::OK;
    }
    _prev_drag = false;
    const double* target = _jnt_position_command;
    if (_post_drag_hold) {
        double deviation = 0.0;
        for (size_t j = 0; j < 6; ++j) {
            deviation = std::max(deviation, std::fabs(_jnt_position_command[j] - _pre_drag_cmd[j]));
        }
        if (deviation < 0.005) target = _hold_target;
        else _post_drag_hold = false;
    }
    if (_post_estop_hold) {
        double deviation = 0.0;
        for (size_t j = 0; j < 6; ++j) {
            deviation = std::max(deviation, std::fabs(_jnt_position_command[j] - _pre_estop_cmd[j]));
        }
        if (deviation < 0.005) target = _hold_target;
        else _post_estop_hold = false;
    }
    if (cancel_hold_) {
        double actual_error = 0.0, command_change = 0.0;
        for (size_t j = 0; j < 6; ++j) {
            actual_error = std::max(actual_error, std::fabs(_jnt_position_command[j] - _jnt_position_state[j]));
            command_change = std::max(command_change, std::fabs(
                _jnt_position_command[j] - cancel_command_at_flush_[j]));
        }
        // Hardware has no JTC goal identity. A changed command can release
        // cancel hold only if every joint starts within 0.005 rad of fresh
        // actual feedback. Evolving old distant setpoints remain held.
        if (command_change > 1e-9 && actual_error <= kCancelReleaseToleranceRad) {
            cancel_hold_ = false;
        } else {
            target = _hold_target;
        }
    }
    JointPos command{};
    ExaxisPos external{0, 0, 0, 0};
    control_diagnostics_.target_state_error_rad = 0.0;
    for (size_t j = 0; j < 6; ++j) {
        command.jPos[j] = target[j] * 180.0 / M_PI;
        control_diagnostics_.target_state_error_rad = std::max(
            control_diagnostics_.target_state_error_rad,
            std::fabs(target[j] - _jnt_position_state[j]));
    }
    if (stream_guard_enabled_ && !stream_latched_.load() && !streamHealthy()) {
        latchStream(StreamState::UNSAFE);
        processFlush();
        return hardware_interface::return_type::OK;
    }
    // Repeated successful hold points can queue ahead of the next trajectory.
    // Suppress exact duplicates only: every changed path point still goes out.
    if (last_servo_target_valid_ &&
        std::equal(last_servo_target_.begin(), last_servo_target_.end(), target)) {
        ++control_diagnostics_.duplicate_servo_targets_skipped;
        return hardware_interface::return_type::OK;
    }
    if (stream_guard_enabled_) {
        if (stream_latched_.load()) return hardware_interface::return_type::OK;
        // Duplicate hold suppression creates benign long gaps between SDK
        // calls. Detect a missed write cadence only when a new point is due.
        if (last_servo_target_valid_ &&
            control_diagnostics_.actual_loop_period_sec >= stream_delay_threshold_sec_) {
            latchStream(StreamState::DELAY, control_diagnostics_.actual_loop_period_sec);
            processFlush();
            return hardware_interface::return_type::OK;
        }
    }
    const auto start = std::chrono::steady_clock::now();
    if (stream_guard_enabled_) stream_rpc_begin_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
        start.time_since_epoch()).count());
    std::copy_n(target, 6, control_diagnostics_.last_servo_target_rad.begin());
    control_diagnostics_.servo_target_observed = true;
    recordRpcBegin(SERVO_RPC, start);
    const int rc = _ptr_robot->ServoJ(&command, &external, 0, 0,
                                    static_cast<float>(servo_command_period_sec_), 0, 0);
    recordRpc(SERVO_RPC, rc, start);
    stream_rpc_begin_ns_.store(0);
    const double call_duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (stream_guard_enabled_ && (rc != 0 || call_duration >= stream_delay_threshold_sec_)) {
        latchStream(rc != 0 ? StreamState::SDK_ERROR : StreamState::DELAY, call_duration);
        return hardware_interface::return_type::OK;
    }
    if (rc != 0) {
        last_servo_target_valid_ = false;
        ++_servo_error_count;
        if (_servo_error_count <= 3) {
            auto recovery_start = std::chrono::steady_clock::now();
            recordRpcBegin(RESET_RPC, recovery_start);
            const int reset_rc = _ptr_robot->ResetAllError();
            recordRpc(RESET_RPC, reset_rc, recovery_start);
            recovery_start = std::chrono::steady_clock::now();
            recordRpcBegin(START_RPC, recovery_start);
            const int start_rc = _ptr_robot->ServoMoveStart();
            recordRpc(START_RPC, start_rc, recovery_start);
        }
    } else {
        std::copy_n(target, last_servo_target_.size(), last_servo_target_.begin());
        last_servo_target_valid_ = true;
        trace_successful_target_observed_ = true;
        _servo_error_count = 0;
    }
    return hardware_interface::return_type::OK;
}

}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
