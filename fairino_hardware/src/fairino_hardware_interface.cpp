#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware{

FairinoHardwareInterface::~FairinoHardwareInterface() { stopServiceThread(); }

void FairinoHardwareInterface::stopServiceThread() {
    if (_svc_exec) _svc_exec->cancel();
    if (_svc_spin_thread.joinable()) _svc_spin_thread.join();
    if (_svc_exec && _svc_node) _svc_exec->remove_node(_svc_node);
    _svc_exec.reset();
    diagnostics_timer_.reset();
    diagnostics_pub_.reset();
    flush_srv_.reset();
    stop_hold_srv_.reset();
    flush_callback_group_.reset();
    _drag_srv.reset();
    _svc_node.reset();
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量

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
        flush_phase_ = FlushPhase::IDLE;
        flush_active_generation_ = 0;
        flush_requested_.store(0);
        flush_completed_.store(0);
        flush_result_.store(static_cast<int>(FlushResult::NONE));
        flush_resume_requested_.store(true);
        hard_inhibit_.store(false);
        cancel_hold_ = false;
        flush_inhibit_ = false;
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
        diagnostics_pub_ = _svc_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(10));
        diagnostics_timer_ = _svc_node->create_wall_timer(100ms, [this]() { publishDiagnostics(); });
        // ★서비스 spin 을 별도 스레드로 — 실시간 write() 루프서 spin_some 을 부르던 걸 대체(지터 제거).
        _svc_exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
        _svc_exec->add_node(_svc_node);
        _svc_spin_thread = std::thread([this](){ _svc_exec->spin(); });
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
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    // ★백그라운드 서비스 spin 정지: executor cancel → 스레드 join → 리소스 해제(순서 중요).
    stopServiceThread();
    _ptr_robot->ServoMoveEnd();//退出伺服模式
    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    _ptr_robot.reset();
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
    control_diagnostics_.flush_generation = flush_requested_.load();
    control_diagnostics_.flush_completed_generation = flush_completed_.load();
    control_diagnostics_.flush_result = flush_result_.load();
    control_diagnostics_.flush_phase = static_cast<int>(flush_phase_);
    // The timer copies this POD under the mutex, then releases it before doing
    // any allocation, formatting or ROS publication. Never block the RT loop.
    std::unique_lock<std::mutex> lock(diagnostics_mutex_, std::try_to_lock);
    if (lock.owns_lock()) published_diagnostics_ = control_diagnostics_;
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
    if (!snapshot.state_valid || sample_age_sec > 0.2 || snapshot.flush_inhibit ||
        snapshot.rpc[SERVO_RPC].rc != 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "state/servo/flush blocked";
    } else if (snapshot.cancel_hold || snapshot.drag_hold || snapshot.estop_hold ||
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
    add("flush_generation", snapshot.flush_generation);
    add("flush_completed_generation", snapshot.flush_completed_generation);
    add("flush_result", snapshot.flush_result);
    add("flush_phase", snapshot.flush_phase);
    add("cancel_hold", snapshot.cancel_hold);
    add("flush_inhibit", snapshot.flush_inhibit);
    add("hard_inhibit", snapshot.hard_inhibit);
    add("post_drag_hold", snapshot.drag_hold);
    add("post_estop_hold", snapshot.estop_hold);
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
    response.success = completed && result == static_cast<int>(FlushResult::SUCCESS);
    response.message = std::string(resume_after_stop ? "stop_and_flush" : "stop_and_hold") +
        " generation=" + std::to_string(generation) +
        (completed ? " result=" + std::to_string(result)
                   : " timeout; completion unconfirmed, motion remains blocked");
    flush_callback_busy_.store(false);
}

void FairinoHardwareInterface::finishFlush(FlushResult result) {
    flush_phase_ = FlushPhase::IDLE;
    if (result != FlushResult::SUCCESS) flush_inhibit_ = true;
    flush_result_.store(static_cast<int>(result));
    flush_completed_.store(flush_active_generation_);
}

bool FairinoHardwareInterface::processFlush() {
    const auto now_ns = []() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };
    const auto expired = [&]() { return now_ns() >= flush_deadline_ns_.load(); };
    const uint64_t requested = flush_requested_.load();
    if (requested > flush_active_generation_) {
        flush_active_generation_ = requested;
        flush_resume_active_ = flush_resume_requested_.load();
        if (!flush_resume_active_) hard_inhibit_.store(true);
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
        const int stop_rc = _ptr_robot->StopMotion();
        recordRpc(STOP_RPC, stop_rc, start);
        start = std::chrono::steady_clock::now();
        const int end_rc = _ptr_robot->ServoMoveEnd();
        recordRpc(END_RPC, end_rc, start);
        start = std::chrono::steady_clock::now();
        const int clear_rc = _ptr_robot->MotionQueueClear();
        recordRpc(CLEAR_RPC, clear_rc, start);
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
        !(rt_snapshot_.tl_dgt_input_l & 1) || rt_snapshot_.safety_stop0_state ||
        rt_snapshot_.safety_stop1_state) {
        finishFlush(FlushResult::UNSAFE_STATE);
        return true;
    }
    if (flush_phase_ == FlushPhase::WAIT_STATIONARY) {
        if (rt_snapshot_.frame_cnt == flush_stationary_frame_) return true;
        flush_stationary_frame_ = rt_snapshot_.frame_cnt;
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
    const rclcpp::Time&, const rclcpp::Duration&) {
    ROBOT_STATE_PKG state{};
    const auto start = std::chrono::steady_clock::now();
    const int rc = _ptr_robot->GetRobotRealTimeState(&state);
    recordRpc(STATE_RPC, rc, start);
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
            control_diagnostics_.robot_state = state.robot_state;
            control_diagnostics_.safety0 = state.safety_stop0_state;
            control_diagnostics_.safety1 = state.safety_stop1_state;
            control_diagnostics_.frame = state.frame_cnt;
        }
    }
    rt_available_for_write_ = rt_valid_;
    commitDiagnostics();
    return rt_valid_ ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type FairinoHardwareInterface::write(
    const rclcpp::Time&, const rclcpp::Duration& period) {
    const auto now = std::chrono::steady_clock::now();
    control_diagnostics_.loop_period_sec = period.seconds();
    control_diagnostics_.actual_loop_period_sec = last_write_at_.time_since_epoch().count() == 0
        ? 0.0 : std::chrono::duration<double>(now - last_write_at_).count();
    last_write_at_ = now;
    ++control_diagnostics_.write_cycles;
    // Stack scope guard performs only a bounded POD copy, including early exits.
    struct CommitOnExit {
        FairinoHardwareInterface* hardware;
        ~CommitOnExit() { hardware->commitDiagnostics(); }
    } commit{this};
    const bool fresh_read = rt_valid_ && rt_available_for_write_ &&
        std::chrono::duration<double>(now - rt_frame_advanced_at_).count() <= kFreshStateSec;
    rt_available_for_write_ = false;
    // Always consume a stop request even when feedback is unavailable.
    if (!fresh_read) {
        rt_valid_ = false;
        if (processFlush()) return hardware_interface::return_type::OK;
        return hardware_interface::return_type::ERROR;
    }
    if (processFlush()) return hardware_interface::return_type::OK;
    const auto& state = rt_snapshot_;

    const bool button = !(state.tl_dgt_input_l & 0x01);
    if (button && !_drag_btn_prev) _drag_req.store(1);
    else if (!button && _drag_btn_prev) _drag_req.store(-1);
    _drag_btn_prev = button;
    const int drag_command = _drag_req.exchange(0);
    if (drag_command == 1 && !_drag_active) {
        auto start = std::chrono::steady_clock::now();
        const int end_rc = _ptr_robot->ServoMoveEnd();
        recordRpc(END_RPC, end_rc, start);
        start = std::chrono::steady_clock::now();
        const int drag_rc = _ptr_robot->DragTeachSwitch(1);
        recordRpc(DRAG_RPC, drag_rc, start);
        _drag_active = true;
        _servo_error_count = 0;
    } else if (drag_command == -1 && _drag_active) {
        auto start = std::chrono::steady_clock::now();
        const int drag_rc = _ptr_robot->DragTeachSwitch(0);
        recordRpc(DRAG_RPC, drag_rc, start);
        start = std::chrono::steady_clock::now();
        const int start_rc = _ptr_robot->ServoMoveStart();
        recordRpc(START_RPC, start_rc, start);
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
        if (_control_mode == 1 && std::none_of(_jnt_torque_command, _jnt_torque_command + 6,
                [](double value) { return !std::isfinite(value); })) {
            return hardware_interface::return_type::OK;  // Existing reserved torque-mode no-op.
        }
        return hardware_interface::return_type::ERROR;
    }
    if (std::any_of(_jnt_position_command, _jnt_position_command + 6,
                    [](double value) { return !std::isfinite(value); })) {
        return hardware_interface::return_type::ERROR;
    }
    const bool drag = state.robot_state == 4;
    const bool estop = state.safety_stop0_state || state.safety_stop1_state;
    if (drag) {
        if (!_prev_drag) std::copy_n(_jnt_position_command, 6, _pre_drag_cmd);
        std::copy_n(_jnt_position_state, 6, _hold_target);
        _prev_drag = true;
        _post_drag_hold = true;
        _servo_error_count = 0;
        return hardware_interface::return_type::OK;
    }
    _prev_drag = false;
    if (estop) {
        if (!_prev_estop) {
            const auto start = std::chrono::steady_clock::now();
            const int rc = _ptr_robot->StopMotion();
            recordRpc(STOP_RPC, rc, start);
        }
        std::copy_n(_jnt_position_state, 6, _hold_target);
        std::copy_n(_jnt_position_command, 6, _pre_estop_cmd);
        _prev_estop = true;
        _post_estop_hold = true;
        _servo_error_count = 0;
        return hardware_interface::return_type::OK;
    }
    _prev_estop = false;

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
    const auto start = std::chrono::steady_clock::now();
    const int rc = _ptr_robot->ServoJ(&command, &external, 0, 0,
                                    static_cast<float>(servo_command_period_sec_), 0, 0);
    recordRpc(SERVO_RPC, rc, start);
    if (rc != 0) {
        ++_servo_error_count;
        if (_servo_error_count <= 3) {
            auto recovery_start = std::chrono::steady_clock::now();
            const int reset_rc = _ptr_robot->ResetAllError();
            recordRpc(RESET_RPC, reset_rc, recovery_start);
            recovery_start = std::chrono::steady_clock::now();
            const int start_rc = _ptr_robot->ServoMoveStart();
            recordRpc(START_RPC, start_rc, recovery_start);
        }
    } else {
        _servo_error_count = 0;
    }
    return hardware_interface::return_type::OK;
}

}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
