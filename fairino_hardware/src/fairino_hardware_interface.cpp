#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量

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



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting ...please wait...");
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
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"初始指令位置: %f,%f,%f,%f,%f,%f",_jnt_position_command[0],\
        _jnt_position_command[1],_jnt_position_command[2],_jnt_position_command[3],_jnt_position_command[4],_jnt_position_command[5]);
        // Enter servo mode before ServoJ commands
        returncode = _ptr_robot->ServoMoveStart();
        if(returncode != 0){
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "ServoMoveStart failed (error=%d)", returncode);
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoMoveStart OK. Hardware activated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    _ptr_robot->ServoMoveEnd();//退出伺服模式
    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    _ptr_robot.release();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
    JointPos state_data;
    error_t returncode = _ptr_robot->GetActualJointPosDegree(1,&state_data);
    if(returncode == 0){
        for(int i=0;i<6;i++){
            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;//注意单位转换，moveit统一用弧度
            //_jnt_torque_state[i] = state_data.jt_cur_tor[i];//注意单位转换
        }
    }else{
        return hardware_interface::return_type::ERROR;
    }
    //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
    _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    if(_control_mode == 0){//位置控制模式
        if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        // ── 손 티칭(펜던트 DRAG) 공존 ────────────────────────────────────────────
        // 드래그 중(robot_state==4)엔 ServoJ 도, 그 error=14 복구(ResetAllError+ServoMoveStart)도
        // 보내지 않는다 — 예전엔 이 복구가 매 사이클 드래그를 걷어차 팔을 옛 위치로 되돌렸다.
        // 종료 후엔 JTC 가 옛 홀드 setpoint 를 유지하는 동안(command 가 진입 스냅샷과 동일) 옮긴 위치를
        // 유지해 스프링백을 막고, 새 goal 이 오면(command 변화) 정상 ServoJ 로 복귀한다. FR5 JTC 는
        // closed-loop(open_loop_control 미설정)라 새 goal 이 측정위치서 시작 → 유지 해제가 이음매 없다.
        ROBOT_STATE_PKG _rt_pkg;
        bool _drag = false, _estop = false;
        if(_ptr_robot->GetRobotRealTimeState(&_rt_pkg) == 0){
            _drag = (_rt_pkg.robot_state == 4);   // 4 = 拖动(드래그 티치)
            _estop = (_rt_pkg.safety_stop0_state != 0 || _rt_pkg.safety_stop1_state != 0);  // 안전정지 SI0/SI1
        }
        if(_drag){
            if(!_prev_drag){                       // 드래그 진입 엣지 — 스프링백 목표 스냅샷
                for(int i=0;i<6;i++) _pre_drag_cmd[i] = _jnt_position_command[i];
            }
            for(int i=0;i<6;i++) _hold_target[i] = _jnt_position_state[i];  // 옮긴 위치 추적
            _prev_drag = true;
            _post_drag_hold = true;
            _servo_error_count = 0;
            return hardware_interface::return_type::OK;   // ServoJ/복구 스킵 → 드래그와 안 싸움
        }
        _prev_drag = false;

        // ── 안전정지(e-stop) anti-springback ─────────────────────────────────────
        // e-stop(safety_stop SI0/SI1)을 ServoJ 보내기 *전에* 감지한다(반응형 error=99 가 아니라 사전
        // 감지). 감지 즉시 StopMotion() 으로 FR5 내부 모션을 abort 해 릴리스 때 재개할 옛 궤적(A)을
        // 없애고, 드래그와 같은 원리로 정지 위치(B)를 _hold_target 에 래치한 뒤 ServoJ 를 스킵한다
        // (return OK — 옛 setpoint 명령이 절대 안 나감). 해제 후엔 _post_estop_hold 로 새 goal 전까지
        // B 를 유지 → 릴리스 시 A 로 튀지 않고 B 에 머문다. 새 goal 이 오면(command 변화) 정상 복귀.
        if(_estop){
            if(!_prev_estop){                          // e-stop 진입 엣지 — 내부 모션 abort(재개할 A 제거)
                _ptr_robot->StopMotion();
            }
            for(int i=0;i<6;i++){
                _hold_target[i]   = _jnt_position_state[i];    // 정지 위치(B)
                _pre_estop_cmd[i] = _jnt_position_command[i];  // JTC 현재 command — 새 goal 판정 기준
            }
            _prev_estop = true;
            _post_estop_hold = true;
            _servo_error_count = 0;
            return hardware_interface::return_type::OK;        // ServoJ 스킵 → A 명령 안 나감
        }
        _prev_estop = false;

        const double* _tgt = _jnt_position_command;
        if(_post_drag_hold){
            double _dev = 0.0;
            for(int i=0;i<6;i++) _dev = std::max(_dev, std::fabs(_jnt_position_command[i]-_pre_drag_cmd[i]));
            if(_dev < 0.005){ _tgt = _hold_target; }   // JTC 아직 옛 setpoint 홀드 → 옮긴 위치 유지
            else { _post_drag_hold = false; }          // 새 goal 도착 → 유지 해제, 정상 복귀
        }
        if(_post_estop_hold){                          // e-stop 해제 후: 새 goal 전까지 B 유지
            double _dev = 0.0;
            for(int i=0;i<6;i++) _dev = std::max(_dev, std::fabs(_jnt_position_command[i]-_pre_estop_cmd[i]));
            if(_dev < 0.005){ _tgt = _hold_target; }
            else { _post_estop_hold = false; }         // JTC 새 궤적 시작 → 해제, 정상 복귀
        }
        JointPos cmd;
        ExaxisPos extcmd{0,0,0,0};
        for(auto j=0;j<6;j++){
            cmd.jPos[j] = _tgt[j]/M_PI*180; //注意单位转换
        }
        // cmdT 를 update_rate(50Hz=20ms) 에 맞춘다(2026-09-07). 옛 0.008(8ms=125Hz)은 update_rate 를
        // 50 으로 낮출 때 안 고쳐진 잔재라, 서보가 8ms 만에 도달 후 12ms 멈춤을 반복해 **뚝뚝 끊기고**
        // 20ms 간격 궤적 스텝을 8ms 안에 못 닿아 **ServoJ error=14** 를 냈다. 0.02 로 20ms 내내 연속 이동.
        int returncode = _ptr_robot->ServoJ(&cmd,&extcmd,0,0,0.02,0,0);
        if(returncode != 0){
            _servo_error_count++;
            if(_servo_error_count <= 3){
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ error=%d, attempting recovery (%d)...", returncode, _servo_error_count);
                _ptr_robot->ResetAllError();
                _ptr_robot->ServoMoveStart();
            } else if(_servo_error_count % 500 == 0){
                // Throttle logging after initial retries
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ error=%d persists (count=%d)", returncode, _servo_error_count);
            }
        } else {
            if(_servo_error_count > 0){
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ recovered after %d errors", _servo_error_count);
            }
            _servo_error_count = 0;
        }
    }else if(_control_mode == 1){//扭矩控制模式
        if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        //_ptr_robot->write(_jnt_torque_command);//注意单位转换
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
        return hardware_interface::return_type::ERROR;
    }
 
    return hardware_interface::return_type::OK;
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
