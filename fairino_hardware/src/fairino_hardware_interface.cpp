#include <limits>

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
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "[servo-diagnostics] build=2026-09-10-v2 cmdT=0.020000 s "
                "hold_release_threshold=0.005000 rad");
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
        // ★서비스 spin 을 별도 스레드로 — 실시간 write() 루프서 spin_some 을 부르던 걸 대체(지터 제거).
        _svc_exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
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



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    // ★백그라운드 서비스 spin 정지: executor cancel → 스레드 join → 리소스 해제(순서 중요).
    if(_svc_exec){ _svc_exec->cancel(); }
    if(_svc_spin_thread.joinable()){ _svc_spin_thread.join(); }
    if(_svc_exec && _svc_node){ _svc_exec->remove_node(_svc_node); }
    _svc_exec.reset();
    _drag_srv.reset();          // ★손교시 언락 서비스 정리
    _svc_node.reset();
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
    // ★손교시 SW 언락: 서비스 spin 은 백그라운드 스레드(_svc_exec)가 처리 — 여기선 원자변수만 읽는다
    //   (실시간 루프에 spin_some 을 넣으면 executor 생성·파괴 지터로 ServoJ 가 뚝뚝 끊긴다). 요청 시
    //   DragTeachSwitch(SDK)는 write 스레드에서 호출(SDK 단독 스레드 → 레이스 없음). read-and-clear 로
    //   콜백이 세팅한 요청을 한 번에 소비(놓침 없음). 드래그 ON → robot_state==4 공존이 ServoJ 스킵.
    // ★플랜지 드래그 버튼 hold-to-drag(사용자 2026-09-09): RT 상태를 루프 시작에서 1회 읽어(아래 estop/
    //   drag 검출과 공용) tl_dgt_input_l bit0(active-LOW: 눌림=0)의 엣지를 본다. 눌림 엣지→_drag_req=1,
    //   뗌 엣지→_drag_req=-1. 서비스와 공존. 버튼이 서보모드서 막혀도 여기서 감지해 ServoMoveEnd 로
    //   드래그 진입시킨다(catch-22 해소). 드래그 중에도(아래 early-return 전) 매 사이클 읽어 뗌을 놓치지 않는다.
    // ★[servo-freeze 진단 2026-09-10] 침묵 skip 분기(robot_state==4·safety_stop·post-hold)가 ServoJ 를
    //   조용히 건너뛰어(return OK) visual_servo 가 'no raw joint progress' 로 abort 하는 원인을, 다음
    //   실험 bag 에서 보이게 로그로 노출한다. ⚠RT 50Hz 루프라 THROTTLE(2s, steady clock)로 스팸·지터 방지.
    static rclcpp::Clock _skip_clk(RCL_STEADY_TIME);
    ROBOT_STATE_PKG _rt_pkg;
    const bool _rt_ok = (_ptr_robot->GetRobotRealTimeState(&_rt_pkg) == 0);
    if(_rt_ok){
        const bool _btn = ((_rt_pkg.tl_dgt_input_l & 0x01) == 0);   // active-LOW: 눌림=bit0=0
        if(_btn && !_drag_btn_prev)       _drag_req.store(1);       // 누름 엣지 → 드래그 ON
        else if(!_btn && _drag_btn_prev)  _drag_req.store(-1);      // 뗌 엣지 → 드래그 OFF(hold 해제)
        _drag_btn_prev = _btn;
    }
    const int _drag_cmd = _drag_req.exchange(0);
    if(_drag_cmd == 1 && !_drag_active){
        // 드래그 진입: ServoMove 가 켜져 있으면(서보 모드) DragTeachSwitch 만으론 드래그모드 진입이
        // 막힌다 → 먼저 ServoMoveEnd 로 서보를 풀고 DragTeachSwitch(1). 같은 연결·write 스레드라
        // 소켓 재생성 없음(SIGSEGV 는 CloseRPC+RPC 재연결 때였음).
        errno_t _r1 = _ptr_robot->ServoMoveEnd();
        errno_t _r2 = _ptr_robot->DragTeachSwitch(1);
        _drag_active = true; _servo_error_count = 0;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "드래그 ON: ServoMoveEnd rc=%d + DragTeachSwitch(1) rc=%d — 버튼+손으로 이동", (int)_r1, (int)_r2);
    } else if(_drag_cmd == -1 && _drag_active){
        // 드래그 종료: DragTeachSwitch(0) → ServoMoveStart 로 서보 재개.
        // ★스프링백 방지(2026-09-09): 펜던트 드래그와 **동일하게** _post_drag_hold 를 켠다. 한 사이클
        //   _jnt_position_command 채택만으론 다음 사이클 JTC 가 옛 setpoint 를 재발행해 팔이 옛 위치로
        //   튀며 ServoJ error=14 가 무한 루프했다(SW OFF 에만 이 hold 가 빠져 있던 게 원인). 이제 새
        //   goal(command 가 _pre_drag_cmd 서 0.005rad 넘게 벗어남) 전까지 _hold_target(손교시로 옮긴
        //   현재 위치)을 ServoJ 목표로 유지 → 스프링백·error=14 루프 제거. (아래 post_drag_hold 실행부 공용.)
        errno_t _r1 = _ptr_robot->DragTeachSwitch(0);
        errno_t _r2 = _ptr_robot->ServoMoveStart();
        for(int i=0;i<6;i++){
            _hold_target[i]  = _jnt_position_state[i];    // 유지 목표 = 현재(손교시로 옮긴) 위치
            _pre_drag_cmd[i] = _jnt_position_command[i];  // JTC 옛 command(=옛자세) = 새 goal 판정 기준
            // ★_jnt_position_command 를 채택(=현재자세)하면 안 된다: 아래 post_drag_hold 검사
            //   _dev=|_jnt_position_command − _pre_drag_cmd| 가 |손교시자세−옛자세|=큰값이 돼 hold 를
            //   **즉시 해제**→JTC 옛 setpoint 로 튄다(버튼 떼면 원래대로 복귀 버그). command 는 JTC 값
            //   (옛자세=_pre_drag_cmd) 그대로 둬야 _dev=0→hold 유지→_tgt=_hold_target(손교시자세)로 머문다.
        }
        _post_drag_hold = true;                           // 새 goal 전까지 현재 위치 유지(펜던트와 동일)
        _drag_active = false; _servo_error_count = 0;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "드래그 OFF: DragTeachSwitch(0) rc=%d + ServoMoveStart rc=%d (현재자세 유지·post_drag_hold)", (int)_r1, (int)_r2);
    }
    if(_drag_active){
        for(int i=0;i<6;i++) _hold_target[i] = _jnt_position_state[i];   // 옮긴 위치 추적(off 시 채택)
        return hardware_interface::return_type::OK;                     // 드래그 중 ServoJ 전면 스킵
    }
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
        // 유지해 스프링백을 막고, command 변화가 해제 임계값에 도달하면 정상 ServoJ로 복귀한다.
        // FR5 JTC는 open_loop_control=true이므로 새 goal이 측정위치에서 시작한다고 가정할 수 없다.
        // ★RT 상태는 write() 상단에서 이미 1회 읽었다(_rt_pkg, _rt_ok) — 버튼·estop·drag 공용(중복 RPC 제거).
        const bool _drag  = _rt_ok && (_rt_pkg.robot_state == 4);   // 4 = 拖动(펜던트/버튼 드래그)
        const bool _estop = _rt_ok && (_rt_pkg.safety_stop0_state != 0 || _rt_pkg.safety_stop1_state != 0);  // SI0/SI1
        if(_drag){
            if(!_prev_drag){                       // 드래그 진입 엣지 — 스프링백 목표 스냅샷
                for(int i=0;i<6;i++) _pre_drag_cmd[i] = _jnt_position_command[i];
            }
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("FairinoHardwareInterface"), _skip_clk, 2000,
                "[servo-skip] robot_state==4(드래그 모드) — ServoJ 스킵, 팔이 JTC/서보 명령을 안 따름. "
                "드래그 미해제 의심(visual_servo 는 'no raw joint progress'로 abort 함).");
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
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("FairinoHardwareInterface"), _skip_clk, 2000,
                "[servo-skip] safety_stop SI0=%d SI1=%d — StopMotion+ServoJ 스킵, 팔 정지(안전정지 해제 필요; "
                "visual_servo 는 'no raw joint progress'로 abort 함).",
                (int)_rt_pkg.safety_stop0_state, (int)_rt_pkg.safety_stop1_state);
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

        const bool _had_drag_hold = _post_drag_hold;
        const bool _had_estop_hold = _post_estop_hold;
        double _cmd_pre_drag_max = 0.0, _cmd_pre_estop_max = 0.0;
        double _cmd_hold_max = 0.0, _cmd_state_max = 0.0;
        int _worst_cmd_state_joint = 0;  // zero-based joint index for tracking diagnostics
        for(int i=0;i<6;i++){
            const double _cmd_state_dev = std::fabs(_jnt_position_command[i] - _jnt_position_state[i]);
            if(_cmd_state_dev > _cmd_state_max){
                _cmd_state_max = _cmd_state_dev;
                _worst_cmd_state_joint = i;
            }
            if(_had_drag_hold || _had_estop_hold){
                _cmd_pre_drag_max = std::max(_cmd_pre_drag_max, std::fabs(_jnt_position_command[i] - _pre_drag_cmd[i]));
                _cmd_pre_estop_max = std::max(_cmd_pre_estop_max, std::fabs(_jnt_position_command[i] - _pre_estop_cmd[i]));
                _cmd_hold_max = std::max(_cmd_hold_max, std::fabs(_jnt_position_command[i] - _hold_target[i]));
            }
        }
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
        if((_had_drag_hold && !_post_drag_hold) || (_had_estop_hold && !_post_estop_hold)){
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "[servo-hold-release] released_drag=%d released_estop=%d post_drag_hold=%d post_estop_hold=%d "
                "max_abs(command-pre_drag)=%.6f rad max_abs(command-pre_estop)=%.6f rad "
                "max_abs(command-hold)=%.6f rad max_abs(command-state)=%.6f rad held_target_selected=%d",
                (int)(_had_drag_hold && !_post_drag_hold), (int)(_had_estop_hold && !_post_estop_hold),
                (int)_post_drag_hold, (int)_post_estop_hold, _cmd_pre_drag_max, _cmd_pre_estop_max,
                _cmd_hold_max, _cmd_state_max, (int)(_tgt == _hold_target));
        }
        if(_tgt == _hold_target){                         // post-drag/estop hold: JTC 명령 무시하고 정지자세 유지
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("FairinoHardwareInterface"), _skip_clk, 2000,
                "[servo-hold] ServoJ is sending held target; post_drag_hold=%d post_estop_hold=%d "
                "max_abs(command-pre_drag)=%.6f rad max_abs(command-pre_estop)=%.6f rad "
                "max_abs(command-hold)=%.6f rad max_abs(command-state)=%.6f rad release_threshold=0.005000 rad",
                (int)_post_drag_hold, (int)_post_estop_hold, _cmd_pre_drag_max, _cmd_pre_estop_max,
                _cmd_hold_max, _cmd_state_max);
        }
        JointPos cmd;
        ExaxisPos extcmd{0,0,0,0};
        double _target_state_max = 0.0;
        for(auto j=0;j<6;j++){
            cmd.jPos[j] = _tgt[j]/M_PI*180; //注意单位转换
            _target_state_max = std::max(_target_state_max, std::fabs(_tgt[j] - _jnt_position_state[j]));
        }
        // cmdT 를 update_rate(50Hz=20ms) 에 맞춘다(2026-09-07). 옛 0.008(8ms=125Hz)은 update_rate 를
        // 50 으로 낮출 때 안 고쳐진 잔재라, 서보가 8ms 만에 도달 후 12ms 멈춤을 반복해 **뚝뚝 끊기고**
        // 20ms 간격 궤적 스텝을 8ms 안에 못 닿아 **ServoJ error=14** 를 냈다. 0.02 로 20ms 내내 연속 이동.
        int returncode = _ptr_robot->ServoJ(&cmd,&extcmd,0,0,0.02,0,0);
        if(_cmd_state_max >= 0.005 || _target_state_max >= 0.005){
            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("FairinoHardwareInterface"), _skip_clk, 2000,
                "[servo-tracking] rc=%d post_drag_hold=%d post_estop_hold=%d held_target_selected=%d "
                "rt_ok=%d robot_state=%d safety0=%d safety1=%d rt_frame=%d main_code=%d sub_code=%d "
                "servoJCmdNum=%d rt_lastServoTarget_raw=%.6f rt_joint_position_deg=%.6f "
                "max_abs(command-state)=%.6f rad max_abs(target-state)=%.6f rad "
                "worst_command_state_joint_index=%d command=%.6f rad target=%.6f rad state=%.6f rad "
                "write_period=%.6f s cmdT=0.020000 s",
                returncode, (int)_post_drag_hold, (int)_post_estop_hold, (int)(_tgt == _hold_target),
                (int)_rt_ok, _rt_ok ? (int)_rt_pkg.robot_state : -1,
                _rt_ok ? (int)_rt_pkg.safety_stop0_state : -1, _rt_ok ? (int)_rt_pkg.safety_stop1_state : -1,
                _rt_ok ? (int)_rt_pkg.frame_cnt : -1, _rt_ok ? _rt_pkg.main_code : -1,
                _rt_ok ? _rt_pkg.sub_code : -1, _rt_ok ? _rt_pkg.servoJCmdNum : -1,
                _rt_ok ? _rt_pkg.lastServoTarget[_worst_cmd_state_joint] : std::numeric_limits<double>::quiet_NaN(),
                _rt_ok ? _rt_pkg.jt_cur_pos[_worst_cmd_state_joint] : std::numeric_limits<double>::quiet_NaN(),
                _cmd_state_max, _target_state_max, _worst_cmd_state_joint,
                _jnt_position_command[_worst_cmd_state_joint], _tgt[_worst_cmd_state_joint],
                _jnt_position_state[_worst_cmd_state_joint], period.seconds());
        }
        if(returncode != 0){
            _servo_error_count++;
            if(_servo_error_count <= 3){
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                    "ServoJ error=%d, attempting recovery (%d)... max_abs(target-state)=%.6f rad "
                    "post_drag_hold=%d post_estop_hold=%d held_target_selected=%d write_period=%.6f s cmdT=0.020000 s",
                    returncode, _servo_error_count, _target_state_max, (int)_post_drag_hold,
                    (int)_post_estop_hold, (int)(_tgt == _hold_target), period.seconds());
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
