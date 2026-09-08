#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <vector>
#include "libfairino/include/robot.h"


#define CONTROLLER_IP_ADDRESS "192.168.50.101"

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  int _control_mode;
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;
  std::unique_ptr<FRRobot> _ptr_robot;
  int _servo_error_count = 0;
  // 손 티칭(펜던트 DRAG) 공존 상태 — write() 에서 사용.
  // 드래그 중(robot_state==4)엔 ServoJ/복구를 스킵하고, 종료 후엔 새 goal 전까지 옮긴 위치를 유지해
  // JTC 의 옛 홀드 setpoint 로의 스프링백을 막는다.
  bool _prev_drag = false;        // 직전 사이클 드래그 여부(엣지 검출)
  bool _post_drag_hold = false;   // 드래그 종료 후 새 goal 전까지 현재자세 유지 모드
  double _pre_drag_cmd[6] = {0};  // 드래그 진입 시 JTC 홀드 setpoint 스냅샷(스프링백 목표)
  double _hold_target[6] = {0};   // 드래그로 옮긴 현재 위치(유지 목표) — e-stop hold 와 공유
  // 안전정지(e-stop = ServoJ error=99) anti-springback — write() 에서 사용.
  // e-stop 이 걸리면 정지 위치를 _hold_target 에 래치해 ServoJ 목표로 삼아, 릴리스해도 JTC 의
  // 앞서간 setpoint 로 급발진("퐉")하지 않게 한다. 새 goal 이 오면(command 이동) 유지 해제.
  bool _post_estop_hold = false;  // e-stop 해제 후 새 goal 전까지 정지 위치(B) 유지 모드
  bool _prev_estop = false;       // 직전 사이클 e-stop 여부(진입 엣지 → StopMotion 1회)
  double _pre_estop_cmd[6] = {0}; // e-stop 중 추적한 JTC command(새 goal 판정 기준)
};

} //end namespace


#endif