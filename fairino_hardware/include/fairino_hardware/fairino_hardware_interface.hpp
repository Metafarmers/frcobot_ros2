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
#include "fairino_hardware/flight_recorder.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <mf_manipulator_msgs/msg/servo_stream_state.hpp>
#include <mf_manipulator_msgs/srv/control_servo_stream.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <array>
#include <chrono>
#include <mutex>
#include <memory>
#include <atomic>
#include <thread>


#define CONTROLLER_IP_ADDRESS "192.168.50.101"

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  friend class FairinoHardwareInterfaceTest;
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  ~FairinoHardwareInterface() override;

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
  double servo_command_period_sec_ = 0.02;  // 50 Hz contract; never infer 125 Hz.
  std::array<double, 6> last_servo_target_{};
  bool last_servo_target_valid_ = false;
  ROBOT_STATE_PKG rt_snapshot_{};  // read/write control-thread owned, not shared with callbacks
  // Observation only: preserve the full installed SDK packet, including fields
  // rejected by control validation. No SDK ABI or control policy changes.
  ROBOT_STATE_PKG trace_raw_sdk_{};
  std::unique_ptr<FlightRecorder> flight_recorder_ = std::make_unique<FlightRecorder>();
  uint64_t trace_cycle_index_ = 0;
  bool trace_successful_target_observed_ = false;
  double trace_reported_period_sec_ = 0.0;
  void startFlightRecorder();
  void recordTrace(TraceKind kind, TraceRpc rpc = TraceRpc::NONE, int rc = 0,
                   int64_t rpc_begin_mono_ns = 0, uint64_t reasons = TRIGGER_NONE);
  bool rt_valid_ = false;
  bool rt_available_for_write_ = false;
  bool rt_frame_seen_ = false;
  std::chrono::steady_clock::time_point rt_frame_advanced_at_{};
  std::chrono::steady_clock::time_point last_write_at_{};
  static constexpr double kFreshStateSec = 0.1;
  static constexpr double kCancelReleaseToleranceRad = 0.005;
  static constexpr double kStoppedVelocityDegSec = 0.5;
  static constexpr unsigned kStoppedFramesRequired = 3;
  static constexpr double kFlushTimeoutSec = 2.0;
  enum class FlushResult : int { NONE, PENDING, SUCCESS, RPC_ERROR, TIMEOUT, UNSAFE_STATE };
  enum class FlushPhase : int { IDLE, WAIT_QUEUE, WAIT_STATIONARY, WAIT_RESTART_FRESH };
  std::atomic<uint64_t> flush_requested_{0};
  std::atomic<uint64_t> flush_completed_{0};
  std::atomic<int> flush_result_{static_cast<int>(FlushResult::NONE)};
  std::atomic<int64_t> flush_deadline_ns_{0};
  std::atomic<bool> flush_callback_busy_{false};
  std::atomic<bool> flush_resume_requested_{true};
  std::atomic<bool> hard_inhibit_{false};
  uint64_t flush_active_generation_ = 0;
  FlushPhase flush_phase_ = FlushPhase::IDLE;
  uint8_t flush_queue_empty_frame_ = 0;
  uint8_t flush_stationary_frame_ = 0;
  uint8_t flush_restart_frame_ = 0;
  unsigned flush_stationary_frames_ = 0;
  bool flush_resume_active_ = true;
  bool cancel_hold_ = false;
  bool flush_inhibit_ = false;
  double cancel_command_at_flush_[6]{};
  enum RpcIndex : size_t { STATE_RPC, SERVO_RPC, STOP_RPC, END_RPC, CLEAR_RPC,
                          QUEUE_RPC, START_RPC, DRAG_RPC, RESET_RPC, RPC_COUNT };
  void recordRpcBegin(RpcIndex index, std::chrono::steady_clock::time_point start);
  struct RpcSample {
    int rc = 0;
    double duration_ms = 0.0;
    uint64_t calls = 0;
    int last_error_rc = 0;
    double last_error_duration_ms = 0.0;
    int64_t last_error_at_ns = 0;
    uint64_t errors = 0;
  };
  struct SdkStateSnapshot {
    std::array<double, 6> joint_position_deg{};
    std::array<double, 6> joint_velocity_deg_sec{};
    int64_t observed_at_ns = 0;  // Local SDK getter completion, not socket receive time.
    int frame = 0;
    int main_code = 0;
    int sub_code = 0;
    int program_state = 0;
    int robot_state = 0;
    int robot_mode = 0;
    int emergency_stop = 0;
    int collision = 0;
    int safety0 = 0;
    int safety1 = 0;
    int motion_done = 0;
    int motion_queue_length = 0;
    int servo_command_count = 0;
    bool observed = false;
  };
  struct DiagnosticSnapshot {
    std::array<RpcSample, RPC_COUNT> rpc{};
    SdkStateSnapshot sdk;
    std::array<double, 6> command_rad{};
    std::array<double, 6> measured_rad{};
    std::array<double, 6> last_servo_target_rad{};
    bool servo_target_observed = false;
    double frame_age_sec = -1.0;
    double command_period_sec = 0.02;
    double loop_period_sec = 0.0;
    double actual_loop_period_sec = 0.0;
    double state_age_sec = 0.0;
    double target_state_error_rad = 0.0;
    int64_t sampled_at_ns = 0;
    uint64_t write_cycles = 0;
    uint64_t flush_generation = 0;
    uint64_t flush_completed_generation = 0;
    int flush_result = 0;
    int flush_phase = 0;
    int queue_length = -1;
    int rt_motion_queue_length = -1;
    uint64_t duplicate_servo_targets_skipped = 0;
    int servo_command_count = 0;
    int robot_state = 0;
    int safety0 = 0;
    int safety1 = 0;
    int frame = 0;
    bool state_valid = false;
    bool cancel_hold = false;
    bool flush_inhibit = false;
    bool hard_inhibit = false;
    bool drag_hold = false;
    bool estop_hold = false;
    bool drag_active = false;
    bool drag_transition_failed = false;
    bool flange_drag_button_enabled = false;
    int raw_tool_di = 0;
    int main_error = 0;
    int sub_error = 0;
  } control_diagnostics_, published_diagnostics_;
  std::mutex diagnostics_mutex_;  // RT uses try_lock; serialization happens only in timer.
  // Single producer: read/write control thread. Single consumer: the service
  // node's mutually-exclusive default callback group. No allocation or logging
  // in the producer, and short-lived errors survive the 100ms timer interval.
  static constexpr size_t kFaultSnapshotCapacity = 16;
  using FaultKey = std::array<int, RPC_COUNT + 9>;
  std::array<DiagnosticSnapshot, kFaultSnapshotCapacity> fault_snapshots_{};
  std::atomic<uint64_t> fault_write_index_{0};
  std::atomic<uint64_t> fault_read_index_{0};
  std::atomic<uint64_t> fault_snapshots_dropped_{0};
  FaultKey previous_fault_key_{};
  bool previous_fault_active_ = false;  // Control thread only.
  bool snapshot_stale_logged_ = false;  // Background callbacks only.
  std::vector<std::array<uint8_t, 16>> reported_aborted_goals_;
  void recordRpc(RpcIndex index, int rc, std::chrono::steady_clock::time_point start);
  void commitDiagnostics();
  void createDiagnosticInterfaces();
  void publishDiagnostics();
  void retainFaultSnapshot();
  void logPendingFaults(const DiagnosticSnapshot& latest, double snapshot_age_sec);
  void logFaultSnapshot(const char* event, const DiagnosticSnapshot& snapshot,
                        const std::string& goal_id = {}, int64_t goal_accepted_ros_ns = 0);
  void logTrajectoryAborts(const action_msgs::msg::GoalStatusArray& statuses);
  void requestStopAndFlush(std_srvs::srv::Trigger::Response& response, bool resume_after_stop);
  bool processFlush();  // true: this write boundary must send no ServoJ
  void finishFlush(FlushResult result);
  // The write thread is the sole owner of admission and SDK mutation. The ROS
  // thread only exchanges bounded POD requests/snapshots through a try-lock.
  using StreamState = mf_manipulator_msgs::msg::ServoStreamState;
  using StreamControl = mf_manipulator_msgs::srv::ControlServoStream;
  struct StreamRequest {
    uint8_t command = 0;
    std::array<char, 65> owner{};
    uint64_t binding = 0, generation = 0, id = 0;
    std::array<double, 6> hold{};
    int64_t deadline_ns = 0;
  } stream_pending_, stream_last_request_;
  struct StreamSnapshot {
    std::array<char, 65> owner{};
    uint64_t binding = 0, generation = 1, completed_id = 0, revision = 0;
    uint8_t phase = StreamState::IDLE, reason = StreamState::NONE, recoveries = 0;
    int code = 0;
    bool feedback_valid = false;
    double age = -1, last_delay = 0;
    std::array<double, 6> measured{}, command{};
    int64_t sampled_ns = 0;
  } stream_control_, stream_published_;
  uint8_t stream_logged_phase_ = 255;
  uint64_t stream_logged_generation_ = 0;
  bool stream_guard_enabled_ = false;  // on_init enables the deployed guard.
  double stream_delay_threshold_sec_ = 0.06;
  std::string stream_session_;
  std::mutex stream_mutex_;
  bool stream_mailbox_pending_ = false;
  std::atomic<bool> stream_latched_{false};
  std::atomic<uint64_t> stream_abort_binding_{0};
  std::atomic<int64_t> stream_rpc_begin_ns_{0};
  std::atomic<uint64_t> stream_lease_binding_{0};
  std::atomic<int64_t> stream_lease_seen_ns_{0};
  bool stream_guard_flush_ = false;
  int64_t stream_stationary_since_ns_ = 0;
  std::array<double, 6> stream_stationary_min_{}, stream_stationary_max_{};
  uint8_t stream_rearm_frame_ = 0;
  unsigned stream_rearm_step_ = 0;
  StreamRequest stream_rearm_request_;
  void createStreamInterfaces();
  void handleStreamControl(const StreamControl::Request&, StreamControl::Response&);
  StreamState streamMessage(const StreamSnapshot&) const;
  void publishStreamState();
  void commitStreamState();
  void consumeStreamRequest();
  void latchStream(uint8_t reason, double delay = 0.0);
  bool processStreamRearm();
  bool streamHealthy() const;
  bool streamAtHold(const std::array<double, 6>& hold) const;
  void finishStreamRequest(const StreamRequest&, int code);
  rclcpp::Service<StreamControl>::SharedPtr stream_srv_;
  rclcpp::Publisher<StreamState>::SharedPtr stream_pub_;
  rclcpp::TimerBase::SharedPtr stream_timer_;
  void startServiceThread();
  void stopServiceThread();
  void drainFaultLogsOnDeactivation();
  // ★손교시 SW 언락(전원재시작 불필요): ~/set_drag_teach(SetBool) → DragTeachSwitch 토글.
  //   ★서비스 spin 은 **별도 백그라운드 스레드**(_svc_exec)에서 돈다 — write()(50Hz 실시간)에서
  //   spin_some 을 부르면 매 사이클 executor 를 생성·파괴해 ServoJ 타이밍에 지터를 준다(뚝뚝 끊김·
  //   심하면 컨트롤러 fault). 콜백은 원자변수 _drag_req 만 세팅하고, write() 는 그걸 읽어 SDK 를
  //   호출한다(SDK 는 write 스레드 단독 → 스레드안전). 드래그 ON → robot_state==4 → 기존 공존이 ServoJ 스킵.
  std::shared_ptr<rclcpp::Node> _svc_node;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _drag_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr flush_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_hold_srv_;
  rclcpp::CallbackGroup::SharedPtr flush_callback_group_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr trajectory_status_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> _svc_exec;
  std::thread _svc_spin_thread;                                          // _svc_exec->spin() 스레드
  std::atomic<int> _drag_req{0};   // 1=드래그ON 요청, -1=OFF, 0=없음(콜백=백그라운드 세팅, write 가 소비)
  bool _drag_active = false;  // 드래그모드 진행중(ServoMoveEnd+DragTeachSwitch) — ServoJ 전면 스킵
  bool flange_drag_button_enabled_ = false;
  bool drag_transition_failed_ = false;
  // Opt-in only: 일반 DI0을 버튼으로 단정하지 않는다. 실제 active-LOW 버튼 배선 장치에서만 사용한다.
  // ★플랜지 드래그 버튼 hold-to-drag(사용자 2026-09-09): tl_dgt_input_l bit0(active-LOW: 눌림=0)이
  //   눌린 동안 드래그, 떼면 해제. write() 상단에서 엣지 감지해 _drag_req 세팅(서비스와 공존).
  bool _drag_btn_prev = false;  // 직전 사이클 버튼 눌림 여부(엣지 검출)
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
