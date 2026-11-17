# FR5 서보 무진행 진단 (2026-09-10)

## 2026-11-16 지연·연속 송신·SI 전후 기록 (0.0.6)

0.0.6은 장애 전후 기록을 각각5초에서 **60초**로 늘린다. 기존 flat 메타데이터와
incident ID/시각 계약은 유지하며 `post_until_mono_ns`는 최초 트리거 뒤60초다.
호스트 수집기의 스레드·호스트·패킷별 coverage를 함께 확인해야 하며, native 파일이
완료됐다는 사실만으로 호스트 샘플이 모두 수집됐다고 판단하지 않는다.

13:57 장애의 제어기 캡처에는 목표 갱신 공백209ms와36ms 동안의 큐1→10 증가가
관절 피드백 소실·SI 보고보다 먼저 관측됐다. 지연의 발생 위치(PC scheduling,
SDK/network, 제어기)와 지연 뒤 slave 이탈의 물리 원인은 아직 확정되지 않았다.
SI·케이블·배터리 문제를 오류코드149의 유일한 원인으로 해석하지 않는다.

기존 제어 주기, SDK 호출·재시도 순서, hold/정지/복구 정책은 유지한다. 새 계측은
`read` 시작·종료, `write` 시작·종료(조기 반환 포함), 기존14개 제어 경로 SDK 호출
지점의 시작·종료를 기록한다. 성공한 호출도 기록하므로 반환코드0인 지연을 조사할
수 있다. activation과 deactivation의 SDK 연결·enable 호출 전체를 추적하는 기능은
아니며, 실제 SDK/네트워크 송수신 시각과 getter 완료 시각은 서로 다르다.

RT에서는 미리 할당한 SPSC 큐에 고정 크기 event를 복사한다. 새 SDK 조회, 문자열
생성, 파일 I/O, ROS 로그 출력, blocking mutex를 추가하지 않는다. 각 event에는
CLOCK_MONOTONIC/CLOCK_REALTIME 시각, Linux 제어 TID, read cycle/write 번호,
기대 제어 주기50Hz(20ms), 별도 ServoJ cmdT, ROS가 전달한 측정 주기와 실제 write 간격, 마지막 getter 관측 시각·frame age·유효성, 명령값,
마지막 ServoJ 시도 목표와 마지막 성공 목표, hold 플래그가 포함된다. 성공 목표
관측 여부는 duplicate suppression의 invalidation 상태와 별도로 유지한다.

마지막 성공한 `GetRobotRealTimeState`의 **설치된 SDK 패킷 전체**를 보존한다.
JSON에는 해당 버전의 모든 필드와 `raw_sdk_size`/`raw_sdk_hex`가 함께 들어간다.
관절 actual 위치·속도·가속도·토크, 제어기/말단/확장 I/O, SI/E-stop/collision,
enable, SDK의 controller clock도 포함된다. SDK getter가 실패하면 마지막 성공
관측값이므로 `sdk_observed_mono_ns`와 `state_valid`를 함께 본다. NaN/Inf는 JSON
숫자에서는 null로, 원본 bytes에서는 그대로 보존한다. 최신 웹 문서에만 있는 필드를
이전 SDK에 덧붙이거나0으로 만들어 기록하지 않는다. 배터리 전압·SI 전원전압은
이 버전의 패킷에 없으므로 관절 토크·그리퍼 전압으로 대체 해석하지 않는다.

전용 백그라운드 writer가 약60초 pre/60초 post를 유지한다. 트리거는50ms 이상인
제어 간격/SDK 호출(성공 포함), 지연 뒤 빠른 catch-up 주기, 기존 stale/invalid/
missing 상태·제어기/SI/SDK 오류의 변화, Action ABORTED다. 첫 트리거의 post
기한은 고정이며 오류가 반복돼도 계속 연장하지 않는다. 완료 뒤5초 이내의 abort
등은 같은 incident ID에 연계한다. 지속되는 raw fault 자체만으로 재개방하지
않지만, 이후 timing/RPC 지연은 새로 제한된 길이의 incident를 만들 수 있다.

정상 운전에는 새 반복 콘솔 출력이나 디스크 시계열을 쓰지 않는다. 메모리는
기본 event 큐8192개, history32768개, incident65536개로 제한한다. 이는50Hz에서
주기당8개 event의 전후60초를 보관하며, 실제 SDK packet을 포함한 event 크기에
따라 메모리가 증가한다. 더 높은 event 발생률·writer 지연에 의한 초과와 파일
실패는 queue/incident drop 및 I/O error 카운터에 남긴다. 디스크에는 hardware
trace를 최대4개 보존하며 이전 process/reboot의 trace도 회전 대상이다. host
recorder 파일은 지우지 않는다. 강제 종료 전의 디스크 동기화를 보장하지 않으며,
정상 shutdown은 기존 ServoMoveEnd/StopMotion/CloseRPC **후에** 잔여 기록을
partial로 저장하고 writer를 join한다.

호스트 연계(승인된 schema_version=1):

- 메타데이터: `/tmp/mf_fairino_flight_recorder/hardware-<pid>.json`.
- 실제 trace: `MF_FAIRINO_DIAGNOSTICS_DIR`가 지정되면 그 경로, 아니면
  `$HOME/.ros/fairino_diagnostics`의 hardware incident JSONL.
- 메타데이터 필드: `pid`, `/proc/self/stat` field22를 시작 때 한 번 읽은 `process_start_ticks`, 실제 제어 `control_tid`, `phase`, `incident_id`,
  `trigger_mono_ns`, `trigger_unix_ns`, `post_until_mono_ns`, `trace_path`.
- phase는 armed/capturing/complete/stopped다. 시작·identity·incident·phase 변경
  때만 원자 교체하며 정상 armed의 주기 갱신은 하지 않는다. 빠른 deactivate→activate로 armed가 되더라도 최근 incident ID/시각/경로는 다음 incident까지 유지한다. 최초 armed만 비어 있다.
- incident 시작에 `event=flight_recorder_incident`를 백그라운드에서 한 번만
  출력한다. 위 식별자/시각/경로와 I/O/drop 카운터를 포함한다.

호스트 recorder는 같은 monotonic clock으로 PC runnable 대기·sleep/network
blocked 상태와 TCP20003(XMLRPC)/20004(state)의 metadata를 대조한다. TCP8080만
관측해서 ServoJ 송신 지연을 판정하지 않는다. SDK 수신 buffer에서 복사한 시각을
실제 packet 수신 시각으로 기록하지 않는다. 하드웨어 캡처만으로 안전입력의 물리
원인이나 고속 가속/감속 시 공급전압을 확정하지 않는다.


## 2026-11-13 오류 발생 시 SDK 상태 로그 (0.0.4)

정상 운전에는 새 주기 로그를 출력하지 않는다. 기존 `/diagnostics`의 최대10Hz
상태 발행은 유지하고, 아래 경우에만 `FairinoHardwareInterface` ERROR 로그를 남긴다.

- `event=sdk_fault`: SDK 호출 실패, 제어 피드백 unavailable/invalid/stale,
  컨트롤러 주/부 오류코드, 충돌·비상·SI 안전정지, 모드 전환/flush 실패.
  같은 오류 조합이 지속되면 한 번만 출력한다. 오류 조합이 바뀌거나 정상 복귀 후
  다시 발생하면 새로 기록하며 정상 복귀 자체는 출력하지 않는다.
- `event=trajectory_aborted`: `joint_trajectory_controller/follow_joint_trajectory/_action/status`에서
  ABORTED goal을 관측했을 때 UUID당 한 번 최신 상태를 기록한다. SDK 오류코드가0인
  목표 미도달도 기록하기 위한 경로다. 성공·실행 중·정상 취소에는 출력하지 않는다.
  topic은 서비스 노드의 namespace를 따르므로 MF 팔에서는
  `/manipulator/joint_trajectory_controller/follow_joint_trajectory/_action/status`다. 상태 topic의 retained
  ABORTED를 처음 관측한 경우도 포함하므로 이 로그의 관측 시각을 최초 정지 시각으로
  간주하지 않는다. `goal_uuid`는 하위 FollowJointTrajectory goal UUID이며 상위 BT UUID와는
  다를 수 있다. `goal_accepted_ros_ns`와 함께 원래 JTC 결과/상태 기록과 대조한다.
- `event=control_snapshot_stale`: 기존200ms snapshot 지연 조건을 넘었을 때 한 번
  출력한다. 제어 샘플 갱신이 재개되기 전에는 반복하지 않는다.

로그에는 주/부 오류코드, robot/program/mode 상태, 충돌·비상·안전정지,
motion_done, 원본 frame, mc_queue_len, servoJCmdNum, 원본 관절각(deg)/속도(deg/s),
제어 명령/마지막 ServoJ 시도 목표/측정각(rad), hold/flush 상태, 실제 write 간격,
각 SDK 호출의 최근 반환코드·소요시간·횟수가 포함된다. motion_done=0, 정상 STOP,
manual/drag 상태, 큐 길이, 추종 오차 자체만으로는 새 오류 로그를 발생시키지 않는다.

한 번이라도 실패한 SDK 호출에는 `*_last_error_rc`, `*_last_error_duration_ms`,
`*_last_error_mono_ns`, `*_errors`도 포함한다. 같은 write에서 정지/복구 재시도가
성공해 최신 반환코드가0이 되어도 최초 실패의 코드와 시각을 잃지 않기 위함이다.
이 필드는 마지막 실패 이력이며 현재도 그 오류가 지속된다는 뜻은 아니다.

`snapshot_mono_ns`는 상태를 보존한 시각, `logged_mono_ns`는 출력 시각이며,
`snapshot_age_sec`는 두 시각의 차이다. `sdk_observed_mono_ns`는 마지막 성공한 SDK
getter의 로컬 완료 시각이며 실제 socket 수신 시각이 아니다. `sdk_sample_age_sec`는
출력 시점에서 그 getter 결과의 나이다. `frame_age_sec`는 snapshot을 보존한 시점의
마지막 유효 frame counter 변화 후 경과시간이다. getter 실패 시 원본 상태는
마지막 성공한 관측값이며, `state_valid=0` 및 timestamp/age와 함께 해석한다.
`sdk_state_observed=0`이면 컨트롤러 상태가 아직 없어0으로 초기화된 필드는 증거로
해석하지 않는다. SDK 상태는 실패한 ServoJ 호출 이전에 읽은 값일 수 있다.

RT read/write는 SDK 상태 필드를 복사하고 고정 크기16개 오류 snapshot 큐에 넣을 뿐,
문자열 생성·ROS 로그 출력·추가 SDK 조회를 하지 않는다. 기존100ms 백그라운드
callback이 출력하므로20ms짜리 일시 오류가 그전에 해제돼도 남는다.
타이머 이전에 하드웨어가 정상 비활성화돼도 SDK 정지/연결 종료 호출 뒤 별도
백그라운드 writer에서 남은 오류를 출력한다. 로그 sink가 느려도 기존 정지 호출을
미루지 않으며, 로그 완료까지 비활성화 응답은 기다린다. 프로세스 강제 종료/전원
단절이나 on_deactivate 없이 소멸하는 경로까지 내구성을 보장하는 저장소는 아니다.
오류 조합이 너무 빠르게 바뀌어 큐가 넘치면
`event=sdk_fault_log_overflow dropped_snapshots=...`로 누락을 표시한다.
목표/측정각의 전체 시계열이나 네트워크 패킷을 보존하는 기능은 아니며,
이 snapshot만으로 물리 정지나 과거 명령 재생을 확정하지 않는다.

가짜 SDK로 정상 무출력, 추가 SDK 호출 없음, 일시 오류 보존, 동일 오류 억제/재발,
실패한 getter의 마지막 상태 표시, ServoJ 실패 목표, SDK 정상 상태의 Action ABORTED,
안전 입력, 비유한 raw 피드백, 큐 overflow, snapshot 지연, Action namespace,
재시도 전 오류 이력, 정지/연결 종료 호출 이후 잔여 로그 출력을 검증한다.

## 2026-11-05 DI0 오인·드래그 전환 실패 수정

일반 플랜지 DI0은 기본적으로 손교시 버튼이 아니다. 자동 감지는 기본 OFF이며,
해당 입력에 active-LOW 버튼이 실제로 배선된 장치만 ros2_control 하드웨어 설정에
`<param name="flange_drag_button_enabled">true</param>`를 명시한다.
허용값은 `true`/`false`/`1`/`0`이다. 명시적인 `set_drag_teach` 서비스와
펜던트의 실제 drag 상태 감시는 이 설정과 무관하게 유지한다.

물리 안전정지 입력은 드래그 모드보다 먼저 처리한다. 정지 중에는 새 drag ON 요청을
버리고 OFF 요청만 보존한다. `ServoMoveEnd`, `DragTeachSwitch`, `ServoMoveStart`가
성공한 순서까지만 상태를 변경하며, 실패 뒤에는 기존 stop-only 경로로 정지·큐 비움·새 RT의
정지 확인을 시도한다. ServoJ는 차단하고 하드웨어 재기동까지
inhibit를 유지한다. 드래그 진입 실패를 성공한 active 상태로 기록하지 않는다.

`/diagnostics`의 `drag_active`, `drag_transition_failed`, `flange_drag_button_enabled`,
`raw_tool_di`, `controller_main_error`, `controller_sub_error`로 모드와 원인을 구분한다.
모드 전환 실패는 ERROR로 표시한다. 이번 변경은 물리 SI 안전정지 입력을 우회하지 않는다.
기존 activation은 ResetAllError·RobotEnable·ServoMoveStart를 호출하므로 재기동은
해당 입력과 실제 오류가 정상인지 확인하며 수행한다.

## 2026-11-05 유지 목표 중복 전송 제거

`write()`는 마지막으로 성공한 ServoJ와 여섯 관절 목표가 정확히 같으면 재전송하지 않는다.
FIFO 모델에서 다음 Home/복귀 궤적을 지연시킨 중복 유지점 전송 경로를 제거한다.
변경된 목표는 크기가 작아도 모두 전송하며, fresh RT·drag·안전정지 검사는 매 주기 유지한다.
50Hz 제어 주기, cmdT=0.02초와 상위 관절 허용 오차는 변경하지 않는다.

활성화·비활성화·Servo 모드 재시작·정지/flush·피드백/SDK 오류에는 마지막 전송 기록을
무효화한다. 이후 첫 목표는 이전과 같아도 다시 전송하고, 실패한 ServoJ를 성공한 목표로
기록하지 않는다. cancel/drag/e-stop의 기존 hold 해제 조건은 유지한다.

진단의 `rt_motion_queue_length`는 최신 유효 RT packet의 `mc_queue_len`이며,
`duplicate_servo_targets_skipped`는 생략한 동일 목표의 누적 횟수다. 기존 `queue_length`는
여전히 flush 중 마지막 직접 조회값이다. 세 값을 구분해서 해석한다.

로봇 없는 재현은 실제 `write()`와 50Hz FIFO 수신 모델을 연결한다. 기존 경로는 유지점
255개 뒤 Home 목표가 5초 추가 대기 뒤에도 0.02rad 남아 1도 허용 오차를 넘었다.
이 테스트는 SDK 수신 모델이며, 실제 컨트롤러의 지연 원인 전체와 실기 성공을 증명하지 않는다.
적용 후 큐·송신 횟수 및 Home/복귀의 실제 도달은 실기 재시험에서 확인한다.

## 2026-09-11 제어·정지 계약

현재 드라이버는 50Hz 제어 설정을 유지한다. 하드웨어 파라미터
`servo_command_period_sec`의 기본값은 `0.02`초이며, 유한한 `[0.02, 0.1]`초만
허용한다. 0.008초(125Hz), NaN/Inf, 숫자 뒤 문자열은 activation 이전에 거부한다.
이 파라미터는 ServoJ 보간 주기이며 controller_manager의 update_rate를 변경하지 않는다.

정상 제어 주기에는 `read()`가 `GetRobotRealTimeState`를 한 번 호출한다. 여섯
`jt_cur_pos`를 degree→rad로 변환하고, `write()`의 버튼·drag·safety 판정에도
같은 패킷을 사용한다. 새 read 없이 재호출한 write, 비유한 관절 값, 100ms 넘게
frame counter가 갱신되지 않은 피드백으로 ServoJ를 보내지 않는다.

`/fairino_hw_control/stop_and_flush` (`std_srvs/srv/Trigger`)는 다음 순서로 처리한다.

1. 서비스 callback은 atomic generation 요청을 등록하고 최대 2초 기다린다.
2. 다음 write 경계에서 `StopMotion → ServoMoveEnd → MotionQueueClear`를 실행한다.
3. 후속 write에서 `GetMotionQueueLength == 0`을 확인한다. 그 뒤 서로 다른 새 RT
   frame 3개에서 `robot_state == 1`이고 모든 `actual_qd`가 0.5deg/s 이하인지 확인한다.
4. drag 또는 safety stop이면 실패하고 송신을 차단한다. 정상 상태에서만
   `ServoMoveStart`를 호출하고, 재시작 뒤 새 RT frame에서도 정지·안전 상태를 확인한
   다음 성공을 응답한다. 완료한 주기에는 ServoJ를 보내지 않는다.

terminal 상태를 확인하지 못한 action timeout에는
`/fairino_hw_control/stop_and_hold`를 사용한다. 정지·큐 비움·연속 정지 확인은 같지만
`ServoMoveStart`를 호출하지 않고 `hard_inhibit`를 재기동까지 유지한다. 따라서 늦게
수락된 옛 goal이 actual 근처 명령을 보내도 물리 ServoJ가 재개되지 않는다.

모든 SDK 호출은 제어 스레드에 남는다. SDK 오류, 큐 비움 실패, timeout 이후에는
cancel hold와 ServoJ 송신 차단을 유지하며, 새로운 성공한 flush가 있어야 재개할 수
있다. Callback timeout은 SDK 실행 자체의 중단을 의미하지 않는다. 동기 SDK 호출
하나의 wall-time 상한은 보장하지 못한다. 응답은 generation과 결과를 포함한다.
결과 코드는 `0=없음, 1=처리 중, 2=성공, 3=SDK/큐 오류, 4=timeout, 5=안전/drag 상태`다.

성공한 resumable flush 뒤에도 cancel hold를 유지한다. 이후 명령이 flush 당시 명령과 달라지고,
**여섯 관절 모두 fresh actual과 0.005rad 이내에서 시작할 때만** cancel hold를
해제한다. 이 경로는 상위 제어기가 terminal을 확인한 실패에만 사용한다. goal 신원을
확인할 수 없는 경로는 위 stop-and-hold로 분리해 재개 자체를 막는다.
기존 drag/e-stop hold의 0.005rad 누적 명령 변화 해제 정책은 그대로 유지한다.

`/diagnostics`의 `fairino_hardware/control`을 백그라운드 100ms timer(최대 10Hz)로
발행한다. RT 쪽은 try_lock으로 POD만 복사하며 ROS 로그/문자열 직렬화를 하지 않는다.
cmdT, controller period/실측 write 간격, RT frame/age, ServoJ 호출 결과·시간,
각 제어 RPC 결과·시간·횟수, servoJCmdNum, 실제 송신 target-state 최대 오차,
hold, `hard_inhibit`, flush generation/phase/result를 포함한다. 200ms 넘게 제어 snapshot이 없으면
ERROR다. `queue_length`는 **flush 중 마지막 직접 조회값**이며 평시 큐 길이로
해석하지 않는다. servoJCmdNum 역시 큐 길이가 아니다.

아래 2026-09-10의 `/rosout` 로그 설명은 이전 바이너리의 관측 기록이다. 현재
주기 진단은 `/diagnostics`를 사용하므로 이를 rosbag에 포함해야 한다.

## 확인된 현상

실패 기록 `20260909_215555_0020_harvest_experiment`의 PBVS 네 번 모두 JTC desired
위치는 변하지만 raw joint feedback은 약 1초 동안 정지해 `no raw joint progress`로
중단된다. 이후 feedback은 이전 desired 궤적을 약 **1.71–1.75초 지연**해 따라간다.
이는 명령 전달과 측정 피드백을 합친 관측 지연이다. 손목 카메라도 비슷한 시각에 움직여
실제 동작 지연을 지지하지만, SDK와 컨트롤러 중 정확히 어디에서 지연되는지는 미확인이다.

분석 입력:

- `/home/mf_robot/workspace/rosbags/harvest/20260909_215555/20260909_215555_0020_harvest_experiment`
- `/rosout`, `/manipulator/joint_states`
- `/manipulator/joint_trajectory_controller/state`
- `/manipulator/joint_trajectory_controller/joint_trajectory`

| 시작 (KST) | 무진행 abort까지 | 목표 최대 변화 (rad) | raw 최대 변화 (rad) | 추정 지연 | 지연 보정 후 RMS (rad) |
|---|---:|---:|---:|---:|---:|
| 09-09 21:59:07.847 | 1.126 s | 0.017326 | 0.0000115 | 1.71 s | 0.000103 |
| 09-09 21:59:17.173 | 1.237 s | 0.013797 | 0.0000113 | 1.71 s | 0.000090 |
| 09-09 21:59:38.913 | 1.170 s | 0.010241 | 0.0000113 | 1.73 s | 0.000069 |
| 09-09 21:59:50.520 | 1.189 s | 0.009728 | 0.0000113 | 1.75 s | 0.000058 |

계산 방법:

- 시작은 `/rosout`의 `[visual_servo] admitted` 수신 시각이다.
- 목표/raw 변화는 시작 -0.1초부터 +1.4초까지 관절별 max-min 중 최댓값이다.
- 지연은 시작부터 +4초까지의 JTC actual과 `desired(t - lag)`를 비교해 여섯 관절
  전체 RMS가 최소인 lag를 선택했다. lag는 0–2.5초, 간격 0.01초이고 desired는
  수신 시각 기준 선형 보간했다. JTC의 `joint_names` 순서를 사용한다.
- raw joint_states의 순서는 JTC와 다르다. 다른 메시지 간 비교 시 이름으로 정렬해야 한다.
- 42개 서보 궤적은 position/velocity가 있는 단일점, `time_from_start=0.1 s`,
  header stamp=0이다. 대략 10Hz로 기록됐다.

실패와 종료 후 움직임의 시간적 순서만으로 "중단해야 움직인다"고 단정하지 않는다.
위 지연을 적용하면 실행 중의 목표 변화와 이후 feedback 형태가 가깝게 일치한다.
무진행 timeout만 늘려 해결됐다고 판정해서도 안 된다.

독립적인 영상 확인에서도 손목 카메라의 optical flow가 admission 후 각각
2.053/1.984/1.979/1.968초에 증가했다. 640×360, 최대 500개 특징점의 인접 프레임
LK flow 중앙값이 0.1px/frame을 넘은 시각이다. 첫 1.5초의 중앙값은 0.017–0.019,
1.9–2.3초에는 0.196–0.399px/frame이었다. 영상 수신-header 차이 중앙값은 30–39ms다.
영상 내 물체도 움직일 수 있으므로 보조 증거로 취급한다.

## 이전 세션 가설의 정정

- hold는 매 주기 이동량이 아니라 `max(abs(command - pre_stop_command)) < 0.005`를
  비교한다. 누적된 작은 변화는 임계값을 넘을 수 있다.
- 첫 세 시도의 desired 범위는 0.010rad보다도 크다. 동일한 고정 스냅샷을 기준으로
  전 구간이 ±0.005rad 안에 머문다는 설명과 맞지 않는다. 단, JTC desired는 HW가
  최종 선택해 SDK에 전달한 target 자체를 기록한 값은 아니다.
- 펜던트 drag hold와 e-stop hold는 모두 `1f0d9ba` (09-08)에 이미 포함됐다.
  이번에 인계된 미커밋 변경은 서비스/플랜지 버튼 경로, 해당 종료 시 hold, 진단 로그다.
- 같은 `1f0d9ba`가 ServoJ cmdT도 0.008→0.02로 바꿨다. 변경 시점만으로 hold를
  회귀 원인이라고 특정할 수 없으며, 큐/타이밍도 조사 대상이다.
- `open_loop_control: true`인 실제 FR5 드라이버 설정과 HW의 기존 closed-loop
  가정 주석은 불일치했다. 진단 패치에서 주석을 정정했다.
- `command_in_type: speed_units`이고 PBVS에는 별도 속도 제한이 있다.
  `scale.linear: 0.4 -> 0.2`만으로 PBVS 실제 속도가 반감됐다고 볼 근거는 없다.
- `b27e4c0` (09-09)의 속도 채움은 harvest executor의 action 궤적 경로다.
  PBVS 스트림 자체는 별도 경로이며 위 bag에서 이미 velocities를 보낸다.
- error 14를 오직 속도 불연속의 증거로 취급하지 않는다. 공식 3.9.2 문서도 일반적인
  인터페이스 실행 실패로 정의한다. 개별 발생 원인은 별도 확인한다.
  [공식 오류표](https://fairino-doc-en.readthedocs.io/3.9.2/SDKManual/errcode.html)

09-10 13:36에 시작한 실행은 다른 실패다. 14:04:50에 계획 노드 CUDA capture 오류,
14:04:59경 프로세스 종료, 14:05:40.946–42.306에 ServoJ error 14 반복이 기록됐다.
해당 실행의 HW 로그에는 drag/hold 진입 기록이 없었다. 이를 bag20의 원인과 합치지 않는다.

## 진단 패치

시작 로그의 `[servo-diagnostics] build=2026-09-10-v2`로 바이너리를 확인한다.

- `[servo-hold]`: 유지 자세를 ServoJ로 보내는 상태. 두 hold 플래그와
  command-pre_drag/pre_estop/hold/state 최대 편차를 기록한다 (2초 throttle).
- `[servo-hold-release]`: 어떤 hold가 임계값을 넘어 해제됐는지 기록한다.
- `[servo-tracking]`: command-state 또는 target-state 편차가 0.005rad 이상일 때
  SDK 반환 코드, 최종 target, 측정값, RT 상태, `frame_cnt`, `servoJCmdNum`,
  `lastServoTarget`, write 주기를 기록한다 (INFO, 2초 throttle).
- 오류 복구 로그에도 target-state, hold 상태, write 주기와 cmdT를 기록한다.

RT 데이터는 write 시작 시 이미 읽은 패킷이다. 같은 주기의 ServoJ 호출 **전** 스냅샷이므로
같은 줄의 target과 한 명령 차이가 날 수 있다. `servoJCmdNum`을 검증 없이 큐 길이라고
해석하지 않는다. `lastServoTarget`은 SDK 원시 값으로 표기하며 단위를 단정하지 않는다.
`rt_joint_position_deg`는 SDK 헤더에 선언된 degree, command/target/state는 rad이다.
worst joint index는 0부터 시작한다. RT 조회 실패 시 정수는 -1, 위치는 NaN으로 표시한다.
2초 throttle 로그만으로 약 1초짜리 실패 구간의 지연을 재구성할 수는 없다.
같이 기록한 desired/raw/camera와 대조해 어느 target과 RT 상태가 관측됐는지 판별한다.

기존 drag/e-stop hold 임계값과 ServoJ cmdT=0.02는 유지한다. drag/e-stop의 명령 변화
기반 해제에는 goal ID가 없다는 한계가 남지만, action terminal 미확인 경로는 별도의
stop-and-hold로 물리 재개를 차단한다.

## 검증과 실기 확인

로봇 없이 실제 `write()`를 fake SDK 심볼에 연결한 `test_write_hold`를 실행한다.
테스트는 실제 SDK 라이브러리를 링크하지 않고, activation/RobotEnable/RPC 호출은 실패한다.
버튼·서비스·펜던트·SI0·SI1 각각에서 정지 중 ServoJ 생략, 옛 목표 유지, 작은 고정 편차,
누적 작은 편차, 현재 자세 기준 새 목표, stale 궤적 변화의 기존 동작을 확인한다.

2026-09-11 검증: 패키지 빌드 및 48개 gtest 통과. CMake lint, cppcheck, XML 검사도
통과했다. 패키지 전체 uncrustify는 기존 vendor 파일을 포함한 스타일 차이로 실패하며,
실기 검증을 통과했다는 의미는 아니다.

workspace root에서 격리 빌드한다:

```bash
source /opt/ros/humble/setup.bash
source install/fairino_msgs/share/fairino_msgs/local_setup.bash
colcon --log-base log/fairino-servo-diagnostic build \
  --base-paths src/frcobot_ros2/fairino_hardware \
  --packages-select fairino_hardware \
  --build-base build/fairino-servo-diagnostic \
  --install-base install/fairino-servo-diagnostic
colcon --log-base log/fairino-servo-diagnostic test \
  --base-paths src/frcobot_ros2/fairino_hardware \
  --packages-select fairino_hardware \
  --build-base build/fairino-servo-diagnostic \
  --install-base install/fairino-servo-diagnostic \
  --ctest-args -R '^test_write_hold$' --output-on-failure
```

실기 검증은 현장 작업자와 재기동/움직임 범위를 확인한 후 진행한다. 새 바이너리를
로드한 실행에서 startup marker를 확인하고, 동일 관절의 command → HW target →
RT lastServoTarget → RT position → joint_states 시간을 대조한다. 손교시 없이 시작하는
시험과 손교시 종료 후 시험을 구분해, 지연과 hold의 영향을 분리한다. `/rosout`,
controller_state, raw joint_states, 입력 joint_trajectory를 함께 기록한다.

현재 결과는 오프라인 진단 및 검증이며 실기 서보 복구 완료를 의미하지 않는다.
