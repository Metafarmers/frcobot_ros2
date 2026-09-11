# FR5 서보 무진행 진단 (2026-09-10)

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
