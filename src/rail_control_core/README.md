# rail_control_core

**의료 레일 관제 디지털 트윈 — A. 관제 코어**
3-FSM · P-EDD 계단식 선점 배차 엔진

---

## 30초 안에 확인하기 (ROS 불필요)

```bash
cd ~/cobot3_ws/src/rail_control_core
pip3 install pyyaml pytest --break-system-packages   # 최초 1회
python3 -m rail_control_core.engine
```

모드 A / 모드 B를 각각 돌려 선점 이벤트와 KPI를 출력합니다.
**ROS2를 띄우지 않아도 됩니다.** 엔진이 순수 파이썬이기 때문입니다.

---

## 설치 및 빌드

```bash
# 1. 워크스페이스에 배치
mkdir -p ~/cobot3_ws/src
cd ~/cobot3_ws/src
# (rail_control_core 폴더를 여기에 둡니다)

# 2. 의존성 확인
cd ~/cobot3_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. 빌드  ※ 반드시 워크스페이스 최상위에서!
colcon build --symlink-install --packages-select rail_control_core
source install/setup.bash
```

> `--symlink-install`을 붙이면 파이썬 파일을 고쳤을 때 재빌드 없이 반영됩니다.
> 개발 중에는 꼭 붙이세요. (단, `config/*.yaml`을 **추가**했을 때는 재빌드 필요)

---

## 실행

```bash
# 기본 (모드 B = 선점형, 30Hz, 자동 시작)
ros2 launch rail_control_core control_core.launch.py

# 비교군 (모드 A = FCFS)
ros2 launch rail_control_core control_core.launch.py mode:=A

# 수동 시작 (시연 때 타이밍을 잡고 싶을 때)
ros2 launch rail_control_core control_core.launch.py autostart:=false

# 5배속 (타이머 1주기당 엔진 5틱)
ros2 launch rail_control_core control_core.launch.py speed_scale:=5.0
```

### 검증 체크리스트 (각각 새 터미널, `source install/setup.bash` 후)

```bash
ros2 topic hz /capsule_pose          # 기대: ~30 Hz
ros2 topic echo /order_event         # 기대: ORDER_RELEASE → ... → SIM_DONE
ros2 topic echo /block_state --once --qos-durability transient_local --qos-reliability reliable
                                     # 기대: 블록 30개 스냅샷 (늦게 켜도 마지막 상태 즉시 수신)
```

모드 B 완주 시 `[SIM_DONE] sim makespan=85.77` 로그가 나오면
관제 코어–ROS 경계까지 전부 정상입니다.

### 시연 조작 (전부 `std_srvs/srv/Trigger`)

```bash
ros2 service call /sim_start    std_srvs/srv/Trigger   # 시작
ros2 service call /sim_pause    std_srvs/srv/Trigger   # 일시정지
ros2 service call /sim_reset    std_srvs/srv/Trigger   # 초기화 (t=0)
ros2 service call /sim_status   std_srvs/srv/Trigger   # 현재 t·mode·오더 상태
ros2 service call /sim_mode_a   std_srvs/srv/Trigger   # 모드 A 로 초기화
ros2 service call /sim_mode_b   std_srvs/srv/Trigger   # 모드 B 로 초기화

ros2 service call /code_crimson std_srvs/srv/Trigger   # 🔴 예약 P0 를 지금 발령
ros2 service call /code_red     std_srvs/srv/Trigger   # 🔴 디포 유휴 캡슐로 추가 P0
```

`/code_crimson` 은 시나리오의 예약 발령(8.0s)을 앞당기는 버튼이라 **이미 발령된 뒤에는
사유와 함께 거부**합니다. 그 시점 이후에 P0 를 더 넣고 싶으면 `/code_red` 를 쓰세요.

### QoS (수신측 필독 — B·C·D 공유)

| 토픽 | QoS | 이유 |
|---|---|---|
| `/capsule_pose` | BEST_EFFORT, depth 1 | 30Hz 위치 스트림 — 최신값만 의미, 유실 허용 |
| `/block_state` | RELIABLE + **TRANSIENT_LOCAL**, depth 1 | 변화 시에만 발행 — 늦게 켠 UI 도 마지막 상태 수신 |
| `/order_event` | RELIABLE, depth 50 | 오더·선점·대피 전 이벤트, 유실 불가 |
| `/control_state` | BEST_EFFORT, depth 1 | 오더 대시보드 집계 (D) |
| `/kpi` | RELIABLE + TRANSIENT_LOCAL, depth 1 | 완주 시 1회, 늦게 켜도 수신 |

> ⚠️ **BEST_EFFORT 토픽을 RELIABLE 구독자로 받으면 매칭 실패로 아무것도 안 옵니다.**
> `rqt` 나 커스텀 구독자에서 `/capsule_pose` 가 안 보이면 구독 QoS 부터 확인하세요.

**v3.2 변경 (2026-08-21 C 합의):** `/capsule_pose` 는 **코어가 발행**합니다(방향 반전).
위치의 진실 소스가 관제 코어이고, 씬 월드 좌표 `x/y/z` 까지 코어가 계산해 실어 보냅니다
— 브릿지는 `/World/Capsules/{capsule_id}` 트랜스폼에 그대로 꽂으면 됩니다.
같은 합의로 **`/capsule_cmd` 는 폐기**되었습니다 (FSM 상태 → pose 의 `state`,
선점 연출 → `/order_event` 의 `event`). 이행 대응표는 [`docs/C연동_매핑표.md`](../../docs/C연동_매핑표.md).

페이로드 스키마 전문은 [`src/rail_bridge/rail_bridge/interface_schema.json`](../rail_bridge/rail_bridge/interface_schema.json).

---

## 테스트

```bash
cd ~/cobot3_ws/src/rail_control_core
python3 -m pytest test/ -v
```

**로직을 고칠 때마다 반드시 돌리세요.** 0.3초면 끝나고, ROS 를 띄울 필요가 없습니다.

검사 항목 (24개):

| 파일 | 개수 | 내용 |
|---|---|---|
| `test_regression.py` | 12 | §6-2 회귀 기준값(모드 A 71.20 / 모드 B P0 67.07), RTA 회복·등급 승격, R8 안전망 불변식, 블록 중복 점유 없음, 전 오더 완료(교착 방지), `/block_state` 페이로드 스키마 |
| `test_bridge.py` | 7 | 제어 명령(start/pause/reset/mode/status), 페이로드 스키마, 변화 시에만 발행, Code Crimson 즉시 발동·중복 거부, SIM_DONE 기준값(=`BASE_B`), 모드 전환 리셋, 좌표 반올림 불변식 |
| `test_edge_cases.py` | 5 | 오더 폭주 완주·안전망, **모드 A 그리드락 재현 + DEADLOCK 감지**, EVAC_SPUR 강제(역방향 배송 vs 콘보이), Code Crimson 발동 타이밍 3종, 폭주 결정론 |

통합 중 문제가 생기면 **이 테스트 통과 여부로 책임을 가릅니다.**
통과하면 관제 로직은 정상이고, 문제는 ROS 경계(빌드·QoS·네트워크)에 있습니다.

---

## 구조

```
rail_control_core/
├── config/
│   └── params.yaml           ← ROS 설정 + engine 블록(엔진 파라미터 오버라이드)
├── rail_control_core/
│   ├── topology.py           ← 블록·회랑·경로 테이블 (v3.1 FROZEN) ★팀 계약★
│   ├── fsm.py                ← 3-FSM 상태 정의 + 데이터 모델
│   ├── engine.py             ← 관제 두뇌 (ROS 무의존) ★핵심★
│   ├── scenario.py           ← 시연 시나리오 v2 발령표 (명세서 §6-1)
│   ├── geometry.py           ← 논리좌표(블록+pos) → 씬 월드 xyz 변환 (v3.2)
│   ├── bridge.py             ← 포트-어댑터 경계 (rclpy 무의존, 여기까지 테스트)
│   └── control_core_node.py  ← rclpy 어댑터 (dict→JSON 변환만)
├── launch/control_core.launch.py
├── test/test_regression.py   ← §6-2 회귀 기준값 고정
├── test/test_bridge.py       ← 노드 로직 (ROS 없이)
├── test/test_edge_cases.py   ← 폭주·그리드락·EVAC_SPUR 엣지케이스
└── tools/kpi_report.py       ← 모드 A/B 비교표 출력 (발표용)
```

**설계 원칙 (포트-어댑터): 판단은 `engine.py`·`bridge.py` 에, ROS 는 `control_core_node.py` 에.**

`bridge.py` 는 rclpy 를 임포트하지 않습니다. 노드가 30Hz 로 `bridge.step()` 을 부르면
발행할 dict 묶음을 돌려주고, 노드는 그걸 JSON 으로 바꿔 뿌리기만 합니다.
덕분에 ROS 없이 노드 로직을 테스트할 수 있고, 통합 디버깅 때 관제 책임과
ROS 경계 책임을 즉시 분리할 수 있습니다.

---

## 블록 길이·좌표를 바꿔야 할 때

블록 길이·용량과 씬 좌표는 `rail_control_core/topology.py` 에 있습니다
(v3.2 에서 B 담당자 실측표로 확정 — `[가정]` 값은 모두 해소되었습니다).
YAML 로 빼지 않은 이유는 팀 계약(ID 동결) 대상이라 코드 리뷰에 그대로 걸리게 하기
위해서입니다. 절차는 이렇습니다.

```bash
# 1) topology.py 의 BLOCKS 길이 / NODE_XY / BLOCK_PATHS 수정
# 2) 새 수치 측정
python3 tools/kpi_report.py

# 3) 기준값이 바뀌었으면 두 곳을 함께 갱신 (한쪽만 고치지 말 것)
#    - docs/topology_spec.md §6-2 표
#    - test/test_regression.py 의 BASE_A / BASE_B
python3 -m pytest test/ -q
```

`kpi_report.py` 는 기준값에서 벗어나면 ⚠️ 를 찍고 exit code 1 로 끝납니다.
`test_bridge.py` 는 기준값을 `test_regression.py` 에서 읽어오므로 따로 고칠 필요가 없습니다.

단, **4가지 시연 장면이 모두 재현되는지** 반드시 함께 확인하세요
(`kpi_report.py` 출력의 `발생 장면` 줄 — YIELD / EVAC_LANE / FINISH_ALLOWED).
숫자만 맞추다 보면 대피 장면이 사라지는 일이 실제로 발생합니다.

### 미결 3건 (B 담당자 회신 대기 — 진행 비차단)

| 항목 | 현재 처리 | 회신 시 수정 위치 |
|---|---|---|
| B2-01 길이 | BB-01 대칭 가정 4.53m | `topology.py` BLOCKS |
| 디포 도크 좌표 | 측면 가상 슬롯 10기 | `topology.py` DOCK_SLOTS |
| BB-01 곡선 웨이포인트 | L자 폴리라인을 4.53m 로 정규화 | `topology.py` BLOCK_PATHS |

전부 **코어만 고치면 되는 구조**입니다 — 브릿지·UI 변경은 필요 없습니다.

---

## 자주 겪는 오류

| 증상 | 원인 | 해결 |
|---|---|---|
| `Package 'rail_control_core' not found` | 소싱 안 함 | `source ~/cobot3_ws/install/setup.bash` |
| `params.yaml` 을 못 찾음 | `setup.py`의 `data_files` 누락 | yaml 추가 후 `colcon build` 재실행 |
| 다른 PC에서 토픽이 안 보임 | 도메인/화이트리스트 불일치 | `ROS_DOMAIN_ID=50`, 화이트리스트에 본인 IP 포함 확인 |
| 자기 노드끼리도 통신 안 됨 | 화이트리스트에 `127.0.0.1` 누락 | XML에 루프백 주소 추가 |
| `colcon build` 후 `src/build` 생성 | `src/` 안에서 빌드함 | `~/cobot3_ws`에서 실행 |
| 코드 고쳤는데 반영 안 됨 | `--symlink-install` 없이 빌드 | 재빌드하거나 옵션 추가 |
