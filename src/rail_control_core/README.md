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
# 기본 (모드 B = 선점형)
ros2 launch rail_control_core control_core.launch.py

# 비교군 (모드 A = FCFS)
ros2 launch rail_control_core control_core.launch.py mode:=A

# 2배속 시연
ros2 launch rail_control_core control_core.launch.py speed_scale:=2.0
```

### 확인

```bash
ros2 topic list
ros2 topic echo /order_event
ros2 topic echo /control_state --once | head -40
ros2 topic hz /control_state          # 10Hz 나와야 정상
```

### 시연 조작

```bash
ros2 service call /code_red    std_srvs/srv/Trigger   # 🔴 Code Red 수동 발령
ros2 service call /toggle_mode std_srvs/srv/Trigger   # 모드 A ↔ B
ros2 service call /reset       std_srvs/srv/Trigger   # 리셋
```

---

## 테스트

```bash
cd ~/cobot3_ws/src/rail_control_core
python3 -m pytest test/ -v
```

**로직을 고칠 때마다 반드시 돌리세요.** 0.2초면 끝납니다.

검사 항목 (16개):
- 회귀 기준값 (모드 A 53.9s / 모드 B 44.7s, P0 완료 시각)
- 4가지 핵심 시연 장면 재현 여부
- 블록 중복 점유 없음 (충돌 방지)
- 전 오더 완료 (교착 방지)
- 종료 후 자원 누수 없음
- 오염/청결 동선 분리
- 토폴로지 형상 (10노드 8블록)

---

## 구조

```
rail_control_core/
├── config/
│   ├── topology.yaml         ← 10노드·8블록 (팀 계약. 함부로 바꾸지 말 것)
│   ├── params.yaml           ← 모드, 임계값, 기준값
│   └── scenario_main.yaml    ← 시나리오 오더 O-1~O-4
├── rail_control_core/
│   ├── topology.py           ← 그래프 로딩 + BFS 경로탐색 + 금지 간선
│   ├── fsm.py                ← 3-FSM 상태 정의 + 데이터 모델
│   ├── engine.py             ← 관제 두뇌 (ROS 무의존) ★핵심★
│   └── control_core_node.py  ← rclpy 래퍼 (판단 로직 없음)
├── launch/control_core.launch.py
├── test/test_regression.py
└── tools/calibrate.py        ← 기준값 맞추기용 파라미터 탐색
```

**설계 원칙: 판단은 `engine.py`에, ROS는 `control_core_node.py`에.**
이 경계를 지키면 테스트가 빨라지고 B·C를 기다리지 않고 개발할 수 있습니다.

---

## 파라미터 캘리브레이션

기준값을 바꾸고 싶을 때:

```bash
python3 tools/calibrate.py
```

구간 통과시간을 탐색해 목표 KPI에 맞춥니다.
단, **4가지 시연 장면이 모두 재현되는지** 반드시 함께 확인하세요.
숫자만 맞추다 보면 대피 장면이 사라지는 일이 실제로 발생합니다.

---

## 자주 겪는 오류

| 증상 | 원인 | 해결 |
|---|---|---|
| `Package 'rail_control_core' not found` | 소싱 안 함 | `source ~/cobot3_ws/install/setup.bash` |
| `config` 파일을 못 찾음 | `setup.py`의 `data_files` 누락 | yaml 추가 후 `colcon build` 재실행 |
| 다른 PC에서 토픽이 안 보임 | 도메인/화이트리스트 불일치 | `ROS_DOMAIN_ID=50`, 화이트리스트에 본인 IP 포함 확인 |
| 자기 노드끼리도 통신 안 됨 | 화이트리스트에 `127.0.0.1` 누락 | XML에 루프백 주소 추가 |
| `colcon build` 후 `src/build` 생성 | `src/` 안에서 빌드함 | `~/cobot3_ws`에서 실행 |
| 코드 고쳤는데 반영 안 됨 | `--symlink-install` 없이 빌드 | 재빌드하거나 옵션 추가 |
