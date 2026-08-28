# 의료 레일 관제 디지털 트윈

병원 지하 1층 · 지상 2층을 잇는 **전용 레일 물류망**을 Isaac Sim 디지털 트윈으로 구축하고,
그 위에서 **우선순위 선점(P-EDD) 관제 알고리즘**을 실행·검증하는 프로젝트입니다.

핵심 시연은 **Code Crimson(대량수혈 프로토콜)** 입니다. 선착순(FCFS) 배차에서 꼴찌로 밀렸던
응급 혈액 콘보이 4대를 선점 배차가 앞으로 끌어올리고, 그 대가로 지연된 화물은 시스템이
스스로 가속해 마감 내 회복시킵니다.

| 오더 | 모드 A (FCFS) | 모드 B (선점+RTA) | 해석 |
|---|---|---|---|
| **P0 혈액 콘보이 4대 → OR2** | 70.77 s | **67.07 s** | **−5.2 %**, 꼴찌 탈출 |
| P1 응급약품 | 37.60 s | 37.60 s | 동률 — 완주 허용으로 손해 0 |
| P2 항암제 | 48.97 s | 76.47 s | 양보 비용 → RTA 회복으로 마감 78 s에 1.53 s 여유 정시 |
| P3 멸균 공급 | 36.17 s | 36.17 s | 동률 — 콘보이와 동방향이라 무간섭 |
| P3 사용 기구 회수 | 74.97 s | 86.77 s | 대피 비용 |

> 수치 출처: [`docs/관제_기술문서.md`](docs/관제_기술문서.md) §5 · 회귀 기준값 `config/params.yaml` `baseline`.
> 30 Hz 결정론 시뮬레이션이며 ±0.2 s를 벗어나면 회귀 테스트가 실패합니다.

---

## 1. 시스템 설계

### 1-1. 4대 PC 구성

| 파트 | 담당 | 역할 | 산출물 |
|---|---|---|---|
| **A. 관제 코어** | 남현지 | 배차·경로·교통제어·긴급대응 판단, 위치 진실 소스 | `src/rail_control_core/` |
| **B. 씬 빌더** | 한석형 | 병원 USD 씬, 레일·캡슐·매니퓰레이터 자산, 카메라 | `isaacpjt/`, `isaacsim/` |
| **C. 연동** | 정희진 | ROS 2 ↔ Isaac Sim 브릿지, 통합 디버깅 | `src/rail_bridge/`, `isaacpjt/scripts/` |
| **D. UI·QA** | 김세은 | 관제 대시보드, 품질 검증 | `src/medical-rail-twin/` |

4대 모두 하나의 유선 스위치(`10.10.0.1~4`), 하나의 ROS 도메인(`136`), 하나의 FastDDS
화이트리스트를 공유합니다.

### 1-2. 계층 구조

```mermaid
flowchart TB
    subgraph D["④ UI · 대시보드 (D)"]
        UI["관제 대시보드<br/>오더 큐 · 지연 비용 게이지 · P2 속도 그래프"]
    end

    subgraph SIM["③ 시뮬레이션 (B · C)"]
        ISAAC["Isaac Sim — 병원 USD 씬<br/>2개층 · 레일 30블록 · 캡슐 10대 · OR 매니퓰레이터"]
        BR["브릿지<br/>JSON → /World/Capsules/Capsule_XX 트랜스폼"]
        BR --- ISAAC
    end

    subgraph ROS["② ROS 2 미들웨어 — Jazzy · DOMAIN 136 · Fast DDS"]
        T1["/capsule_pose (30 Hz, BEST_EFFORT)"]
        T2["/block_state (변화 시, TRANSIENT_LOCAL)"]
        T3["/order_event (RELIABLE, depth 50)"]
        SRV["std_srvs/Trigger 8종<br/>sim_start · pause · reset · status<br/>mode_a · mode_b · code_crimson · code_red"]
    end

    subgraph CORE["① 관제 코어 (A) — 포트·어댑터 3계층"]
        NODE["control_core_node.py — rclpy 어댑터<br/>(dict → JSON 변환만)"]
        BRIDGE["bridge.py — 제어·페이로드 (rclpy 무의존)"]
        ENG["engine.py — 관제 두뇌 (순수 Python)"]
        TOPO["topology.py — 블록·경로 테이블 (v3.1 FROZEN)"]
        GEO["geometry.py — 논리좌표 → 씬 월드 xyz"]
        NODE --> BRIDGE --> ENG
        ENG --- TOPO
        ENG --- GEO
    end

    CORE --> ROS
    ROS --> SIM
    ROS --> D

    style CORE fill:#fff4e6,stroke:#e8890c,stroke-width:3px
    style ROS fill:#eef6ff,stroke:#2b6cb0
    style SIM fill:#eefaf0,stroke:#2f855a
    style D fill:#faf0fb,stroke:#805ad5
```

**설계 원칙 — 판단은 코어에, ROS는 어댑터에.**
`bridge.py`는 rclpy를 임포트하지 않습니다. 노드가 30 Hz로 `bridge.step()`을 호출하면 발행할
dict 묶음을 돌려주고, 노드는 그것을 JSON으로 바꿔 뿌리기만 합니다. 덕분에 **ROS 없이 전 로직을
테스트**할 수 있고, 통합 장애가 나면 "테스트 통과 여부"로 관제 책임과 ROS 경계 책임을 즉시 가릅니다.

**위치의 진실 소스는 관제 코어입니다.** 캡슐의 블록 ID·진행거리·방향은 물론 **씬 월드 좌표 x/y/z까지
코어가 계산해서 발행**하고, 씬과 UI는 구독만 합니다. 브릿지는 받은 좌표를 프림 트랜스폼에 그대로 꽂습니다.

### 1-3. 알고리즘 5계층

| 계층 | 채택 | 탈락 대안과 이유 |
|---|---|---|
| ① 배차 | **P-EDD** (우선순위 → EDD) + 콘보이 슈퍼오더 | 단순 FIFO — 응급 역전 불가 |
| ② 경로 | 정적 경로 테이블 | WHCA\*/CBS — 트리형 토폴로지라 경로가 유일, 탐색할 것이 없음 |
| ③ 교통 제어 | **철도식 폐색** — 블록 상호배제 + 차두간격 + 방향 토큰 + 단방향 + 교행 루프(R13) | 은행원 알고리즘 — 더 복잡하면서 이 구조에선 이득 없음 |
| ④ 긴급 대응 | **ETA 이동 파도(bow-wave) 선점** + 3분기 스윕 + 롤링 해제 | 전 경로 일괄 잠금 — 원거리 하위 오더까지 과잉 지연 |
| ⑤ 속도 | **RTA 슬랙 회복**(R11) + 회복 불가 시 P2→P1 **자동 승격**(R12) | 단순 비례식 — 여유 0으로 수렴 / MPC — 30블록 MVP엔 과설계 |

### 1-4. 3-FSM

| FSM | 상태 |
|---|---|
| **오더** | CREATED → EN_ROUTE → ARRIVED → DONE |
| **캡슐** | DOCKED/QUEUED/STANDBY → MOVING ↔ YIELD_WAIT/EVACUATED/FINISHING → UNLOADING → (OR이면 SERVICING) → REMOVED |
| **블록** | FREE / RESERVED / OCCUPIED |

---

## 2. 플로우 차트

### 2-1. 오더 생애주기

```mermaid
flowchart TD
    A(["시나리오 발령표<br/>P2 1s · P1 2s · P3 2s · P0 8s · P3 17s"]) --> B["오더 생성<br/>due = 발령시각 + priority_due_sec"]
    B --> C{"배차 모드"}
    C -->|"A · FCFS"| D1["발령 순 정렬"]
    C -->|"B · PEDD"| D2["우선순위 → EDD 정렬"]
    D1 --> E
    D2 --> E["캡슐 배정<br/>P0는 대기열 4대 = 단일 콘보이 슈퍼오더"]
    E --> F["정적 경로 테이블 조회 (블록 체인)"]
    F --> G["MOVING — 30 Hz 주행"]
    G --> H{"다음 블록 진입 요청"}
    H -->|"허가"| G
    H -->|"거부 · 선점"| I["§2-2 진입 판정"]
    I --> G
    G --> J{"목적지 도착?"}
    J -->|"아니오"| G
    J -->|"예"| K["UNLOADING (2 s)"]
    K --> L{"목적지가 OR?"}
    L -->|"예"| M["SERVICING — 수술실 딥<br/>WORK 정지 → 관제 BLUE_CMD → 복귀"]
    L -->|"아니오"| N
    M --> N["REMOVED · 오더 DONE"]
    N --> O(["전 오더 완료 → SIM_DONE + KPI 발행"])

    style I fill:#ffe0e0,stroke:#c53030
    style O fill:#e0f7e0,stroke:#2f855a
```

### 2-2. 블록 진입 판정 — 안전망이 항상 먼저

```mermaid
flowchart TD
    REQ(["진입 요청"]) --> S1{"① 블록 용량 여유?<br/>(Code Crimson 중 콘보이는 상한 개방)"}
    S1 -->|"아니오"| W["대기 — 노드 0.4 m 앞 정지"]
    S1 -->|"예"| S2{"② 차두 간격 0.9 m 확보?"}
    S2 -->|"아니오"| W
    S2 -->|"예"| S3{"③ 방향 토큰 일치?<br/>단방향 구간 역주행 아님?"}
    S3 -->|"아니오"| S3B{"교행 루프 R13<br/>쌍둥이 레인 있음?"}
    S3B -->|"예"| MP["MEET_PASS — 대피 레인으로 치환 통과"]
    S3B -->|"아니오"| DL["대기 → 20 s 무진행 시 DEADLOCK 이벤트"]
    S3 -->|"예"| MODE{"배차 모드"}

    MODE -->|"A · FCFS"| OK(["진입 허가"])
    MODE -->|"B · PEDD"| R6{"R6 — 이미 발령된<br/>상위 오더만 판단 대상<br/>(예지 금지)"}
    R6 --> R5{"R5 — 상위 캡슐 ETA ≤ 15 s?<br/>(이동 파도 안인가)"}
    R5 -->|"아니오"| OK
    R5 -->|"예"| POS{"내 위치가 파도와 어떤 관계인가"}

    POS -->|"파도 밖"| R2["R2 양보 — 진입 보류 (YIELD)"]
    POS -->|"파도와 같은 방향"| R4["R4 완주 허용 — 세우지 않고 통과 (FINISH_ALLOWED)"]
    POS -->|"파도와 역방향"| R3["R3 대피<br/>① 루프 레인 치환 EVAC_LANE<br/>② 빈 지선 회피 EVAC_SPUR"]

    R4 --> OK
    R2 --> R7
    R3 --> R7["R7 롤링 해제 — 파도 통과 후 자동 재개 (RESUME)"]
    R7 --> RTA{"양보로 슬랙 < 10 s?"}
    RTA -->|"예"| R11["R11 RTA 회복 — 필요속도 역산·래치 (RTA_ENGAGED)<br/>불가 시 R12 등급 승격"]
    RTA -->|"아니오"| OK
    R11 --> OK

    style S1 fill:#eef6ff,stroke:#2b6cb0,stroke-width:2px
    style S2 fill:#eef6ff,stroke:#2b6cb0,stroke-width:2px
    style S3 fill:#eef6ff,stroke:#2b6cb0,stroke-width:2px
    style OK fill:#e0f7e0,stroke:#2f855a
    style R11 fill:#fff4e6,stroke:#e8890c
```

파란 박스 ①②③이 **R8 안전망**이며 **모드 A·B 양쪽에서 항상 먼저** 검사됩니다.
그래서 선착순 모드는 *느릴 뿐 위험하지 않습니다* — 충돌·교착 방지는 배차 정책과 무관합니다.

### 2-3. 데이터 흐름 (1틱 = 1/30 초)

```mermaid
sequenceDiagram
    participant E as engine.py<br/>관제 엔진
    participant B as bridge.py<br/>페이로드 조립
    participant N as control_core_node.py<br/>rclpy 어댑터
    participant R as ROS 2 / Fast DDS
    participant C as 브릿지 · C
    participant S as Isaac Sim 씬 · B
    participant U as 대시보드 · D

    E->>E: 배차 정렬 · 진입 판정 · 속도 산출
    E->>E: geometry.py — 블록+진행거리 → 씬 월드 xyz
    E->>B: 상태 스냅샷
    B->>N: 발행할 dict 묶음
    N->>R: /capsule_pose · /block_state · /order_event (JSON)
    R->>C: 구독
    C->>S: /World/Capsules/Capsule_XX 트랜스폼 적용
    R->>U: 구독 → 큐·게이지 갱신
    U->>R: ros2 service call (Trigger 8종)
    R->>N: 제어 명령
    N->>E: start / pause / reset / mode / code_crimson
```

---

## 3. 운영체제 · 실행 환경

`src/rail_control_core/tools/setup_env.sh` 에 고정된 **팀 확정값**입니다. 4대 전원이 동일해야
하며, 하나라도 다르면 **그 PC만 조용히 통신이 안 됩니다.**

| 항목 | 값 |
|---|---|
| OS | **Ubuntu 24.04 LTS (Noble)** |
| ROS 2 | **Jazzy Jalisco** |
| Python | **3.12** (Jazzy 기본) |
| RMW | **`rmw_fastrtps_cpp`** (eProsima Fast DDS) |
| `ROS_DOMAIN_ID` | **136** |
| FastDDS 프로파일 | `~/.ros/fastdds_whitelist.xml` — `127.0.0.1` + `10.10.0.1~4` 인터페이스 화이트리스트 |
| 유선 대역 | `10.10.0.0/24` (조원 번호 = 마지막 옥텟) |
| 방화벽 | `ufw` 비활성화 |
| 빌드 | `colcon` (ament_python) |
| 시뮬레이터 | NVIDIA **Isaac Sim** (`omni.usd` · `omni.kit.app` · `omni.replicator.core` · `pxr` API 사용) |
| Isaac Sim | 5.1.0 |
| NVIDIA GPU 드라이버 | 580.173.02 (기본 요구 사양: 580.65.06 이상 필수, 패키지명: nvidia-driver-580-open) |
| CUDA 버전 | 13.0 (nvidia-smi 표기 기준) |
| 커널 | Linux Kernel 6.14.0-27-generic |
| GPU | NVIDIA GeForce RTX 5080 (VRAM 16GB/ 16303 MiB) |

### 환경 설정 (최초 1회)

```bash
cd ~/cobot3_ws/src/rail_control_core/tools
chmod +x setup_env.sh
./setup_env.sh 3            # ← 본인 조원 번호(1~4). 10.10.0.3 을 쓴다는 뜻
./setup_env.sh 3 --check    # 아무것도 고치지 않고 진단만
./setup_env.sh 3 --fix      # .bashrc 의 낡은 충돌 설정을 자동 주석 처리
source ~/.bashrc
rosenv                      # 현재 설정값 확인
```

이 스크립트는 화이트리스트 XML 생성 → `.bashrc` 마커 블록 갱신 → 방화벽 해제 →
빌드 도구 설치 → **다른 조원 PC ping 진단**까지 한 번에 처리합니다. 여러 번 실행해도
중복되지 않습니다.

---

## 4. 사용 장비

### 4-1. 물리 장비

| # | 장비 | 역할 | 비고 |
|---|---|---|---|
| 1 | 일반 PC | **A — 관제 코어** 노드 실행 | GPU 불필요 (엔진이 순수 Python) |
| 2 | GPU PC | **B — 씬 빌드**, Isaac Sim 자산 제작 | |
| 3 | GPU PC | **C — Isaac Sim 런타임 + 브릿지**, 통합 디버깅 | |
| 4 | 일반 PC | **D — 관제 대시보드 · QA** | |
| — | 유선 스위치 + LAN 케이블 4본 | 4대 동일 서브넷 (`10.10.0.0/24`) | Wi-Fi 사용 금지 — 화이트리스트가 유선만 허용 |

> 각 PC의 GPU 모델·드라이버 버전은 저장소에 기록이 없습니다.

### 4-2. 씬 내 시뮬레이션 자산 (Isaac Sim)

| 자산 | 내용 |
|---|---|
| 병원 씬 | 2개 층(B1F z=4.0 / 2F z=13.0) + 수직 쉬프트, 노드 19개 · 블록 30개 · 총 연장 ≈ 85 m |
| 캡슐 | **10대** (`Capsule_01`~`Capsule_10`), W 25 × L 60 × H 50 cm, 점유 피치 0.9 m |
| 스테이션 | ST-INJ(주사조제실) · ST-PHM(약제부) · ST-CSR(중앙공급실) · ST-ICU · ST-OR1 · ST-OR2 + N-B1(혈액은행 대기열) |
| 디포 | 충전 도크 10기 (BB-07 존) |
| **매니퓰레이터** | **두산 M0609** 6축 협동로봇 (`/World/m0609`) + **OnRobot RG2-FT** 그리퍼 — OR 하역 시퀀스 담당 |
| 화물 | 수술팩 `SurgicalPack.usd` (0.3 × 0.09 × 0.25 m, 12 kg RigidBody) |
| 비전 | Replicator 합성 데이터셋 생성 (640×640, 35 mm 렌즈, 조명·포즈 랜덤화) |

> 매니퓰레이터·비전은 **Isaac Sim 씬 안의 시뮬레이션 자산**입니다. 이 저장소에는 실물
> 로봇 제어 코드가 포함되어 있지 않습니다 (USD `DriveAPI` 관절 타깃 보간 방식).

---

## 5. 의존성

### 5-1. 관제 코어 (`rail_control_core`) — ROS 없이도 동작

| 구분 | 패키지 |
|---|---|
| 실행 | `rclpy`, `std_msgs`, `std_srvs`, `python3-yaml` (PyYAML) |
| 테스트 | `pytest`, `ament_copyright`, `ament_flake8`, `ament_pep257` |
| 빌드 | `setuptools`, `colcon` (ament_python) |

**엔진 자체는 표준 라이브러리 + PyYAML만 씁니다.** NumPy·SciPy 같은 수치 라이브러리 의존이 없어
ROS를 띄우지 않고도 전 로직을 검증할 수 있습니다.

### 5-2. 브릿지 (`rail_bridge`)

`rclpy`, `std_msgs`, `geometry_msgs`

### 5-3. Isaac Sim 스크립트 (`isaacpjt/scripts/`)

`omni.usd`, `omni.kit.app`, `omni.timeline`, `omni.replicator.core`, `pxr`(USD), `rclpy`
— **Isaac Sim 내장 Python 환경에서 실행**되며, 별도 pip 설치가 필요 없습니다.

### 5-4. 설치

```bash
# ROS 의존성 일괄 설치
cd ~/cobot3_ws
rosdep install --from-paths src --ignore-src -r -y

# 또는 최소 설치 (엔진만 돌릴 때)
pip3 install pyyaml pytest --break-system-packages

# Python 의존성 일괄 설치 — OCR·라벨 생성·그리퍼 드라이버까지 포함
pip3 install -r requirements.txt --break-system-packages

# OCR 엔진 (pytesseract 가 호출하는 실행 파일)
sudo apt install tesseract-ocr tesseract-ocr-eng
```

pip 로 설치할 항목의 정본은 [`requirements.txt`](requirements.txt) 입니다.

>### 🖥️ 관제 대시보드 웹 UI (`src/medical-rail-twin`)

본 프로젝트의 관제 UI 대시보드는 독립 프론트엔드 레포지토리로 연동되어 있습니다.

#### 저장소 클론 및 서브모듈 초기화
```bash
# 서브모듈을 포함하여 프로젝트 전체 클론
git clone --recurse-submodules [https://github.com/Hanseokhyung/rokey_cobot3.git](https://github.com/Hanseokhyung/rokey_cobot3.git)

# 이미 클론한 경우 하위 모듈 동기화
git submodule update --init --recursive

---

#### 대시보드 실행
cd src/medical-rail-twin
npm install
npm run dev

---
```
#### 2. 저장소를 분리 운영(독립 실행) 형태로 안내하는 경우

서브모듈 등록 없이 별도 컴포넌트로 명시하고자 할 때는 '오류/주의' 문구 대신 **"마이크로서비스/독립 모듈 아키텍처"** 관점으로 포장하여 작성합니다.

## 🧩 서브 시스템 구성 및 설치 가이드

### 1. 관제 코어 및 시뮬레이션 브릿지 (Core & Sim)
ROS 2 Jazzy 기반 관제 엔진(`rail_control_core`)과 Isaac Sim 연동 브릿지(`rail_bridge`)를 빌드합니다.
```bash
cd ~/rokey_cobot3
colcon build --symlink-install
source install/setup.bash

## 관제 모니터링 대시보드 (Web UI)
: 실시간 오더 큐, 캡술 텔레메트리, RTA slack 현황을 모니터링하는 웹 대시보드입니다.

```bash
# 프론트엔드 클론 및 구동
git clone <대시보드_공식_레포_URL> src/medical-rail-twin
cd src/medical-rail-twin
npm install
npm run start

## 6. 사용법

### 6-0. 30초 안에 확인하기 (ROS 불필요)

```bash
cd ~/cobot3_ws/src/rail_control_core
pip3 install pyyaml pytest --break-system-packages   # 최초 1회
python3 -m rail_control_core.engine
```

모드 A / 모드 B를 각각 돌려 선점 이벤트와 KPI를 출력합니다.

### 6-1. 빌드

```bash
cd ~/cobot3_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install`을 붙이면 파이썬 파일을 고쳤을 때 재빌드 없이 반영됩니다.
단 `config/*.yaml`을 **추가**했을 때는 재빌드가 필요합니다.
⚠️ `src/` 안에서 빌드하지 마세요 — 반드시 워크스페이스 최상위(`~/cobot3_ws`)에서.

### 6-2. 관제 노드 실행

```bash
# 기본 — 모드 B(선점형), 30 Hz, 자동 시작
ros2 launch rail_control_core control_core.launch.py

# 비교군 — 모드 A(FCFS)
ros2 launch rail_control_core control_core.launch.py mode:=A

# 수동 시작 — 시연 때 타이밍을 잡고 싶을 때
ros2 launch rail_control_core control_core.launch.py autostart:=false

# 5배속
ros2 launch rail_control_core control_core.launch.py speed_scale:=5.0
```

### 6-3. 시연 조작 (전부 `std_srvs/srv/Trigger`)

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

`/code_crimson`은 시나리오 예약 발령(t=8.0 s)을 앞당기는 버튼이라 **이미 발령된 뒤에는 사유와
함께 거부**합니다. 그 이후에 P0를 더 넣으려면 `/code_red`를 쓰세요.

### 6-4. 동작 확인

```bash
ros2 topic hz   /capsule_pose         # 기대: ~30 Hz
ros2 topic echo /order_event          # 기대: ORDER_RELEASE → … → SIM_DONE
ros2 topic echo /block_state --once \
     --qos-durability transient_local --qos-reliability reliable
                                      # 기대: 블록 30개 스냅샷
```

모드 B 완주 시 `[SIM_DONE] sim makespan=86.77` 로그가 나오면 관제 코어–ROS 경계까지 정상입니다.

#### 토픽 QoS (수신측 필독)

| 토픽 | QoS | 이유 |
|---|---|---|
| `/capsule_pose` | BEST_EFFORT, depth 1 | 30 Hz 위치 스트림 — 최신값만 의미 |
| `/block_state` | RELIABLE + **TRANSIENT_LOCAL**, depth 1 | 변화 시에만 발행 — 늦게 켠 UI도 마지막 상태 수신 |
| `/order_event` | RELIABLE, depth 50 | 오더·선점·대피 이벤트, 유실 불가 |
| `/control_state` | BEST_EFFORT, depth 1 | 오더 대시보드 집계 |
| `/kpi` | RELIABLE + TRANSIENT_LOCAL, depth 1 | 완주 시 1회 |

> ⚠️ **BEST_EFFORT 토픽을 RELIABLE 구독자로 받으면 매칭 실패로 아무것도 오지 않습니다.**
> `/capsule_pose`가 안 보이면 구독 QoS부터 확인하세요.

### 6-5. 테스트 (로직 수정 시 필수)

```bash
cd ~/cobot3_ws/src/rail_control_core
python3 -m pytest test/ -v
```

**31종 전부 통과**해야 합니다. 2초면 끝나고 ROS를 띄울 필요가 없습니다.

| 파일 | 개수 | 내용 |
|---|---|---|
| `test_regression.py` | 13 | 회귀 기준값(A 74.97 / B P0 67.07), 5장면 발생, RTA 회복·등급 승격, **R8 안전망 불변식 6종**, 블록 중복 점유 없음, 전 오더 완료 |
| `test_bridge.py` | 7 | 제어 명령, 페이로드 스키마, 변화 시에만 발행, Code Crimson 중복 거부, 좌표 반올림 |
| `test_edge_cases.py` | 5 | 5오더 폭주, **교행 R13 OFF/ON 대조**, EVAC_SPUR, Code Crimson 타이밍, 결정론 |
| `test_service_dip.py` | 6 | OR 서비스 딥(SERVICING · BLUE_CMD) |

통합 중 문제가 생기면 **이 테스트 통과 여부로 책임을 가릅니다.** 통과하면 관제 로직은 정상이고,
문제는 ROS 경계(빌드·QoS·네트워크)에 있습니다.

### 6-6. 발표용 KPI 표 뽑기

```bash
python3 src/rail_control_core/tools/kpi_report.py        # 모드 A/B 비교표 + 기준값 대조
python3 src/rail_control_core/tools/kpi_report.py --md   # 마크다운 표
```

기준값에서 벗어나거나 문서–코드가 갈라지면 ⚠️를 찍고 **exit code 1**로 끝납니다.
시연 전 반드시 **exit 0**을 확인하세요.

### 6-7. 자주 겪는 오류

| 증상 | 원인 | 해결 |
|---|---|---|
| `Package 'rail_control_core' not found` | 소싱 안 함 | `source ~/cobot3_ws/install/setup.bash` |
| `params.yaml`을 못 찾음 | `setup.py`의 `data_files` 누락 | yaml 추가 후 `colcon build` 재실행 |
| 다른 PC에서 토픽이 안 보임 | 도메인/화이트리스트 불일치 | `ROS_DOMAIN_ID=136`, 화이트리스트에 본인 IP 포함 확인 |
| 자기 노드끼리도 통신 안 됨 | 화이트리스트에 `127.0.0.1` 누락 | XML에 루프백 주소 추가 |
| `colcon build` 후 `src/build` 생성 | `src/` 안에서 빌드함 | `~/cobot3_ws`에서 실행 |
| 코드 고쳤는데 반영 안 됨 | `--symlink-install` 없이 빌드 | 재빌드하거나 옵션 추가 |
| 캡슐이 노드에 딱 붙지 않고 0.4 m 앞에 정지 | **정상 동작** (`node_setback`) | 노드는 통과 캡슐과 공유 기하라 끝점 정지 시 겹침 |

---

## 7. 저장소 구조

```
cobot3_ws/
├── src/
│   ├── rail_control_core/          # A — 관제 코어 (ROS 2 패키지)
│   │   ├── config/params.yaml      #   ROS 설정 + 엔진 파라미터 + 회귀 기준값
│   │   ├── rail_control_core/
│   │   │   ├── topology.py         #   블록·회랑·경로 테이블 (v3.1 FROZEN) ★팀 계약★
│   │   │   ├── fsm.py              #   3-FSM 상태 정의
│   │   │   ├── engine.py           #   관제 두뇌 (ROS 무의존) ★핵심★
│   │   │   ├── scenario.py         #   시연 시나리오 발령표
│   │   │   ├── geometry.py         #   논리좌표 → 씬 월드 xyz
│   │   │   ├── bridge.py           #   포트-어댑터 경계 (rclpy 무의존)
│   │   │   └── control_core_node.py#   rclpy 어댑터
│   │   ├── launch/ test/ tools/
│   │   └── README.md               #   관제 코어 상세 문서
│   ├── rail_bridge/                # C — 인터페이스 스키마 + mock 노드 2종
│   ├── ros2_vg10_gripper/          # B — OnRobot VG10 그리퍼 드라이버 (ROS 2 패키지)
│   │   ├── ros2_vg10_gripper/      #   vg10_driver.py · vg10_node.py
│   │   ├── urdf/ meshes/ launch/   #   vg10.urdf · vg10.xacro · STL · USD
│   │   └── package.xml setup.py
│   └── medical-rail-twin/          # D — 대시보드 (서브모듈 포인터, 비어 있음)
├── rail_ui/                        # D — 오프라인 자체포함 관제 대시보드
│   ├── rail_dashboard_v11.html     #   대시보드 본체 (index.html 이 여기로 리다이렉트)
│   ├── rail_dashboard_single.html  #   단일 파일 배포판
│   ├── roslib.min.js fonts/        #   로컬 번들 — 인터넷 없이 동작
│   ├── mock/ voice_ai/             #   목데이터 · 음성 UI
│   ├── start_dashboard.sh          #   http://localhost:8791 로 기동
│   └── README_전달.md              #   다른 PC 전달 방법
├── requirements.txt                # pip 의존성 정본
├── isaacpjt/                       # B·C — Isaac Sim 자산·스크립트
│   ├── assets/                     #   USD 씬(parctice.usd = 최종 씬), 수술팩, 합성 데이터 생성기
│   ├── config/                     #   or_station.json · waypoints_v3.json
│   └── scripts/                    #   브릿지, 레일 드라이버, M0609 모션, OR 하역
├── isaacsim/usd/                   # 쉬프트 씬
└── docs/
    ├── topology_spec.md            # 토폴로지 명세서 (정본, ID 동결)
    ├── 관제_기술문서.md              # 알고리즘·검증 기술 문서
    ├── C연동_매핑표.md               # 인터페이스 이행 대응표
    ├── 시연_큐시트.md                # 발표 진행 큐시트
    ├── 예상_QA.md                   # 심사 예상 질문
    ├── B1F_도면.png · 2F_도면.png    # 기준 도면
    └── A_mode.webm · B_mode.webm     # 모드 A/B 시연 녹화
```

**정본 문서**

| 대상 | 정본 |
|---|---|
| 토폴로지 · 규칙 R1~R13 · 파라미터 | [`docs/topology_spec.md`](docs/topology_spec.md) |
| 알고리즘 · 검증 · 한계 | [`docs/관제_기술문서.md`](docs/관제_기술문서.md) |
| 토픽 페이로드 스키마 | [`src/rail_bridge/rail_bridge/interface_schema.json`](src/rail_bridge/rail_bridge/interface_schema.json) |
| 블록 길이·씬 좌표 | `src/rail_control_core/rail_control_core/topology.py` (코드가 단일 진실 소스) |

---

## 8. 프로젝트 실행 코드 정리

### 8-1. Isaac Sim 실행

Ubuntu 터미널에서:

```bash
export ROS_DOMAIN_ID=136
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

~/isaacsim/isaac-sim.sh
```

Isaac Sim이 켜지면 최종 씬 `parctice.usd` 를 연다.

그다음 **LIVE OFF** 로 설정한다.

> Manipulator 시연 기준으로 `Scene Open → LIVE OFF → Door Controller → Vision Capture → Play → Orchestrator` 순서로 사용했다.

---

### 8-2. Capsule 버튼 + Door Controller

**실행 위치** — Isaac Sim → Window → Script Editor

**실행 파일** — `capsule_button_follow_and_door_controller.py`

이 코드는 다음을 담당한다.

```
Capsule_01 ~ Capsule_10
↓
외부 빨간 버튼 위치 동기화
↓
버튼 Press 감지
↓
Door Open / Close
```

먼저 이 코드를 한 번 Run한다. 정상적으로 실행되면 대략 다음이 출력된다.

```
Connected 10/10
initial door = CLOSED
```

**이 코드는 다른 이동 / Manipulator 코드보다 먼저 실행한다.**

---

### 8-3. Capsule 이동 테스트

이동 테스트 코드는 전부 Isaac Sim Script Editor에서 실행한다. 씬 `parctice.usd` 를 먼저 연 상태여야 한다.

#### 8-3-1. Shaft 상승 / 하강 테스트

사용했던 Shaft 테스트 코드:

```
SB_UP   B1F → F2
   ↓
SB_DN   F2 → B1F
```

실행 순서:

```
1. parctice.usd 열기
2. Script Editor 열기
3. Shaft 테스트 코드 실행
```

이 코드는 Capsule Body를 직접 움직여 `SB_UP → Upper Transfer → SB_DN` 순으로 움직인다. 별도 터미널 실행은 필요 없다.

#### 8-3-2. B1F 전체 이동 테스트

B1F 테스트 코드는 `Capsule_02` 를 사용한다.

경로:

```
BB_01 → BB_02 → BB_03 → BB_04a → BB_05 → BB_06a → BB_06b → BB_06d
→ BB_07 → BB_08 → BB_09 → BB_05 → BB_04b → BB_03 → BB_02 → BB_01
```

코드 자체가 **B1F FULL ROUTE TEST** 로 구성되어 있고 `Capsule_02` 와 B1F Blocks를 사용한다.

실행: `parctice.usd` → Script Editor → **B1F FULL ROUTE** 코드 Run

#### 8-3-3. 2F 전체 이동 테스트

2F 테스트도 Script Editor에서 실행한다.

경로:

```
B2_01 → B2_02 → B2_03 → B2_04a → B2_05 → B2_06 → B2_07a → B2_08 → OR1
→ B2_08 → B2_07b → B2_06 → B2_09 → OR2 → B2_09 → B2_05 → B2_04b
→ B2_03 → B2_02 → B2_01
```

OR1 / OR2도 이 코드 안에 포함돼 있다.

실행: `parctice.usd` → Script Editor → **F2 FULL ROUTE** 코드 Run

#### 8-3-4. 전체 레일 순환 테스트

전체 경로 확인용 코드는 **C02 WHOLE SYSTEM LAP** 이다.

전체 동작:

```
B1 Depot → BB_08 → B1 Main → SB_UP → F2 → OR1 → OR2 → SB_DN → B1 Main → Depot
```

실행:

```
1. parctice.usd 열기
2. Capsule Door/Button Controller 실행
3. Script Editor에서 C02 WHOLE SYSTEM LAP 코드 Run
```

전체 Capsule 레일 주행만 볼 때는 이 코드가 가장 편하다.

---

### 8-4. OCR 실행

OCR은 Script Editor + 별도 Ubuntu 터미널을 같이 사용한다.

**터미널 1 — OCR Node**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=136

cd ~/rokey_cobot3/isaacpjt/scripts
python3 or_label_ocr_node.py
```

이 터미널은 닫지 않고 그대로 둔다.

OCR 결과로 `item_code`, `packaging_date`, `expiration_date`, `order_id`, `delivery_add`, `capsule_id`, `confidence` 등이 여기 출력된다.

---

### 8-5. Label Capture 코드

**실행 위치** — Isaac Sim Script Editor

**파일** — `or_label_capture.py`

실행 후 다음이 뜨는지 확인한다.

```
LABEL CAPTURE MODULE ACTIVE
```

이 코드는 RSD455 카메라에서 SurgicalPack 라벨 이미지를 Capture한다.

---

### 8-6. Manipulator 실행

Manipulator의 메인 코드는 `or_unload_orchestrator.py` 이다. 이 파일이 M0609 + VG10 전체 시퀀스를 담당한다.

동작:

```
Button 접근 → Button Press → Door Open
→ SurgicalPack 접근 → VG10 Grip → PACK_GRIPPED
→ Camera 이동 → Label Capture / OCR
→ Tray 이동 → VG10 Release → PACK_UNLOADED
→ Button 접근 → Button Press → Door Close
```

실제 전체 Sequence도 이 순서다.

---

### 8-7. Manipulator + OCR 최종 실행 순서

팀원이 실제로 실행할 때는 이 순서대로 하면 된다.

**터미널 1** — 계속 켜둔다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=136

cd ~/rokey_cobot3/isaacpjt/scripts
python3 or_label_ocr_node.py
```

**터미널 2**

```bash
export ROS_DOMAIN_ID=136
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

~/isaacsim/isaac-sim.sh
```

**Isaac Sim**

```
1. parctice.usd Open

2. LIVE OFF

3. Script Editor
   capsule_button_follow_and_door_controller.py
   → Run

4. Script Editor
   or_label_capture.py
   → Run

5. ▶ PLAY

6. Script Editor
   or_unload_orchestrator.py
   → Run
```

이게 Manipulator + Door + VG10 + OCR 시연 기본 실행 순서다.

---

### 8-8. 관제 없이 Manipulator만 테스트

`or_unload_orchestrator.py` 에서:

```python
AUTO_TRIGGER = False
```

로 설정한다. 그러면 Script Editor에서 `or_unload_orchestrator.py` → Run 하는 순간 Manipulator Sequence가 바로 시작된다.

관제와 연결해서 쓸 때는 반드시 다음으로 복원한다.

```python
AUTO_TRIGGER = True
```

---

### 8-9. Capsule POV Camera

별도 Python 실행이 필요 없다.

Capsule 아래 Camera가 이미 Child로 들어가 있으므로 `Viewport → Camera 선택 → Camera_01 ~ Camera_10` 중 하나를 선택하면 된다.

Capsule이 움직이면 Camera도 같이 이동한다.

---

### 8-10. 가장 중요한 실행 주의사항

**Script Editor 코드는 같은 코드를 여러 번 Run하지 않는다.**

특히 아래 세 개를 여러 번 실행하면 문제가 생길 수 있다.

```
capsule_button_follow_and_door_controller.py
or_label_capture.py
or_unload_orchestrator.py
```

| 중복 실행 시 발생 |
|---|
| ROS subscriber 중복 |
| Update subscription 중복 |
| Node 중복 |
| Event callback 중복 |

그래서 기본적으로 `Isaac 새로 실행 → Scene Open → 각 코드 한 번씩만 Run` 으로 하는 게 안전하다.

---

### 8-11. 팀원용 초간단 버전

**[Isaac 실행]**

```bash
export ROS_DOMAIN_ID=136
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
~/isaacsim/isaac-sim.sh
```

**[Scene]**

```
parctice.usd Open
LIVE OFF
```

**[Script Editor 실행 순서]**

```
1. capsule_button_follow_and_door_controller.py

2. or_label_capture.py
   ※ OCR 사용 시

3. ▶ PLAY

4. or_unload_orchestrator.py
```

**[OCR 터미널]**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=136
cd ~/rokey_cobot3/isaacpjt/scripts
python3 or_label_ocr_node.py
```

**[이동 테스트]**

```
Shaft 테스트
B1F FULL ROUTE
F2 FULL ROUTE
C02 WHOLE SYSTEM LAP
```

→ 전부 `parctice.usd` 를 연 뒤 Isaac Sim Script Editor에서 Run
