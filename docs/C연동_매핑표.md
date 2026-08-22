# C(연동) 인터페이스 이행 매핑표 — JSON 스키마 → rail_interfaces msg

> ## ⚠️ 전송 방식만 다릅니다 — 커스텀 msg 대신 `std_msgs/String` + JSON
>
> 이 표의 **구조적 합의는 전부 그대로 반영되었습니다**: 위치 진실 소스 = 관제 코어,
> `/capsule_pose` 방향 반전(코어 발행), `/capsule_cmd` 폐기, ID 표기 통일, xyz 코어 계산.
>
> 다만 **`rail_interfaces` 커스텀 msg 패키지는 만들지 않았습니다.** 팀이 `06c0783`
> 에서 `rail_msgs` 를 삭제하고 `std_msgs/String` + JSON 으로 합의했기 때문입니다
> (커스텀 msg 는 4대 PC 전원 재빌드 필요 + rosbridge 변환 비용).
>
> **읽는 법**: 아래 표의 `msg 필드` 이름이 곧 **JSON 키 이름**입니다. 1:1 대응은 그대로
> 성립하므로 C 쪽 작업량은 동일합니다. 배열 msg(`CapsulePoseArray`)는 JSON 의
> `{"sim_t": ..., "capsules": [...]}` 객체에 해당합니다.
>
> ```python
> # 브릿지 수신 예시
> data = json.loads(msg.data)              # std_msgs/String
> for c in data["capsules"]:
>     prim = f"/World/Capsules/{c['capsule_id']}"
>     set_transform(prim, c["x"], c["y"], c["z"])
>     set_visible(prim, c["state"] != "REMOVED")
> ```
>
> 실제 발행 중인 페이로드 전문: [`src/rail_bridge/rail_bridge/interface_schema.json`](../src/rail_bridge/rail_bridge/interface_schema.json)


합의(2026-08-21): 위치 진실 소스 = 관제 코어 / 정식 msg 사용 / ID 표기 통일 / `/capsule_cmd` 폐기.
이 문서는 기존 `rail_bridge/interface_schema.json` 및 mock 2종을 새 규격으로 옮길 때의 1:1 대응표입니다.

## 1. /capsule_pose — 방향 반전: 이제 **코어가 발행**, 브릿지·UI는 구독

`rail_interfaces/msg/CapsulePoseArray` (30Hz, **BEST_EFFORT depth1** — 구독 QoS 동일하게 맞출 것)

| 기존 JSON 필드 | 새 msg 필드 | 비고 |
|---|---|---|
| `capsule_id: "C-01"` | `capsules[].capsule_id: "C01"` | **하이픈 제거** (명세 §5) |
| `current_block` | `capsules[].block_id` | REMOVED 시 `""` |
| `node` | (없음) | 노드 대신 `block_id + pos_m + forward`가 정본 |
| `x, y, z` | `capsules[].x, y, z` | **코어가 계산해 줌** — 프림 트랜스폼에 그대로 적용 |
| `status` | `capsules[].state` | 값 집합: DOCKED/QUEUED/STANDBY/MOVING/YIELD_WAIT/EVACUATED/FINISHING/UNLOADING/REMOVED |
| (없음) | `capsules[].pos_m, forward` | 블록 진입점 기준 진행거리·방향 (정밀 보간 필요 시 사용) |
| (없음) | `sim_t`, `stamp` | 시뮬 시각 / ROS 시각 |

브릿지 처리: `for c in msg.capsules:` → `/World/Capsules/{c.capsule_id}` 트랜스폼 = (c.x, c.y, c.z).

> **[확인 대기]** 실제 브릿지(`isaacpjt/scripts/isaac_twin_m2_bridge.py`)는 프림을
> `/World/Capsules/Capsule_01` 형식으로 씁니다. 브릿지가 내부에서 `capsule_id` → 프림을
> 매핑하므로 동작에는 문제가 없습니다. 명명 규약을 코드(`Capsule_01`) 기준으로 확정할지
> 석형님 회신 후 이 문서를 실물에 맞춰 고칩니다.
`state == "REMOVED"` 면 프림 숨김(visibility off) 권장. 경로 규약 `/World/Capsules/C01~C10` 유지.

## 2. /capsule_cmd — **폐기**

`kind=="capsule_state"` 정보는 pose의 `state` 필드로, `kind=="preempt"` 연출 정보는
`/order_event` 의 `event ∈ {YIELD, EVAC_LANE, EVAC_SPUR, FINISH_ALLOWED, RESUME}` 로 대체됩니다.

| 기존 preempt JSON | 새 OrderEvent 대응 |
|---|---|
| `action: "EVACUATE"` | `event: "EVAC_LANE"` (레인) 또는 `"EVAC_SPUR"` (지선) |
| `by: "CR-1"` | 선점 주체는 항상 P0 — `detail` 에 맥락 포함 |
| `target`, `block`, `siding` | `subject` = 대피한 캡슐 id, `detail` = `"B2-05->B2-04b"` 형식 |

## 3. /order_event — 코어 발행 (RELIABLE depth50)

| 기존 JSON | 새 msg 필드 | 비고 |
|---|---|---|
| `order_id: "CR-1"` | `subject: "O-4"` | **오더 ID 체계 통일** (명세 §6-1) |
| `kind: "order_event"` | `event` | ORDER_RELEASE / ORDER_ARRIVE / SIM_DONE / CODE_CRIMSON 등 |
| `priority`, `item` | `detail` | 예: ORDER_RELEASE 의 detail = "P0" |
| `code_red: true` | (폐기) | 명칭은 **Code Crimson** — 발동 이벤트는 `event: "CODE_CRIMSON"` |
| `t` | `sim_t` | |

## 4. 서비스 — ⚠️ 확정 변경: `std_srvs/Trigger` 8종 (2026-08-22)

`rail_interfaces/srv/CodeCrimson`, `rail_interfaces/srv/SimCommand` 는 **deprecated** 입니다.
만들지 마세요. 서비스는 관제 코어 노드(`control_core_node.py`)가 제공하며,
커스텀 srv 없이 ack 를 받기 위해 **명령당 하나씩** `std_srvs/srv/Trigger` 로 나눴습니다.

| 서비스 | 역할 |
|---|---|
| `/sim_start` | 시나리오 진행 시작 |
| `/sim_pause` | 일시정지 (시뮬 시간 정지) |
| `/sim_reset` | 현재 모드로 초기화 (t=0) |
| `/sim_status` | t·mode·running·오더 상태를 message 로 반환 |
| `/sim_mode_a` | 모드 A(FCFS)로 초기화 — 비교군 |
| `/sim_mode_b` | 모드 B(PEDD 선점)로 초기화 |
| `/code_crimson` | 예약 P0 발령을 지금으로 앞당김 (이미 발령됐으면 사유와 함께 거부) |
| `/code_red` | 디포 유휴 캡슐로 추가 P0 생성 (예약 P0 이후에도 사용 가능) |

```bash
ros2 service call /sim_start std_srvs/srv/Trigger
```

기존 `SimCommand{command: ...}` 의 6개 명령은 `/sim_*` 6종에 1:1 대응합니다.

## 5. mock 노드 수정 요약

- **mock_sim**: `/capsule_pose` 발행 코드 → **구독**으로 변경 (코어→씬 단방향). 수신 xyz를 프림에 적용만.
- **mock_core**: 발행 스키마를 위 msg 타입으로 교체, ID 를 C01/O-4 체계로. 시연 4장면 이벤트는
  실제 코어가 내는 이름(YIELD, FINISH_ALLOWED, EVAC_LANE, CODE_CRIMSON, SIM_DONE)과 동일하게.
- 검증: 실코어 노드를 켜고 `ros2 topic echo /capsule_pose` 값과 mock 처리 결과가 같은 프림 위치를
  만드는지 1회 대조하면 끝.

## 6. 좌표 규칙 (참고)

- Z: B1F 레일 4.0 / 2F 레일 13.0, 샤프트는 z 보간 (SB-UP x=-7.25, SB-DN x=-6.75)
- L자 블록: BB-08 경유점 (6.5, 5.0) / BB-09 경유점 (3.2, -4.0)
- DOCKED 캡슐: 디포 측면 가상 슬롯 배치 중 — **B의 도크 좌표 회신 시 코어만 수정** (브릿지 변경 불필요)
