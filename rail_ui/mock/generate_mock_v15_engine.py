# -*- coding: utf-8 -*-
"""
v15: 관제 엔진(rail_control_core.engine.Engine) 실측 tick 트레이스를 그대로 목데이터로
변환한다. v14까지는 "최종 도착시각(KPI_*_ARRIVE)에 맞춰 블록별 등속 이동을 사후
스케일링"하는 근사 방식이었는데, 그 방식은 최종 도착시각만 실측과 맞고 중간 과정
(콘보이 차두 YIELD/RESUME 정지, OR 서비스 딥 SERVICE_START/BLUE_CMD/SERVICE_DONE
단계별 타이밍 등)은 반영되지 않아 애니메이션이 실제 물리와 어긋났다(콘보이가 서로
가까워질 때 "튕기는" 것처럼 보이는 문제의 근본 원인 중 하나).

이 스크립트는 근사/스케일링을 전혀 하지 않는다 — scenario.build()로 만든 실제
Engine을 DT=1/30s 단위로 그대로 돌리고, 0.2초마다 캡슐/블록/오더 상태를 있는 그대로
샘플링한다. 모드 B(선점형)는 관제 엔진이 실제로 계산하는 물리적 최단 시간이므로
이 트레이스가 곧 "모드 B 최대한 빠른 루트"의 정답이다.
"""
import sys, json, os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_ENGINE_SRC = os.path.normpath(os.path.join(_THIS_DIR, "..", "..", "src", "rail_control_core"))
sys.path.insert(0, _ENGINE_SRC)

from rail_control_core import scenario
from rail_control_core.scenario import build
from rail_control_core.fsm import OrderState, CapsuleState
from rail_control_core import topology as T
from rail_control_core import geometry as G


def _capsule_xyz(c):
    """캡슐의 씬 월드 좌표 — bridge.Bridge._xyz 와 동일 규칙(단일 출처 geometry.py).
    이 값이 채워져 있어야 대시보드가 목데이터를 /capsule_pose 로 재발행할 때
    Isaac(GPU PC)이 프림을 실제 위치로 옮길 수 있다."""
    if c.state == CapsuleState.DOCKED:
        return G.dock_slot_xyz(int(c.cid[1:]) - 1)
    if c.state == CapsuleState.SERVICING and c.svc_bid:
        return G.service_pose_xyz(c.svc_bid, c.svc_s)
    if c.block:
        return G.pose_to_xyz(c.block, c.pos, bool(c.fwd))
    return (0.0, 0.0, 0.0)

DT = 1.0 / 30.0
SAMPLE_EVERY = 6  # 6 * DT = 0.2s — 기존 v14와 동일한 프레임 간격 유지
T_LIMIT = 300.0

ITEM_LABELS = {
    "O-4": "응급 수혈혈액",
    "O-3": "응급약품",
    "O-2": "항암제·TPN 조제약",
    "O-1": "멸균 물품 공급",
    "O-5": "사용 기구 회수",
}

ALL_BLOCK_IDS = list(T.BLOCKS.keys())

# order_event에서 subject가 오더 id인 이벤트 종류 (그 외는 캡슐 id로 취급)
ORDER_SUBJECT_EVENTS = {"ORDER_RELEASE", "ORDER_ARRIVE", "RTA_ENGAGED", "PRIORITY_PROMOTED"}


def run_engine(mode):
    """실제 Engine을 끝까지 돌리며 0.2초 간격으로 프레임을 샘플링한다.

    모드 A(FCFS 베이스라인)는 R13 교행(meet_pass)까지 꺼서 대피 레인을 전혀 쓰지
    않고 이송을 시도한다 — 공급(O-1,C05)·회수(O-5,C08)가 단선에서 정면 조우하면
    풀리지 않아 DEADLOCK까지 가는 것이 이 baseline의 의도된 실패 시나리오다.
    (test_edge_cases.test_meet_pass_off_deadlocks_on_resolves와 동일한 토글 방식 —
    Engine 자체는 그대로 두고 scenario.PARAMS를 빌드 직전에만 조정한다.)
    모드 B는 R13이 켜진 정상 선점형 그대로 돌린다.
    """
    scenario.PARAMS["meet_pass_enabled"] = (mode == "B")
    # 실제 매니퓰레이터(m0609) OR 하역 시퀀스(isaacpjt/scripts/or_unload_orchestrator.py)
    # 실측 소요시간으로 WORK 체류(work_dwell_sec)를 맞춘다 — 기존 2.0s는 자리표시자였음.
    #   PRESS_OPEN : goto(button,2.0)+release대기 0.6+goto(button_approach,1.5)+문열림애니 0.7 = 4.8s
    #   MOVE(취출) : goto(grasp,2.5)+grip 0.8+goto(camera,3.0)+hold 2.0+goto(tray,3.0)+grip 0.8 = 12.1s
    #   PRESS_CLOSE: goto(button,2.5)+release대기 0.6+goto(button_approach,1.5)+문닫힘애니 0.7 = 5.3s
    #   합계 4.8+12.1+5.3 = 22.2s (양 모드 공통 — 물리 로봇 시퀀스라 선점 여부와 무관)
    scenario.PARAMS["work_dwell_sec"] = 22.2
    eng = build(mode)
    frames = []
    prev_block_key = [None]

    def sample():
        t_s = round(eng.t, 1)

        orders_payload = {}
        for oid, o in eng.orders.items():
            orders_payload[oid] = {
                "priority": o.prio,
                "state": o.state.value,
                "release": round(o.release_t, 2),
                "due": round(o.due_t, 2),
                "arrive": round(o.arrive_t, 2) if o.arrive_t is not None else None,
                "wait": round(o.wait_total, 2),
                "capsules": list(o.capsule_ids),
            }
        control_state = {
            "sim_t": t_s, "kind": "control_state", "mode": mode, "running": True,
            "orders": orders_payload, "capsule_count": len(eng.capsules),
        }

        capsules_payload = []
        for cid, c in eng.capsules.items():
            oid = c.order.oid if c.order else None
            x, y, z = _capsule_xyz(c)
            capsules_payload.append({
                "capsule_id": cid, "block_id": c.block or "",
                "pos_m": round(c.pos, 3), "forward": c.fwd,
                "state": c.state.value, "order_id": oid,
                # v3.7 OR 서비스 딥: SERVICING 중엔 c.block이 비고(레일 점유 이탈)
                # svc_bid(B2-08/B2-09)·svc_s(딥 경로 진행거리 0~9.09m, WORK=5.59m)로 추적
                "svc_bid": c.svc_bid or "", "svc_s": round(c.svc_s, 3),
                "x": round(x, 4), "y": round(y, 4), "z": round(z, 4),
            })
        capsule_pose = {"sim_t": t_s, "kind": "capsule_pose", "capsules": capsules_payload}

        occ_map = {}
        for cp in capsules_payload:
            if cp["block_id"]:
                occ_map.setdefault(cp["block_id"], []).append(cp["capsule_id"])
        blocks = []
        for bid in ALL_BLOCK_IDS:
            occ = occ_map.get(bid, [])
            cap = T.BLOCKS[bid][3]
            blocks.append({
                "block_id": bid, "state": "OCCUPIED" if occ else "FREE",
                "occupancy": len(occ), "capacity": cap, "locked": False,
                "corridor": "", "dir": 1, "capsule_ids": occ,
            })
        bkey = json.dumps(blocks, sort_keys=True)
        changed = (bkey != prev_block_key[0])
        prev_block_key[0] = bkey

        frames.append({
            "t": t_s,
            "control_state": control_state,
            "capsule_pose": capsule_pose,
            "block_state": {"sim_t": t_s, "kind": "block_state", "blocks": blocks} if changed else None,
        })

    def all_done_and_settled():
        # 오더 회계상 DONE(=레일 인계 완료, KPI 확정)이어도 매니퓰레이터 딥 연출
        # (work_dwell_sec=22.2s 반영 후 SERVICE_DONE까지)이 안 끝난 캡슐이 있으면
        # 트레이스를 계속 재생한다 — 안 그러면 뒷 캡슐이 딥 중간에 뚝 끊겨 멈춘다.
        if not all(o.state == OrderState.DONE for o in eng.orders.values()):
            return False
        if any(c.state == CapsuleState.SERVICING for c in eng.capsules.values()):
            return False
        return True

    sample()  # t=0 스냅샷
    tick_count = 0
    deadlock_extra = 0.0
    DEADLOCK_TAIL_SEC = 8.0  # 데드락 확정 후에도 잠깐 더 재생해 "멈춰선 캡슐"이 화면에 보이게 함
    while eng.t < T_LIMIT and not all_done_and_settled():
        eng.tick()
        tick_count += 1
        if tick_count % SAMPLE_EVERY == 0:
            sample()
        if eng._deadlock_logged:
            deadlock_extra += DT
            if deadlock_extra > DEADLOCK_TAIL_SEC:
                break
    sample()  # 종료 시점 최종 스냅샷 (makespan/데드락 정지 프레임 누락 방지)

    return eng, frames, round(eng.t, 2)


def build_order_events(eng, mode, final_t):
    events = []
    for t, ev, subj, detail in eng.events:
        e = {"sim_t": round(t, 2), "kind": "order_event", "event": ev, "subject": subj, "detail": detail}
        if ev in ORDER_SUBJECT_EVENTS and subj in eng.orders:
            o = eng.orders[subj]
            e["priority"] = o.prio
            e["item"] = ITEM_LABELS.get(subj, "")
            e["state"] = o.state.value
        elif subj in eng.capsules:
            c = eng.capsules[subj]
            oid = c.order.oid if c.order else None
            e["capsule"] = subj
            e["order_id"] = oid
            e["priority"] = c.order.prio if c.order else None
            if oid:
                e["item"] = ITEM_LABELS.get(oid, "")
        else:
            e["priority"] = None
        events.append(e)

    # CODE_CRIMSON 배너용 합성 이벤트 (엔진은 로그하지 않지만 UI가 소비함) — O-4 발령 시각에 부여
    o4_release = next((e for e in events if e["event"] == "ORDER_RELEASE" and e["subject"] == "O-4"), None)
    if o4_release:
        events.append({
            "sim_t": o4_release["sim_t"], "kind": "order_event", "event": "CODE_CRIMSON",
            "subject": "O-4", "detail": "Code Crimson 콘보이 출동 (BB-09 -> ST-OR2 집결)",
            "priority": 0, "item": ITEM_LABELS["O-4"], "state": "EN_ROUTE", "code_red": True,
        })

    # 매니퓰레이터(m0609) OR 하역 서브이벤트 — 씬의 or_unload_orchestrator.py 시퀀스를
    # 로그에 재현한다. WORK 도착(=BLUE_CMD 시각 - work_dwell_sec)부터 dwell 동안 진행:
    #   버튼프레스→도어개방(4.8s)→파지→카메라 라벨판독(hold)→트레이 적재→버튼재프레스→도어폐쇄(22.2s)
    dwell = float(scenario.PARAMS.get("work_dwell_sec", 22.2))
    OR_SEQ = [
        (0.0,   "BUTTON_PRESS", "m0609: 캡슐 버튼 프레스 → 도어 개방 지령"),
        (4.8,   "DOOR_OPEN",    "캡슐 도어 개방 완료 (slide_y 0.7s)"),
        (7.3,   "ARM_PICK",     "그리퍼 파지 — 수술팩 12kg 취출"),
        (12.9,  "VISION_OCR",   "카메라 위치 정지 — 라벨 판독 (hold 2.0s)"),
        (16.9,  "ARM_PLACE",    "OR 트레이 적재 완료 → 그리퍼 개방"),
        (17.4,  "BUTTON_PRESS", "m0609: 버튼 재프레스 → 도어 폐쇄 지령"),
        (dwell, "DOOR_CLOSED",  "캡슐 도어 폐쇄 완료 → 출차(BLUE) 승인 대기"),
    ]
    for se in [e for e in events if e["event"] == "SERVICE_START" and "B2-09" in (e["detail"] or "")]:
        cap = se["subject"]
        blue = next((e for e in events if e["event"] == "BLUE_CMD" and e["subject"] == cap), None)
        t0 = round((blue["sim_t"] - dwell) if blue else (se["sim_t"] + 11.2), 2)
        for dt, ev, detail in OR_SEQ:
            events.append({
                "sim_t": round(t0 + dt, 2), "kind": "order_event", "event": ev,
                "subject": cap, "detail": detail, "capsule": cap,
                "order_id": se.get("order_id"), "priority": se.get("priority"),
                "item": se.get("item", ""), "actor": "m0609",
            })

    all_done = all(o.state == OrderState.DONE for o in eng.orders.values())
    if all_done:
        # 모든 오더가 완주한 경우의 makespan은 "마지막 캡슐이 실제로 도착한 시각"
        # (arrive_t 최댓값) — 엔진 tick 루프가 회계상 DONE으로 넘어가는 시각(eng.t)은
        # 후처리 지연이 조금 있어 이보다 늦다.
        makespan = round(max(o.arrive_t for o in eng.orders.values()), 2)
        events.append({
            "sim_t": makespan, "kind": "order_event", "event": "SIM_DONE",
            "subject": "sim", "detail": f"makespan={makespan}", "priority": None, "item": "", "state": None,
        })
    else:
        # 데드락으로 완주하지 못한 경우 makespan은 트레이스가 멈춘 시각(final_t)이고,
        # SIM_DONE은 내보내지 않는다 — 실제로 끝나지 않았고, DEADLOCK 이벤트(이미
        # eng.events에서 변환됨)가 UI 배너로 그 사실을 계속 보여준다
        # (모드 전환/리셋 전까지 자동으로 닫히지 않음).
        makespan = final_t

    events.sort(key=lambda e: e["sim_t"])
    return events, all_done, makespan


def build_kpi(eng, mode, makespan, completed):
    orders = {}
    for oid, o in eng.orders.items():
        orders[oid] = {
            "priority": o.prio,
            "arrive": round(o.arrive_t, 2) if o.arrive_t is not None else None,
            "wait": round(o.wait_total, 2),
        }
    arrival_order = sorted(
        (oid for oid, o in eng.orders.items() if o.arrive_t is not None),
        key=lambda k: eng.orders[k].arrive_t,
    )
    return {
        "kind": "kpi", "mode": mode, "makespan": makespan, "completed": completed,
        "orders": orders, "arrival_order": arrival_order,
    }


def route_node(bid, fwd, end):
    """블록 (id, 진행방향)의 시작/끝 노드 id. end=False면 시작 노드, True면 끝 노드."""
    a, b = T.BLOCKS[bid][0], T.BLOCKS[bid][1]
    if fwd:
        return b if end else a
    return a if end else b


def build_order_meta(eng):
    meta = {}
    for oid, o in eng.orders.items():
        cid0 = o.capsule_ids[0]
        route = eng.capsules[cid0].route
        bid0, fwd0 = route[0]
        bidN, fwdN = route[-1]
        meta[oid] = {
            "item": ITEM_LABELS.get(oid, ""), "priority": o.prio,
            "capsules": list(o.capsule_ids),
            "route_start": route_node(bid0, fwd0, end=False),
            "route_end": route_node(bidN, fwdN, end=True),
        }
    return meta


def main():
    eng_b, frames_b, final_t_b = run_engine("B")
    events_b, completed_b, makespan_b = build_order_events(eng_b, "B", final_t_b)
    kpi_b = build_kpi(eng_b, "B", makespan_b, completed_b)

    eng_a, frames_a, final_t_a = run_engine("A")
    events_a, completed_a, makespan_a = build_order_events(eng_a, "A", final_t_a)
    kpi_a = build_kpi(eng_a, "A", makespan_a, completed_a)

    order_meta = build_order_meta(eng_b)  # 경로 자체는 모드 무관 (A/B 공통 토폴로지)

    print(f"[모드 B] makespan={makespan_b}s completed={completed_b} orders={kpi_b['orders']}")
    print(f"[모드 A] makespan={makespan_a}s completed={completed_a} orders={kpi_a['orders']}")
    print(f"frames_b={len(frames_b)} frames_a={len(frames_a)} "
          f"events_b={len(events_b)} events_a={len(events_a)}")

    out = {
        "_about": "v15.1: 관제 엔진(engine.py) 실측 tick 트레이스 직결 생성. 모드 B는 "
                  "선점형 최단 시간 그대로, 모드 A는 R13 교행(meet_pass)까지 비활성화한 "
                  "FCFS 베이스라인 — O-1 공급(C05)/O-5 회수(C08)가 단선에서 정면 조우해도 "
                  "대피 레인으로 치환하지 않고 그대로 진입을 시도하다 DEADLOCK까지 간다.",
        "frames_b": frames_b,
        "frames_a": frames_a,
        "order_events_b": events_b,
        "order_events_a": events_a,
        "kpi_a": kpi_a,
        "kpi_b": kpi_b,
        "order_meta": order_meta,
    }

    out_path = os.path.join(_THIS_DIR, "mock_data_v15.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, separators=(",", ":"))
    print("wrote", out_path)


if __name__ == "__main__":
    main()
