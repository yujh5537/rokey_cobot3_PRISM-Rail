"""시연 시나리오 '동시다발 오더와 계단식 선점' v2 — 토폴로지 v3.1 / 명세서 §6-1 발령표와 동기화.
가정: P1~P3 담당 캡슐은 출발 스테이션 선배치(STANDBY), 콘보이 4대는 대기열(QUEUED).
"""
from .engine import Engine
from .fsm import Order, Capsule, CapsuleState
from . import topology as T

PARAMS = {
    # A1 속도 체계 (2026-08-23 확정): 수평 평시 1.0 / 회복상한 2.0, 샤프트 0.5, 곡선·분기 0.5
    "v_nominal": 1.0,
    "v_max": 2.0,
    "speed_shaft": 0.5,
    "speed_curve": 0.5,
    "accel": 0.8,               # 가속 제한 [m/s^2] (용혈 방지)
    # RTA 슬랙 회복 (R11) / 자동 승격 (R12)
    "rta_slack_threshold": 10.0,
    "rta_margin": 0.4,
    # R9 개정(v3.5): Code Crimson 활성 중 콘보이 캡슐에 한해 블록 점유 상한 개방
    "convoy_bunch_cap": 4,
    "meet_pass_enabled": True,   # R13 교행 (3부 대조 시연용 토글)
    # v3.6.1 씬 실사 반영: 대기 정지는 노드에서 0.4m 물러남 / 대피는 레인 중앙 홀드
    "node_setback": 0.4,
    "evac_hold_frac": 0.5,
    # v3.7 OR 서비스 딥 (B 3차 핸드오프 "추가된 좌표"): 딥 서행 0.5m/s,
    # WORK 하단에서 작업 체류 2s 후 관제가 BLUE 이동 명령을 발행한다.
    "speed_service": 0.5,
    "work_dwell_sec": 2.0,
    "pitch": 0.9,
    "yield_window_sec": 15.0,
    "unload_sec": 2.0,
    "stall_timeout_sec": 20.0,
        # 시연 스케일 due (A2 확정): 선점 지연이 슬랙을 실제로 위협하는 값.
    # 실세계 값(P1 180 / P2 600 / P3 1800)은 명세서 §6-1 비고에 병기.
    "priority_due_sec": {0: 30, 1: 40, 2: 77, 3: 130},
}

# 오더: (id, prio, route, release_t, capsule_ids)
# (id, prio, route, release_t, capsules, speed_cap)
# v3.6: 공급/회수 짝 흐름 — O-1 멸균 공급(CSR->OR1)은 회수와 교차하지 않는 후속 슬롯,
#       O-5 사용 기구 회수(OR1->CSR)가 역방향 배우로서 대피·교행 장면 담당
ORDER_DEFS = [
    ("O-1", 3, "P3_OR1", 2.0, ["C05"], 0.7),   # 멸균 물품 공급 CSR->OR1 (적재 안정 저속)
    ("O-2", 2, "P2_ICU", 1.0, ["C06"], None),
    ("O-3", 1, "P1_ICU", 2.0, ["C07"], None),
    ("O-4", 0, None, 8.0, ["C01", "C02", "C03", "C04"], None),  # Code Crimson 콘보이
    ("O-5", 3, "P3_CSR", 17.0, ["C08"], 0.7),   # 사용 기구 회수 OR1->CSR (역방향)
]
# v3.5: Code Crimson 콘보이는 OR2 단일 집결 (구 OR1/OR2 분산 폐기 — A 결정 2026-08-24)
CONVOY_ROUTES = {"C01": "P0_OR2", "C02": "P0_OR2", "C03": "P0_OR2", "C04": "P0_OR2"}


def build(mode: str) -> Engine:
    eng = Engine(PARAMS, mode)
    for oid, prio, route, rel, cids, spd in ORDER_DEFS:
        due = rel + PARAMS["priority_due_sec"][prio]
        eng.orders[oid] = Order(oid, prio, route or "convoy", rel, due, cids, speed=spd)
    # 선배치 캡슐 (출발 스테이션 대기)
    standby = {"C05": "P3_OR1", "C06": "P2_ICU", "C07": "P1_ICU", "C08": "P3_CSR"}
    for cid, rname in standby.items():
        oid = next(o for o, _, r, _, cs, _ in ORDER_DEFS if cid in cs)
        c = Capsule(cid, eng.orders[oid], list(T.ROUTES[rname]),
                    idx=0, state=CapsuleState.STANDBY)
        bid, fwd = c.route[0]
        c.block, c.fwd, c.pos = bid, fwd, 0.0
        eng.capsules[cid] = c
        eng.occ[bid].append(c)
        eng._corridor_update_on_enter(bid, fwd)
    # 콘보이 4대 (출동 대기열 BB-08, 선두가 N-B1측)
    L = T.BLOCKS["BB-08"][2]
    for i, cid in enumerate(["C01", "C02", "C03", "C04"]):
        c = Capsule(cid, eng.orders["O-4"], list(T.ROUTES[CONVOY_ROUTES[cid]]),
                    idx=-1, state=CapsuleState.QUEUED)
        c.block, c.fwd, c.pos = "BB-08", True, L - i * PARAMS["pitch"]
        eng.capsules[cid] = c
        eng.occ["BB-08"].append(c)
    # 디포 유휴 2대 (점유만) — v3.6: C08 은 ST-OR1 선배치로 이동
    for cid in ["C09", "C10"]:
        c = Capsule(cid, None, [], state=CapsuleState.DOCKED)
        c.block = "BB-07"
        eng.capsules[cid] = c
        eng.occ["BB-07"].append(c)
    return eng


def run(mode: str, verbose: bool = False) -> dict:
    eng = build(mode)
    result = eng.run()
    if verbose:
        print(f"\n=== 모드 {mode} 이벤트 타임라인 ===")
        for t, ev, subj, d in eng.events:
            print(f"  t={t:6.2f}  {ev:14s} {subj:5s} {d}")
    result["scene_events"] = sorted({ev for _, ev, _, _ in eng.events}
                                    & {"YIELD", "EVAC_LANE", "EVAC_SPUR", "FINISH_ALLOWED",
                                       "MEET_PASS", "RTA_ENGAGED", "PRIORITY_PROMOTED"})
    return result


def build_custom(mode: str, order_defs, standby: dict[str, str],
                 convoy: dict[str, str] | None = None,
                 idle: tuple = ("C08", "C09", "C10")):
    """엣지케이스용 빌더 — 발령표·선배치·콘보이 구성을 자유 지정.
    order_defs: [(oid, prio, route_or_None, release_t, [cids], speed)]
    standby: cid -> route명 (출발 스테이션 선배치)
    convoy: cid -> route명 (BB-08 대기열, 선두부터)"""
    from .engine import Engine
    from .fsm import Order, Capsule, CapsuleState
    from . import topology as T
    eng = Engine(PARAMS, mode)
    for oid, prio, route, rel, cids, spd in order_defs:
        due = rel + PARAMS["priority_due_sec"][prio]
        eng.orders[oid] = Order(oid, prio, route or "convoy", rel, due, cids, speed=spd)
    for cid, rname in standby.items():
        oid = next(o for o, _, _, _, cs, _ in order_defs if cid in cs)
        c = Capsule(cid, eng.orders[oid], list(T.ROUTES[rname]),
                    idx=0, state=CapsuleState.STANDBY)
        bid, fwd = c.route[0]
        c.block, c.fwd, c.pos = bid, fwd, 0.0
        eng.capsules[cid] = c
        eng.occ[bid].append(c)
        eng._corridor_update_on_enter(bid, fwd)
    if convoy:
        L = T.BLOCKS["BB-08"][2]
        for i, (cid, rname) in enumerate(convoy.items()):
            oid = next(o for o, _, _, _, cs, _ in order_defs if cid in cs)
            c = Capsule(cid, eng.orders[oid], list(T.ROUTES[rname]),
                        idx=-1, state=CapsuleState.QUEUED)
            c.block, c.fwd, c.pos = "BB-08", True, L - i * PARAMS["pitch"]
            eng.capsules[cid] = c
            eng.occ["BB-08"].append(c)
    for cid in idle:
        c = Capsule(cid, None, [], state=CapsuleState.DOCKED)
        c.block = "BB-07"
        eng.capsules[cid] = c
        eng.occ["BB-07"].append(c)
    return eng


def main() -> None:
    """ROS 없이 시나리오를 A/B 두 모드로 완주시키는 CLI (setup.py 의 `sim` 진입점)."""
    errs = T.validate()
    print("경로 무결성:", "OK" if not errs else errs)
    for mode in ("A", "B"):
        r = run(mode, verbose=True)
        print(f"\n[모드 {mode}] makespan={r['makespan']}s  orders={r['orders']}"
              f"\n  발생 장면: {r['scene_events']}")


if __name__ == "__main__":
    main()
