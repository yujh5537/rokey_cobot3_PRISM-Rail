"""시연 시나리오 '동시다발 오더와 계단식 선점' v2 — 토폴로지 v3.1 / 명세서 §6-1 발령표와 동기화.
가정: P1~P3 담당 캡슐은 출발 스테이션 선배치(STANDBY), 콘보이 4대는 대기열(QUEUED).
"""
from .engine import Engine
from .fsm import Order, Capsule, CapsuleState
from . import topology as T

PARAMS = {
    "speed_default": 1.5,
    "speed_shaft": 1.0,
    "pitch": 0.9,
    "yield_window_sec": 15.0,
    "unload_sec": 2.0,
    "stall_timeout_sec": 20.0,
    "priority_due_sec": {0: 60, 1: 180, 2: 600, 3: 1800},
}

# 오더: (id, prio, route, release_t, capsule_ids)
# (id, prio, route, release_t, capsules, speed_cap)
ORDER_DEFS = [
    ("O-1", 3, "P3_CSR", 10.0, ["C05"], 0.7),   # 오염 기구: 저속 운송 규정 0.7m/s
    ("O-2", 2, "P2_ICU", 1.0, ["C06"], None),
    ("O-3", 1, "P1_ICU", 2.0, ["C07"], None),
    ("O-4", 0, None, 8.0, ["C01", "C02", "C03", "C04"], None),  # Code Crimson 콘보이
]
CONVOY_ROUTES = {"C01": "P0_OR1", "C02": "P0_OR1", "C03": "P0_OR2", "C04": "P0_OR2"}


def build(mode: str) -> Engine:
    eng = Engine(PARAMS, mode)
    for oid, prio, route, rel, cids, spd in ORDER_DEFS:
        due = rel + PARAMS["priority_due_sec"][prio]
        eng.orders[oid] = Order(oid, prio, route or "convoy", rel, due, cids, speed=spd)
    # 선배치 캡슐 (출발 스테이션 대기)
    standby = {"C05": "P3_CSR", "C06": "P2_ICU", "C07": "P1_ICU"}
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
    # 디포 유휴 3대 (점유만)
    for cid in ["C08", "C09", "C10"]:
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
                                    & {"YIELD", "EVAC_LANE", "EVAC_SPUR", "FINISH_ALLOWED"})
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
