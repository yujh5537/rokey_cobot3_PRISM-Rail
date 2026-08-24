"""엣지케이스 테스트 (5일차) — 시연 시나리오 밖의 극한 상황 검증.
실행: python3 tests/test_edge_cases.py
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from rail_control_core.scenario import build, build_custom, PARAMS
from rail_control_core.fsm import OrderState, CapsuleState
from rail_control_core import topology as T

CONVOY_ROUTES = {"C01": "P0_OR2", "C02": "P0_OR2", "C03": "P0_OR2", "C04": "P0_OR2"}  # v3.5 OR2 집결


def _eff_cap(occ, cap, bunch=4):
    """R9 개정: 활성 P0(Code Crimson) 캡슐이 점유 중인 블록은 상한 개방분까지 정상."""
    from rail_control_core.fsm import OrderState
    if any(c.order is not None and c.order.prio == 0
           and c.order.state == OrderState.EN_ROUTE for c in occ):
        return max(cap, bunch)
    return cap


def _run_checked(eng, until=300.0):
    """완주까지 틱 + 매 틱 R8 안전망 불변식 검사. (성능 위해 2틱마다 검사)"""
    violations, k = [], 0
    while eng.t < until:
        eng.tick()
        k += 1
        if k % 2 == 0:
            for bid, (_, _, length, cap, oneway) in T.BLOCKS.items():
                occ = eng.occ[bid]
                if len(occ) > _eff_cap(occ, cap):
                    violations.append(f"t={eng.t:.2f} {bid} 용량 {len(occ)}/{cap}")
                if len({c.fwd for c in occ}) > 1:
                    violations.append(f"t={eng.t:.2f} {bid} 정면 대치")
                if oneway and any(not c.fwd for c in occ):
                    violations.append(f"t={eng.t:.2f} {bid} 역주행")
        if all(o.state == OrderState.DONE for o in eng.orders.values()):
            return eng, violations
    raise RuntimeError(f"교착/타임아웃 t={eng.t:.1f}: "
                       + str({c.cid: (c.block, c.state.value)
                              for c in eng.capsules.values()
                              if c.state != CapsuleState.REMOVED}))


# ── ① 오더 폭주: 전 오더가 1.5초 안에 동시 발령 ──────────────────────
BURST_DEFS = [
    ("O-1", 3, "P3_CSR", 0.5, ["C05"], 0.7),
    ("O-2", 2, "P2_ICU", 0.5, ["C06"], None),
    ("O-3", 1, "P1_ICU", 1.0, ["C07"], None),
    ("O-4", 0, None, 1.5, ["C01", "C02", "C03", "C04"], None),
]
BURST_STANDBY = {"C05": "P3_CSR", "C06": "P2_ICU", "C07": "P1_ICU"}


def test_burst_mode_b_completes_safely():
    """폭주에도 모드 B는 교착 없이 완주하고 안전망 위반이 없어야 한다."""
    eng = build_custom("B", BURST_DEFS, BURST_STANDBY, convoy=CONVOY_ROUTES)
    eng, v = _run_checked(eng)
    assert not v, v[:5]
    arr = {o.oid: o.arrive_t for o in eng.orders.values()}
    assert arr["O-3"] < arr["O-2"]          # 폭주 속에서도 P1 < P2 (PEDD)
    assert max(arr.values()) < 120


GRIDLOCK_DEFS = [  # v3.3 최종 좌표에서 A를 확정 교착시키는 간섭 구성 (P3 발령 24s, 스캔 검증)
    ("O-1", 3, "P3_CSR", 24.0, ["C05"], 0.7),
    ("O-2", 2, "P2_ICU", 1.0, ["C06"], None),
    ("O-3", 1, "P1_ICU", 2.0, ["C07"], None),
    ("O-4", 0, None, 8.0, ["C01", "C02", "C03", "C04"], None),
]


def test_gridlock_mode_a_detected_mode_b_survives():
    """[발표 논거] 같은 간섭 구성에서 모드 A(FCFS)는 정면 그리드락 —
    선점 없이는 양방향 단선에서 회피 불가능함을 증명하는 음성 테스트.
    관제는 20초 무진행 시 DEADLOCK 이벤트로 대치 당사자까지 통보하고,
    모드 B는 동일 구성을 대피 기동으로 완주한다."""
    eng = build_custom("A", GRIDLOCK_DEFS, BURST_STANDBY, convoy=CONVOY_ROUTES)
    for _ in range(int(120 * 30)):
        eng.tick()
        if any(e[1] == "DEADLOCK" for e in eng.events):
            break
    dl = [e for e in eng.events if e[1] == "DEADLOCK"]
    assert dl, "그리드락 미감지"
    assert "C05" in dl[0][3] and "C01" in dl[0][3]   # 대치 당사자 포함 통보
    eng_b = build_custom("B", GRIDLOCK_DEFS, BURST_STANDBY, convoy=CONVOY_ROUTES)
    eng_b, v = _run_checked(eng_b)
    assert not v, v[:5]


# ── ② EVAC_SPUR 강제: 2F->B1F 역방향 배송이 콘보이와 본선 대향 ──────
def test_evac_spur_forced():
    """B1F 본선을 동진하는 캡슐이 서진 콘보이 파도와 만나면 지선(SP-INJ)으로
    대피(EVAC_SPUR)했다가 콘보이 통과 후 왕복 복귀해 완주해야 한다."""
    defs = [
        ("O-X", 3, "X_ICU_PHM", 0.0, ["C05"], None),
        ("O-4", 0, None, 34.0, ["C01", "C02", "C03", "C04"], None),  # v3.3: 캡슐 BB-01 진입 직후 파도 (33~35 창 중앙, 스캔 검증)
    ]
    eng = build_custom("B", defs, {"C05": "X_ICU_PHM"}, convoy=CONVOY_ROUTES)
    eng, v = _run_checked(eng)
    assert not v, v[:5]
    names = [e[1] for e in eng.events]
    assert "EVAC_SPUR" in names, f"EVAC_SPUR 미발생: {sorted(set(names))}"
    spur_ev = next(e for e in eng.events if e[1] == "EVAC_SPUR")
    assert spur_ev[2] == "C05" and "SP-" in spur_ev[3]
    assert "RESUME" in names                # 대피 후 복귀까지 확인
    assert eng.orders["O-X"].state == OrderState.DONE


# ── ③ Code Crimson 수동 발동 타이밍 (브리지 계층) ────────────────────
def test_code_crimson_timing():
    from rail_control_core.bridge import Bridge
    # (a) 일시정지 중 발동 -> start 후 즉시 출동
    b = Bridge("B"); b.command("start")
    for _ in range(60):
        b.step()                             # t=2.0
    b.command("pause")
    ok, oid, _ = b.code_crimson("일시정지 중 발동")
    assert ok and oid == "O-4"
    b.command("start")
    ev = []
    for _ in range(30):
        ev += b.step()["events"]
    assert any(e["event"] == "ORDER_RELEASE" and e["subject"] == "O-4" for e in ev)
    # (b) 예약 자동 발령(8.0s) 이후 수동 발동 -> 거부
    b2 = Bridge("B"); b2.command("start")
    for _ in range(int(9 * 30)):
        b2.step()
    ok2, _, msg = b2.code_crimson()
    assert not ok2 and "이미 발령" in msg
    # (c) reset 후 재발동 가능
    b2.command("reset"); b2.command("start")
    for _ in range(15):
        b2.step()
    ok3, _, _ = b2.code_crimson()
    assert ok3


# ── ④ 폭주 시 도착 순서 결정론 ───────────────────────────────────────
def test_burst_determinism():
    def arr():
        eng = build_custom("B", BURST_DEFS, BURST_STANDBY, convoy=CONVOY_ROUTES)
        eng, _ = _run_checked(eng)
        return {o.oid: round(o.arrive_t, 2) for o in eng.orders.values()}
    assert arr() == arr()


if __name__ == "__main__":
    for fn in [test_burst_mode_b_completes_safely, test_gridlock_mode_a_detected_mode_b_survives,
               test_evac_spur_forced, test_code_crimson_timing, test_burst_determinism]:
        fn()
        print(f"PASS {fn.__name__}")
    print("엣지케이스 테스트 전부 통과")
