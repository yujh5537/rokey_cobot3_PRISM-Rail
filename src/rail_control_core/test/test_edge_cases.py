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
BURST_DEFS = [   # v3.6: 5오더 폭주 (전부 1.5초 내)
    ("O-1", 3, "P3_OR1", 0.5, ["C05"], 0.7),
    ("O-5", 3, "P3_CSR", 0.5, ["C08"], 0.7),
    ("O-2", 2, "P2_ICU", 0.5, ["C06"], None),
    ("O-3", 1, "P1_ICU", 1.0, ["C07"], None),
    ("O-4", 0, None, 1.5, ["C01", "C02", "C03", "C04"], None),
]
# C08 이 선배치로 빠지므로 build_custom 의 idle 기본값에서 반드시 제외할 것 (이중 배치 방지)
BURST_STANDBY = {"C05": "P3_OR1", "C08": "P3_CSR", "C06": "P2_ICU", "C07": "P1_ICU"}
BURST_IDLE = ("C09", "C10")


def test_burst_mode_b_completes_safely():
    """폭주에도 모드 B는 교착 없이 완주하고 안전망 위반이 없어야 한다."""
    eng = build_custom("B", BURST_DEFS, BURST_STANDBY, convoy=CONVOY_ROUTES,
                       idle=BURST_IDLE)
    eng, v = _run_checked(eng)
    assert not v, v[:5]
    arr = {o.oid: o.arrive_t for o in eng.orders.values()}
    assert arr["O-3"] < arr["O-2"]          # 폭주 속에서도 P1 < P2 (PEDD)
    assert max(arr.values()) < 120


def test_meet_pass_off_deadlocks_on_resolves():
    """[발표 3부] 교행 규칙(R13)을 끄면 공급(O-1)·회수(O-5)가 동측 가지에서 정면 교착 —
    선점(모드 B)으로도 못 푼다(파도 밖 P3끼리의 대치라 선점이 개입할 여지가 없음).
    관제는 무진행 20초를 DEADLOCK 으로 감지·통보한다.
    R13 을 켜면 동일 구성이 양 모드 모두 완주 —
    "교착은 스케줄링이 아니라 교통 규칙 계층이 푼다"의 음성/양성 대조."""
    import importlib
    from rail_control_core import scenario as S
    for mode in ("A", "B"):
        importlib.reload(S)
        S.PARAMS["meet_pass_enabled"] = False
        eng = S.build(mode)
        try:
            eng.run(until=140)
            assert False, f"{mode}: R13 OFF 인데 완주"
        except RuntimeError:
            pass
        dl = [x for x in eng.events if x[1] == "DEADLOCK"]
        assert dl and "C05" in dl[0][3] and "C08" in dl[0][3], f"{mode}: 감지 실패"
    importlib.reload(S)                      # 토글 원복 (기본 True)
    eng = S.build("A")
    eng.run(until=170)                       # ON: 양 모드 완주 — A 로 대표 확인
    assert all(o.state.value == "DONE" for o in eng.orders.values())


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
        eng = build_custom("B", BURST_DEFS, BURST_STANDBY, convoy=CONVOY_ROUTES,
                           idle=BURST_IDLE)
        eng, _ = _run_checked(eng)
        return {o.oid: round(o.arrive_t, 2) for o in eng.orders.values()}
    assert arr() == arr()


if __name__ == "__main__":
    for fn in [test_burst_mode_b_completes_safely, test_meet_pass_off_deadlocks_on_resolves,
               test_evac_spur_forced, test_code_crimson_timing, test_burst_determinism]:
        fn()
        print(f"PASS {fn.__name__}")
    print("엣지케이스 테스트 전부 통과")
