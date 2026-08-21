"""회귀 테스트 — 2일차 측정 기준값(토폴로지 v3.1, 시연 시나리오 v2) 고정.
명세서 §6-2 표와 동일한 값 — 한쪽만 고치지 말 것.
실행: python3 -m pytest test/ -q   (또는 python3 test/test_regression.py)
블록 길이/파라미터 변경 시 기준값 재측정 후 이 파일 갱신 (커밋 메시지에 명시).
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from rail_control_core import topology as T
from rail_control_core.scenario import run

TOL = 0.2  # 허용 오차(초): 30Hz 틱 양자화 감안

# ── 기준값 (2026-08-21 재측정, 토폴로지 v3.2 씬 실측 길이 / 발령 P2=1,P1=2,P0=8,P3=10 / P3 speed 0.7) ──
BASE_A = {"makespan": 51.70, "O-1": 41.33, "O-2": 24.73, "O-3": 30.73, "O-4": 51.70}
BASE_B = {"makespan": 64.23, "O-1": 64.23, "O-2": 54.60, "O-3": 26.37, "O-4": 47.60}


def _close(a, b):
    assert abs(a - b) <= TOL, f"{a} != {b} (±{TOL})"


def test_topology_integrity():
    assert T.validate() == []


def test_mode_a_baseline():
    r = run("A")
    _close(r["makespan"], BASE_A["makespan"])
    for oid in ("O-1", "O-2", "O-3", "O-4"):
        _close(r["orders"][oid]["arrive"], BASE_A[oid])


def test_mode_b_baseline():
    r = run("B")
    _close(r["makespan"], BASE_B["makespan"])
    for oid in ("O-1", "O-2", "O-3", "O-4"):
        _close(r["orders"][oid]["arrive"], BASE_B[oid])


def test_mode_b_scenes_present():
    """발표 4장면 중 3장면이 시연 시나리오에서 실제 발생하는지.
    v3.2 실측 길이 반영 후 대피 레인 치환(EVAC_LANE)이 발생 — 발표 장면 ③의 핵심.
    [커버리지 공백] EVAC_SPUR(지선 회피)는 이제 미발생 — 5일차 엣지케이스에서 별도 검증."""
    r = run("B")
    assert {"YIELD", "EVAC_LANE", "FINISH_ALLOWED"} <= set(r["scene_events"])


def test_pedd_effect():
    """선점 효과: 모드 B에서 P0/P1이 모드 A보다 빨라야 한다."""
    a, b = run("A"), run("B")
    assert b["orders"]["O-4"]["arrive"] < a["orders"]["O-4"]["arrive"]  # P0 혈액
    assert b["orders"]["O-3"]["arrive"] < a["orders"]["O-3"]["arrive"]  # P1 응급약품
    # 모드 B 도착 순서 = 우선순위 순 (P1 < P2 < P3; P0는 4대 콘보이 특성상 P2와 근접 도착)
    ob = b["orders"]
    assert ob["O-3"]["arrive"] < ob["O-2"]["arrive"] < ob["O-1"]["arrive"]



def _run_with_invariants(mode: str):
    """매 틱마다 R8 안전망 불변식을 검사하며 시나리오를 완주시킨다."""
    from rail_control_core.scenario import build
    from rail_control_core.fsm import CapsuleState
    from rail_control_core.engine import DT
    from rail_control_core import topology as T
    eng = build(mode)
    violations = []
    guard = 0
    while guard < 30000:
        guard += 1
        eng.tick()
        for bid, (_, _, length, cap, oneway) in T.BLOCKS.items():
            occ = eng.occ[bid]
            if len(occ) > cap:
                violations.append(f"t={eng.t:.2f} {bid} 용량 초과 {len(occ)}/{cap}")
            dirs = {c.fwd for c in occ}
            if len(dirs) > 1:
                violations.append(f"t={eng.t:.2f} {bid} 정면 대치 {[c.cid for c in occ]}")
            if oneway and any(not c.fwd for c in occ):
                violations.append(f"t={eng.t:.2f} {bid} 단방향 역주행")
            for i in range(1, len(occ)):
                if occ[i - 1].pos - occ[i].pos < -1e-6:
                    violations.append(f"t={eng.t:.2f} {bid} 추월/추돌 {occ[i].cid}")
            for c in occ:
                if c.pos > length + 1e-6 or c.pos < -1e-6:
                    violations.append(f"t={eng.t:.2f} {bid} 구간 이탈 {c.cid} pos={c.pos:.2f}")
        if all(o.state.value == "DONE" for o in eng.orders.values()):
            break
    return eng, violations


def test_safety_invariants_mode_a():
    """R8: 모드 A(안전망만)에서도 충돌·역주행·용량 초과가 없어야 한다."""
    eng, v = _run_with_invariants("A")
    assert not v, v[:5]


def test_safety_invariants_mode_b():
    """R8: 모드 B(선점 활성) 전 구간에서 안전망 불변식 유지."""
    eng, v = _run_with_invariants("B")
    assert not v, v[:5]


def test_block_snapshot_shape():
    """/block_state 토픽 페이로드 원형의 스키마·정합성 (STEP 3 인터페이스 계약)."""
    from rail_control_core.scenario import build
    eng = build("B")
    for _ in range(600):   # t=20s
        eng.tick()
    snap = eng.block_snapshot()
    assert len(snap) == len(T.BLOCKS)
    for bid, d in snap.items():
        assert set(d) == {"state", "occupancy", "capacity", "locked",
                          "corridor", "dir", "capsules"}
        assert d["state"] in ("FREE", "RESERVED", "OCCUPIED")
        assert d["occupancy"] <= d["capacity"]
        assert (d["state"] == "OCCUPIED") == (d["occupancy"] > 0)
        assert len(d["capsules"]) == d["occupancy"]


def test_determinism():
    r1, r2 = run("B"), run("B")
    assert r1["makespan"] == r2["makespan"]
    assert {k: v["arrive"] for k, v in r1["orders"].items()} == \
           {k: v["arrive"] for k, v in r2["orders"].items()}


if __name__ == "__main__":
    for fn in [test_topology_integrity, test_mode_a_baseline, test_mode_b_baseline,
               test_mode_b_scenes_present, test_pedd_effect,
               test_safety_invariants_mode_a, test_safety_invariants_mode_b,
               test_block_snapshot_shape, test_determinism]:
        fn()
        print(f"PASS {fn.__name__}")
    print("모든 회귀 테스트 통과")
