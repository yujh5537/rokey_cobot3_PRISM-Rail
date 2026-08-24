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

# ── 기준값 (2026-08-24, v3.6: O-1 멸균 공급 역전 + O-5 회수 추가 + R13 교행) ──
# 발령 O-1(공급 CSR->OR1)=2, O-2=1, O-3=2, O-4(P0)=8, O-5(회수 OR1->CSR)=17
# due(시연) P0=30,P1=40,P2=77(abs 78),P3=130 / 속도 평시1.0·최대2.0·샤프트0.5·곡선0.5
BASE_A = {"makespan": 71.17, "O-1": 36.17, "O-2": 48.43, "O-3": 37.60, "O-4": 71.17, "O-5": 54.77}
BASE_B = {"makespan": 85.77, "O-1": 36.17, "O-2": 76.60, "O-3": 37.60, "O-4": 67.07, "O-5": 85.77}


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
    """발표 장면이 시연 시나리오에서 실제 발생하는지 (v3.6 = 5장면).
    역방향 배우(O-5 회수)가 콘보이와 정면 조우하면서 장면 ③이 대피 레인 치환
    (EVAC_LANE)으로 복귀했고, 공급(O-1)과 회수(O-5)가 동측 가지에서 스쳐 지나가는
    신규 장면 ⑤ 교행(MEET_PASS, R13)이 추가됐다."""
    r = run("B")
    assert {"YIELD", "EVAC_LANE", "FINISH_ALLOWED", "MEET_PASS"} <= set(r["scene_events"])


def test_pedd_effect():
    """선점 효과: 모드 B에서 P0이 모드 A보다 빨라야 하고 P1은 나빠지지 않아야 한다.
    (v3.3부터 P1은 양 모드 공통 v_max 등급이라 A에서도 무간섭이면 동률 가능)"""
    a, b = run("A"), run("B")
    assert b["orders"]["O-4"]["arrive"] < a["orders"]["O-4"]["arrive"]  # P0 혈액
    assert b["orders"]["O-3"]["arrive"] <= a["orders"]["O-3"]["arrive"]  # P1 응급약품
    # 모드 B: 간섭을 겪은 오더들 사이에서 우선순위가 지연을 배분한다 (P1 < P2 < P3 회수).
    # O-1 공급은 콘보이와 동방향이라 무간섭 조기 완주(36.2s) — 순서 비교 대상이 아니다.
    # "선점은 필요할 때만 개입한다"가 이 예외의 의미이며, 결함이 아니다.
    ob = b["orders"]
    assert ob["O-3"]["arrive"] < ob["O-2"]["arrive"] < ob["O-5"]["arrive"]



def test_mock_core_timeline_matches_baseline():
    """mock_core 의 재생 타임라인 수치가 회귀 기준값과 어긋나지 않는지.

    mock_core 는 관제 노드가 죽었을 때의 플랜 B 라서, 수치가 낡으면 장애 상황에서
    심사 화면에 옛 값이 뜬다 (v3.3->v3.5 구간에 실제로 두 번 어긋났다).
    rclpy 를 임포트하지 않고 AST 로 TIMELINE 리터럴만 읽어 대조한다.
    """
    import ast
    path = os.path.join(os.path.dirname(__file__), "..", "..",
                        "rail_bridge", "rail_bridge", "mock_core_node.py")
    if not os.path.exists(path):
        return                                   # 브릿지 패키지 없이 코어만 쓰는 배포
    tree = ast.parse(open(path, encoding="utf-8").read(), filename=path)
    tl = next((ast.literal_eval(n.value) for n in tree.body
               if isinstance(n, ast.Assign)
               and any(getattr(t, "id", "") == "TIMELINE" for t in n.targets)), None)
    assert tl, "mock_core_node.py 에서 TIMELINE 을 찾지 못했습니다"
    detail = {ev: d for ev, _, d, *_ in tl}
    assert "MEET_PASS" in detail, "v3.6 교행 장면이 mock 타임라인에 없습니다"
    for ev, key in (("ORDER_ARRIVE", "O-4"), ("SIM_DONE", "makespan")):
        got = float(detail[ev].split("=")[1])
        assert abs(got - BASE_B[key]) <= TOL, \
            f"mock {ev}={got} != BASE_B[{key}]={BASE_B[key]} — mock_core 타임라인을 갱신하세요"


def _eff_cap(occ, cap, bunch=4):
    """R9 개정: 활성 P0(Code Crimson) 캡슐이 점유 중인 블록은 상한 개방분까지 정상."""
    from rail_control_core.fsm import OrderState
    if any(c.order is not None and c.order.prio == 0
           and c.order.state == OrderState.EN_ROUTE for c in occ):
        return max(cap, bunch)
    return cap


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
            if len(occ) > _eff_cap(occ, cap):
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


def test_xyz_rounding_at_source():
    """발행원 반올림: /capsule_pose 로 나가는 모든 좌표는 소수 4자리 이내.
    (실기 검증 중 발견된 3.9999999999999996 류 부동소수 잔재의 재발 방지)"""
    from rail_control_core.geometry import pose_to_xyz, dock_slot_xyz
    for bid, (_, _, length, _, _) in T.BLOCKS.items():
        for frac in (0.0, 0.333, 0.5, 0.777, 1.0):
            for fwd in (True, False):
                for v in pose_to_xyz(bid, length * frac, fwd):
                    assert v == round(v, 4), f"{bid} frac={frac} fwd={fwd}: {v}"
    for i in range(10):
        for v in dock_slot_xyz(i):
            assert v == round(v, 4)


def test_rta_recovery():
    """R11: P2가 선점 지연 후 RTA로 회복해 정시 도착, 회복 없으면(대조군) 지각."""
    import importlib
    from rail_control_core import scenario as S
    importlib.reload(S)
    eng = S.build("B"); eng.run()
    assert any(e[1] == "RTA_ENGAGED" for e in eng.events)
    o2 = eng.orders["O-2"]
    assert o2.arrive_t <= o2.due_t - 0.5          # 정시 (여유 포함)
    assert not any(e[1] == "PRIORITY_PROMOTED" for e in eng.events)  # 본 시나리오는 회복 가능 범위
    S.PARAMS["rta_slack_threshold"] = -1e9        # 회복 비활성 대조군
    eng2 = S.build("B"); eng2.run()
    assert eng2.orders["O-2"].arrive_t > o2.due_t # 회복 없으면 지각
    importlib.reload(S)


def test_priority_promotion():
    """R12: due가 물리 하한보다 빠듯하면 P2->P1 자동 승격으로 최대 노력 전환."""
    import importlib
    from rail_control_core import scenario as S
    importlib.reload(S)
    S.PARAMS["priority_due_sec"][2] = 55          # 물리적으로 회복 불가한 due
    eng = S.build("B"); eng.run()
    assert any(e[1] == "PRIORITY_PROMOTED" and e[2] == "O-2" for e in eng.events)
    assert eng.orders["O-2"].promoted and eng.orders["O-2"].prio == 1
    importlib.reload(S)


def test_determinism():
    r1, r2 = run("B"), run("B")
    assert r1["makespan"] == r2["makespan"]
    assert {k: v["arrive"] for k, v in r1["orders"].items()} == \
           {k: v["arrive"] for k, v in r2["orders"].items()}


if __name__ == "__main__":
    for fn in [test_topology_integrity, test_mode_a_baseline, test_mode_b_baseline,
               test_mode_b_scenes_present, test_pedd_effect,
               test_safety_invariants_mode_a, test_safety_invariants_mode_b,
               test_xyz_rounding_at_source, test_rta_recovery, test_priority_promotion,
               test_block_snapshot_shape, test_determinism]:
        fn()
        print(f"PASS {fn.__name__}")
    print("모든 회귀 테스트 통과")
