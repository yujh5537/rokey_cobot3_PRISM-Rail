"""브리지 테스트 — ROS 없이 노드 로직 검증 (STEP 3 핵심 검증).
실행: python3 -m pytest test/ -q   (또는 python3 test/test_bridge.py)
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from rail_control_core import topology as T
from rail_control_core.bridge import Bridge
# 기준값은 test_regression.py 를 단일 출처로 삼는다. 여기에 숫자를 베껴 두면
# 토폴로지가 바뀔 때마다 한쪽만 갱신되어 어긋난다 (실제로 v3.2 에서 겪음).
from test_regression import BASE_B, TOL

TICKS_PER_SEC = 30


def _run_secs(b: Bridge, secs: float) -> list[dict]:
    events = []
    for _ in range(int(secs * TICKS_PER_SEC)):
        events += b.step()["events"]
    return events


def test_start_pause_reset_status():
    b = Bridge("B")
    assert b.step()["events"] == [] and b.step()["sim_t"] == 0.0  # 정지 상태
    ok, _ = b.command("start"); assert ok
    _run_secs(b, 1.0)
    assert abs(b.eng.t - 1.0) < 0.05
    ok, msg = b.command("pause"); assert ok
    t = b.eng.t
    _run_secs(b, 0.5)
    assert b.eng.t == t                       # 일시정지 중 시간 정지
    ok, msg = b.command("status"); assert ok and "mode=B" in msg
    ok, _ = b.command("reset"); assert ok and b.eng.t == 0.0
    ok, msg = b.command("warp"); assert not ok  # 알 수 없는 명령 거부


def test_capsule_payload_schema():
    b = Bridge("B"); b.command("start")
    out = b.step()
    caps = out["capsules"]
    assert len(caps) == 10
    for c in caps:
        # v3.2: 위치 진실 소스가 코어가 되면서 x/y/z 가 추가됨 (C 매핑표 §1)
        # v3.8.2: prio 추가 (씬 캡슐 색 연출용, B 요청).
        #   필드 추가는 하위 호환이라 기존 구독자는 무시하면 그만이지만,
        #   /capsule_pose 는 B(씬)·D(UI) 공용 계약이므로 스키마를 여기서 고정해
        #   변경이 조용히 지나가지 않게 한다.
        assert set(c) == {"capsule_id", "block_id", "pos_m", "forward",
                          "state", "order_id", "prio", "x", "y", "z"}
        assert all(isinstance(c[k], (int, float)) for k in ("x", "y", "z"))
    # 콘보이 4대는 대기열 BB-08 에서 시작
    for c in caps:
        assert c["prio"] in (0, 1, 2, 3, 9), f"{c['capsule_id']}.prio = {c['prio']!r}"
        # 오더가 없으면 미배정(9), 있으면 P0~P3
        assert (c["prio"] == 9) == (c["order_id"] == ""), \
            f"{c['capsule_id']}: order_id={c['order_id']!r} 인데 prio={c['prio']}"

    q = [c for c in caps if c["capsule_id"] in ("C01", "C02", "C03", "C04")]
    assert all(c["block_id"] == "BB-08" and c["state"] == "QUEUED" for c in q)
    # 디포 유휴 캡슐은 레일이 아니라 측면 도크 슬롯 좌표를 받는다
    docked = [c for c in caps if c["state"] == "DOCKED"]
    assert docked and all((c["x"], c["y"]) in [tuple(s) for s in T.DOCK_SLOTS]
                          for c in docked)


def test_block_state_published_on_change_only():
    b = Bridge("B")
    out1 = b.step()
    assert out1["blocks"] is not None          # 최초 1회 전체 스냅샷
    out2 = b.step()
    assert out2["blocks"] is None              # 정지 상태 = 변화 없음 = 미발행
    b.command("start")
    changed = sum(1 for _ in range(300) if b.step()["blocks"] is not None)
    assert 0 < changed < 300                   # 변화 시에만 발행 (매 틱 발행 아님)


def test_code_crimson_immediate_release():
    b = Bridge("B"); b.command("start")
    _run_secs(b, 1.0)                          # t=1.0 (예약 발령 8.0s 이전)
    ok, oid, msg = b.code_crimson("훈련 상황")
    assert ok and oid == "O-4"
    ev = _run_secs(b, 0.2)
    names = [e["event"] for e in ev]
    assert "ORDER_RELEASE" in names            # 즉시 발령됨
    ok2, _, msg2 = b.code_crimson()
    assert not ok2 and "이미 발령" in msg2      # 중복 발동 거부


def test_full_run_to_sim_done():
    b = Bridge("B"); b.command("start")
    ev = _run_secs(b, 120)
    names = [e["event"] for e in ev]
    assert "SIM_DONE" in names
    assert not b.running                       # 완료 후 자동 정지
    done = next(e for e in ev if e["event"] == "SIM_DONE")
    assert abs(float(done["detail"].split("=")[1]) - BASE_B["makespan"]) < TOL


def test_mode_switch_resets():
    b = Bridge("B"); b.command("start"); _run_secs(b, 2)
    ok, msg = b.command("mode_a")
    assert ok and b.mode == "A" and b.eng.t == 0.0 and not b.running


def test_payload_rounding_invariant():
    """발행 페이로드의 모든 좌표·수치가 소수 4자리 이내여야 한다.

    부동소수 잔재(예: z=4.366666666666666, y=3.9999999999999996)가 그대로
    토픽에 실려 나간 사고가 두 번 있었다. 발행원에서 막는다.
    """
    b = Bridge("B"); b.command("start")
    checked = 0
    for _ in range(400):
        out = b.step()
        for c in out["capsules"]:
            for k in ("x", "y", "z", "pos_m"):
                v = c[k]
                assert round(v, 4) == v, f"{c['capsule_id']}.{k} = {v!r} (소수 4자리 초과)"
                checked += 1
    assert checked > 0


if __name__ == "__main__":
    for fn in [test_start_pause_reset_status, test_capsule_payload_schema,
               test_payload_rounding_invariant,
               test_block_state_published_on_change_only,
               test_code_crimson_immediate_release,
               test_full_run_to_sim_done, test_mode_switch_resets]:
        fn()
        print(f"PASS {fn.__name__}")
    print("브리지 테스트 전부 통과")
