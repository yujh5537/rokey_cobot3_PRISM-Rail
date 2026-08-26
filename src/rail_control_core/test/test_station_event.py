"""v3.8: Isaac OR 스테이션 이벤트 게이트 회귀 테스트.

왜 필요한가 — 기존 31종은 게이트 OFF 경로만 검증한다.
시연에서 실제로 쓰이는 건 ON 경로이므로, 다음 4가지가 회귀로 고정되어야 한다:
  ① OFF 기본값이 기존 동작을 바꾸지 않을 것 (발표 기준값 A 73.70 / B 94.47 보존)
  ② ON 이면 DOOR_CLOSED 전에는 절대 출발하지 않을 것 (물리적 모순 방지)
  ③ Isaac 무응답이어도 폴백으로 완주할 것 (시연 안전망)
  ④ OR2 콘보이(Code Crimson)는 게이트 영향을 받지 않을 것 (대상 한정)
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from rail_control_core import scenario as S

DWELL = 2.0
TIMEOUT = 30.0
TARGET = "C05"          # O-1 멸균 물품 공급, ST-OR1 (scenario.py 기준)
CONVOY = "C01"          # Code Crimson 콘보이, OR2


def _build(enable, timeout=TIMEOUT):
    eng = S.build("B")
    eng.p["station_event_enabled"] = enable
    eng.p["door_event_timeout_sec"] = timeout
    return eng


def _run_until_blue(eng, cid, send_after=None, limit=250.0):
    """cid 캡슐이 BLUE 승인될 때까지 진행. send_after 초 뒤 DOOR_CLOSED 주입."""
    sent = False
    while eng.t < limit:
        eng.tick()
        c = eng.capsules.get(cid)
        if c is None:
            continue
        if send_after is not None and not sent and c.svc_wait_until > 0 \
           and eng.t >= c.svc_wait_until + send_after:
            eng.on_station_event(cid, "DOOR_CLOSED")
            sent = True
        if c.svc_blue:
            return eng.t, c.svc_wait_until
    return None, None


def test_gate_off_is_time_based():
    """① OFF 기본값 — WORK 대기 후 dwell 만큼만 지나면 승인 (기존 동작)"""
    t, wait = _run_until_blue(_build(False), TARGET)
    assert t is not None, "OFF 인데 BLUE 승인이 나오지 않았다"
    assert abs(t - wait) < 0.2, f"OFF 는 dwell 기반이어야 한다 (t={t}, wait={wait})"


def test_gate_on_waits_for_door_closed():
    """② ON — 이벤트가 없으면 dwell 이 지나도 출발하지 않는다"""
    eng = _build(True)
    while eng.t < 250.0:
        eng.tick()
        c = eng.capsules.get(TARGET)
        if c and c.svc_wait_until > 0 and eng.t > c.svc_wait_until + 5.0:
            assert not c.svc_blue, "DOOR_CLOSED 없이 출발이 승인됐다"
            return
    assert False, "WORK 대기 상태에 도달하지 못했다"


def test_gate_opens_on_door_closed():
    """② ON — DOOR_CLOSED 수신 시점에 승인되며, 폴백 시각보다 훨씬 빠르다"""
    t, wait = _run_until_blue(_build(True), TARGET, send_after=5.0)
    assert t is not None, "이벤트를 보냈는데 승인되지 않았다"
    assert t < wait + TIMEOUT - 1.0, f"폴백이 아니라 이벤트로 열려야 한다 (t={t})"


def test_gate_fallback_on_timeout():
    """③ Isaac 무응답 — timeout 후 폴백 승인 (시연 완주 보장)"""
    t, wait = _run_until_blue(_build(True), TARGET)
    assert t is not None, "폴백이 동작하지 않아 시뮬이 멈췄다"
    assert abs(t - (wait + TIMEOUT)) < 0.3, f"폴백 시각 불일치 (t={t}, 기대={wait + TIMEOUT})"


def test_convoy_unaffected_by_gate():
    """④ OR2 콘보이는 게이트 대상이 아니다 — 게이트 ON 이어도 dwell 기반"""
    t, wait = _run_until_blue(_build(True), CONVOY)
    if t is None:
        return                      # 콘보이가 서비스 딥에 안 들어가는 시나리오면 무관
    assert abs(t - wait) < 0.3, f"콘보이가 이벤트 게이트에 걸렸다 (t={t}, wait={wait})"


def test_label_mismatch_is_logged():
    """라벨 캡슐ID 불일치는 LABEL_MISMATCH 로 남아야 한다 (오적재 검출)"""
    eng = _build(True)
    for _ in range(50):
        eng.tick()
    eng.on_station_event(TARGET, "LABEL_READ",
                         {"ok": True, "label_capsule": "C99",
                          "order_id": "O-1", "delivery_add": "ST-OR1"})
    # 이벤트 로그는 (sim_t, event, subject, detail) 튜플 형식
    evs = [e[1] for e in eng.events]
    assert "LABEL_MISMATCH" in evs, "LABEL_MISMATCH 이벤트가 로그에 없다"
    assert eng.capsules[TARGET].svc_label.get("match") is False, \
        "svc_label.match 가 False 로 표시되지 않았다"


def test_label_match_ok():
    """라벨 캡슐ID 일치 시 match=True"""
    eng = _build(True)
    for _ in range(50):
        eng.tick()
    eng.on_station_event(TARGET, "LABEL_READ",
                         {"ok": True, "label_capsule": TARGET,
                          "order_id": "O-1", "delivery_add": "ST-OR1"})
    assert eng.capsules[TARGET].svc_label.get("match") is True
