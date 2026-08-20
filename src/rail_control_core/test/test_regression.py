"""
test_regression.py — 2일차 산출물: 회귀 테스트.

회귀 테스트란?
--------------
"코드를 고쳤는데 예전에 잘 되던 게 망가지지 않았는지" 확인하는 테스트입니다.
관제 로직은 조건문이 얽혀 있어서, 선점 규칙 한 줄만 바꿔도 엉뚱한 데가 깨집니다.
그래서 **로직을 고칠 때마다** 아래를 돌리세요. 10초면 끝납니다.

    cd ~/cobot3_ws/src/rail_control_core
    python3 -m pytest test/ -v

ROS를 띄우지 않아도 됩니다. 엔진이 순수 파이썬이기 때문입니다.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rail_control_core.engine import build_engine          # noqa: E402
from rail_control_core.fsm import BlockState, OrderState   # noqa: E402

# ---------------------------------------------------------------------
# 기준 KPI — ⚠️ 팀 미확정 상태
#
#   config/params.yaml 의 baseline_kpi / baseline_mode_a / baseline_mode_b 를
#   바꾸면 이 테스트가 자동으로 그 지표를 기준으로 돕니다.
#   현재 잠정값: p0_finish_s (P0 인계 완료 시각)
#     - 발표자료의 54.3 / 44.2 와 0.5초 이내로 일치하는 유일한 후보이기 때문
#     - 팀 확정 후 tools/kpi_report.py 로 값을 다시 뽑아 params.yaml 을 갱신하세요
# ---------------------------------------------------------------------
import yaml  # noqa: E402

CFG = Path(__file__).resolve().parent.parent / "config"
_P = yaml.safe_load((CFG / "params.yaml").read_text(
    encoding="utf-8"))["control_core_node"]["ros__parameters"]

BASELINE_KPI = _P.get("baseline_kpi", "p0_finish_s")
BASELINE_A = float(_P.get("baseline_mode_a", 53.9))
BASELINE_B = float(_P.get("baseline_mode_b", 44.7))
TOL = float(_P.get("baseline_tolerance", 0.5))


def measure(mode: str, kpi: str = None) -> float:
    """지정한 KPI 지표를 하나 뽑습니다."""
    kpi = kpi or BASELINE_KPI
    eng = build_engine(mode)
    eng.run_until_done(400.0)
    p0 = next(o for o in eng.orders.values() if o.priority == 0)
    table = {
        "p0_finish_s": p0.finish_s,
        "p0_lead_time_s": p0.lead_time_s,
        "makespan_s": eng.makespan(),
        "avg_lead_time_s": eng.kpi()["avg_lead_time_s"],
    }
    if kpi not in table:
        raise KeyError(f"알 수 없는 baseline_kpi: {kpi} (선택지: {list(table)})")
    return float(table[kpi])


def p0_finish(mode: str) -> float:
    return measure(mode, "p0_finish_s")


# =====================================================================
# 1) 기준값 회귀 — 이게 깨지면 배차 로직이 바뀐 것입니다
# =====================================================================
@pytest.mark.parametrize("mode,expected", [("A", BASELINE_A), ("B", BASELINE_B)])
def test_baseline_kpi(mode, expected):
    got = measure(mode)
    assert abs(got - expected) <= TOL, (
        f"모드 {mode}의 {BASELINE_KPI}가 바뀌었습니다: "
        f"기준 {expected}s → 실측 {got:.1f}s"
    )


def test_mode_b_is_faster_for_p0():
    """선점의 존재 이유. 이게 깨지면 프로젝트의 주장 자체가 무너집니다."""
    a, b = p0_finish("A"), p0_finish("B")
    assert b < a, f"모드 B({b:.1f}s)가 모드 A({a:.1f}s)보다 빨라야 합니다"
    assert (a - b) >= 5.0, "P0 개선폭이 5초 미만이면 시연 효과가 약합니다"


def test_p0_meets_deadline_only_in_mode_b():
    """모드 A에서는 P0가 마감을 넘기고, 모드 B에서는 지킨다 — 발표의 핵심 대비."""
    a_over = build_engine("A"); a_over.run_until_done(400.0)
    b_over = build_engine("B"); b_over.run_until_done(400.0)
    a_p0 = next(o for o in a_over.orders.values() if o.priority == 0)
    b_p0 = next(o for o in b_over.orders.values() if o.priority == 0)
    assert a_p0.finish_s > a_p0.due_s, "모드 A에서 P0가 마감을 초과해야 대비가 성립합니다"
    assert b_p0.finish_s <= b_p0.due_s, "모드 B에서는 P0가 마감을 지켜야 합니다"


# =====================================================================
# 2) 4가지 핵심 장면 — 발표 시연이 실제로 재현되는지
# =====================================================================
@pytest.fixture(scope="module")
def mode_b_actions() -> list[dict]:
    eng = build_engine("B")
    acts = []
    while eng.now < 300 and not eng.all_done():
        for ev in eng.step():
            if ev.kind == "preempt":
                acts.append({"t": ev.t, **ev.payload})
    return acts


@pytest.mark.parametrize("action,scene", [
    ("YIELD_WAIT",  "① P1이 P2를 선점 — 진입 전 양보"),
    ("RUN_THROUGH", "② P0의 완주 허용 판단 — 역주행 불가"),
    ("EVACUATE",    "③ P3의 물리적 대피 — 지선 회피"),
    ("RESUME",      "④ 자동 복구 — 하위 화물 본선 복귀"),
])
def test_scene_occurs(mode_b_actions, action, scene):
    assert any(a["action"] == action for a in mode_b_actions), f"장면 누락: {scene}"


def test_shaft_never_evacuates(mode_b_actions):
    """수직 쉬프트(B04)는 단선이라 중간 이탈이 불가능합니다."""
    bad = [a for a in mode_b_actions
           if a["action"] == "EVACUATE" and a.get("block") == "B04"]
    assert not bad, f"수직 쉬프트에서 대피가 발생했습니다: {bad}"


# =====================================================================
# 3) 안전 불변식(Invariant) — 절대 깨지면 안 되는 것들
# =====================================================================
@pytest.mark.parametrize("mode", ["A", "B"])
def test_no_block_double_occupancy(mode):
    """한 블록에 캡슐 2대 = 충돌. 매 틱 검사합니다."""
    eng = build_engine(mode)
    while eng.now < 300 and not eng.all_done():
        eng.step()
        inside = [c.cur_block for c in eng.capsules.values() if c.cur_block]
        assert len(inside) == len(set(inside)), (
            f"T+{eng.now:.1f}s 블록 중복 점유: {inside}"
        )


@pytest.mark.parametrize("mode", ["A", "B"])
def test_all_orders_delivered(mode):
    """교착(deadlock)이 나면 여기서 걸립니다."""
    eng = build_engine(mode)
    eng.run_until_done(300.0)
    stuck = [o.oid for o in eng.orders.values() if o.state != OrderState.DELIVERED]
    assert not stuck, f"완료되지 않은 오더(교착 의심): {stuck}"


@pytest.mark.parametrize("mode", ["A", "B"])
def test_blocks_released_at_end(mode):
    eng = build_engine(mode)
    eng.run_until_done(300.0)
    held = [b for b, s in eng.block_state.items() if s != BlockState.FREE]
    assert not held, f"종료 후에도 점유 중인 블록(자원 누수): {held}"


def test_dirty_cargo_never_uses_clean_only_path():
    """오염 기구 동선 분리 — 금지 간선(Forbidden Edge)이 실제로 작동하는지."""
    eng = build_engine("B")
    path = eng.topo.find_path("N4", "N6", "clean")
    assert "B07" not in path, "청결 화물이 오염 전용 구간(B07)에 배정되었습니다"


def test_topology_shape():
    """B·C·D가 참조하는 계약. 임의로 바뀌면 안 됩니다."""
    eng = build_engine("B")
    assert len(eng.topo.nodes) == 10, "노드는 10개여야 합니다"
    assert len(eng.topo.blocks) == 8, "블록은 8개여야 합니다"
    assert eng.topo.siding_of("J2") == "E1", "J2의 대피 지선은 E1이어야 합니다"


# =====================================================================
# 4) 시연 서비스 — /code_red 버튼이 실제로 동작하는가
# =====================================================================
def test_code_red_order_gets_delivered():
    """시연 중 /code_red 로 들어온 오더에 전용 캡슐이 배치되어 완주해야 합니다.

    과거 버그: 캡슐 ID를 order_id 의 마지막 조각으로 만들어서
    "CR-1" → "C-1" 이 기존 O-1 캡슐과 충돌 → 배차 불가 → QUEUED 무한 대기.
    """
    from rail_control_core.fsm import Order   # noqa: PLC0415

    eng = build_engine("B")
    while eng.now < 20.0:
        eng.step()

    eng.add_order(Order(
        oid="CR-1", priority=0, item="응급 수혈혈액(Code Red)",
        origin="N1", dest="N4",
        release_s=eng.now, due_s=eng.now + 25.0,
        cargo_class="clean", code_red=True,
    ))
    eng.run_until_done(300.0)

    cr = eng.orders["CR-1"]
    assert cr.assigned_capsule is not None, "Code Red 오더에 캡슐이 배차되지 않았습니다"
    assert cr.assigned_capsule not in ("C-1", "C-2", "C-3", "C-4"), (
        f"기존 캡슐({cr.assigned_capsule})과 충돌했습니다"
    )
    assert cr.state == OrderState.DELIVERED, (
        f"Code Red 오더가 완료되지 않았습니다: {cr.state.value}"
    )


def test_capsule_ids_stable_for_main_scenario():
    """메인 시나리오의 캡슐 ID는 C-1~C-4 로 고정 (B·C·D 가 참조하는 계약)."""
    eng = build_engine("B")
    assert sorted(eng.capsules) == ["C-1", "C-2", "C-3", "C-4"]
