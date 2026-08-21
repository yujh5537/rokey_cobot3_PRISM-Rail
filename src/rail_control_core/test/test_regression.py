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
# 기준값 (2026-08-20 측정, 파라미터 잠금)
#   정의: P0(O-4, Code Red) 오더의 '인계 완료 시각' T+초
#   발표자료 기준값 54.3 / 44.2 와 각각 0.5초 이내로 일치
# ---------------------------------------------------------------------
BASELINE_A = 53.9
BASELINE_B = 44.7
TOL = 0.5


def p0_finish(mode: str) -> float:
    eng = build_engine(mode)
    eng.run_until_done()
    return next(o.finish_s for o in eng.orders.values() if o.priority == 0)


# =====================================================================
# 1) 기준값 회귀 — 이게 깨지면 배차 로직이 바뀐 것입니다
# =====================================================================
@pytest.mark.parametrize("mode,expected", [("A", BASELINE_A), ("B", BASELINE_B)])
def test_p0_finish_baseline(mode, expected):
    got = p0_finish(mode)
    assert abs(got - expected) <= TOL, (
        f"모드 {mode} P0 완료 시각이 바뀌었습니다: 기준 {expected}s → 실측 {got:.1f}s"
    )


def test_mode_b_is_faster_for_p0():
    """선점의 존재 이유. 이게 깨지면 프로젝트의 주장 자체가 무너집니다."""
    a, b = p0_finish("A"), p0_finish("B")
    assert b < a, f"모드 B({b:.1f}s)가 모드 A({a:.1f}s)보다 빨라야 합니다"
    assert (a - b) >= 5.0, "P0 개선폭이 5초 미만이면 시연 효과가 약합니다"


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
