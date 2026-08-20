"""
engine.py — 관제 코어의 두뇌. ROS2에 전혀 의존하지 않는 순수 파이썬 엔진입니다.

왜 ROS와 분리하나요?
--------------------
ROS 노드 안에 로직을 다 넣으면 테스트할 때마다 ROS를 띄워야 해서 지옥이 됩니다.
엔진을 순수 파이썬으로 빼두면
  · `python3 -m rail_control_core.engine` 한 줄로 즉시 검증 가능
  · pytest 회귀 테스트를 ROS 없이 0.1초 만에 돌릴 수 있음
  · 나중에 ROS를 갈아끼워도 로직은 그대로
이게 A(관제 코어) 담당자가 B·C를 기다리지 않고 혼자 진도를 뺄 수 있는 비결입니다.

동작 방식
--------
`step(dt)` 를 계속 호출하는 틱(tick) 기반 시뮬레이션입니다.
한 틱에서 하는 일은 순서가 정해져 있습니다.

  1. 오더 릴리스   : 발생 시각이 된 오더를 큐에 넣는다
  2. 배차          : 유휴 캡슐에 큐 맨 앞 오더를 붙인다      (모드 A=FCFS / B=P-EDD)
  3. 선점 판정     : 상위 오더의 경로를 막고 있는 하위 캡슐을 찾아 처분한다
  4. 캡슐 전진     : 각 캡슐의 FSM을 한 스텝 진행시킨다
  5. 이벤트 배출   : 바뀐 것들을 이벤트 리스트로 내보낸다
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from .fsm import (
    BlockState,
    Capsule,
    CapsuleState,
    Order,
    OrderState,
)
from .topology import Topology

DEFAULT_PARAMS = {
    "mode": "B",
    "tick_s": 0.1,
    "load_s": 1.0,
    "handover_s": 2.0,
    "return_s": 2.0,
    "runthrough_threshold_s": 12.0,
    "evacuate_min_gap": 1,
}


@dataclass
class Event:
    t: float
    kind: str          # order_event | block_state | capsule_state | preempt | kpi
    payload: dict

    def as_dict(self) -> dict:
        return {"t": round(self.t, 2), "kind": self.kind, **self.payload}


class RailEngine:
    # =================================================================
    def __init__(self, topo: Topology, params: dict | None = None):
        self.topo = topo
        self.p = {**DEFAULT_PARAMS, **(params or {})}
        self.mode: str = str(self.p["mode"]).upper()

        self.now: float = 0.0
        self.orders: dict[str, Order] = {}
        self.queue: list[Order] = []
        self.capsules: dict[str, Capsule] = {}

        # 블록 FSM 상태 + 소유자
        self.block_state: dict[str, BlockState] = {b: BlockState.FREE for b in topo.blocks}
        self.block_owner: dict[str, str | None] = {b: None for b in topo.blocks}

        self.events: list[Event] = []
        self.log: list[str] = []

    # -----------------------------------------------------------------
    # 초기화 헬퍼
    # -----------------------------------------------------------------
    def add_capsule(self, cid: str, node: str) -> Capsule:
        cap = Capsule(cid=cid, node=node, home=node)
        self.capsules[cid] = cap
        return cap

    def load_scenario(self, path: str | Path) -> None:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        for o in data["orders"]:
            self.add_order(Order(
                oid=o["id"], priority=int(o["priority"]), item=o["item"],
                origin=o["origin"], dest=o["dest"],
                release_s=float(o["release_s"]), due_s=float(o["due_s"]),
                cargo_class=o.get("cargo_class", "clean"),
                code_red=bool(o.get("code_red", False)),
                load_s=(float(o["load_s"]) if "load_s" in o else None),
            ))

    def add_order(self, order: Order) -> None:
        """오더를 등록하고, 출발지에 전용 캡슐을 배치합니다 (MVP: 캡슐 = 오더 1:1).

        캡슐 ID는 오더 ID에서 뽑되 **반드시 충돌을 피해야** 합니다.
        예) 시연 중 /code_red 로 들어온 "CR-1" 을 단순히 마지막 조각으로 자르면
            "C-1" 이 되어 기존 O-1 의 캡슐과 겹칩니다. 그러면 새 캡슐이 만들어지지
            않아 출발지에 배차할 캡슐이 없고, 오더가 QUEUED 상태로 영원히 멈춥니다.
        """
        self.orders[order.oid] = order

        cid = f"C-{order.oid.split('-')[-1]}"
        if cid in self.capsules:                 # 충돌 → 오더 ID 전체를 사용
            cid = f"C-{order.oid}"               # 예: CR-1 → C-CR-1
            n = 2
            while cid in self.capsules:
                cid = f"C-{order.oid}-{n}"
                n += 1
        self.add_capsule(cid, order.origin)

    # =================================================================
    # 메인 루프
    # =================================================================
    def step(self, dt: float | None = None) -> list[Event]:
        dt = dt if dt is not None else float(self.p["tick_s"])
        self.events = []
        self.now += dt

        self._release_orders()
        self._dispatch()
        if self.mode == "B":
            self._preempt()
        self._advance_capsules(dt)
        return self.events

    def run_until_done(self, limit_s: float = 600.0) -> float:
        """모든 오더가 끝날 때까지 돌리고 makespan(마지막 완료 시각)을 반환."""
        dt = float(self.p["tick_s"])
        while self.now < limit_s and not self.all_done():
            self.step(dt)
        return self.makespan()

    def all_done(self) -> bool:
        return bool(self.orders) and all(
            o.state == OrderState.DELIVERED for o in self.orders.values()
        )

    # =================================================================
    # 1. 오더 릴리스
    # =================================================================
    def _release_orders(self) -> None:
        for o in self.orders.values():
            if o.state == OrderState.CREATED and self.now >= o.release_s:
                o.state = OrderState.QUEUED
                self.queue.append(o)
                self._emit("order_event", order_id=o.oid, state=o.state.value,
                           priority=o.priority, item=o.item,
                           origin=o.origin, dest=o.dest, code_red=o.code_red)

    # =================================================================
    # 2. 배차 — 여기가 모드 A와 B가 갈리는 지점
    # =================================================================
    def _sort_queue(self) -> None:
        if self.mode == "A":
            # 모드 A: 선착순(FCFS). 우선순위를 아예 보지 않습니다. → 비교군
            self.queue.sort(key=lambda o: (o.release_s, o.oid))
        else:
            # 모드 B: P-EDD. 등급 먼저, 같은 등급이면 마감 빠른 순.
            self.queue.sort(key=lambda o: o.pedd_key)

    def _dispatch(self) -> None:
        self._sort_queue()
        for o in list(self.queue):
            cap = self._free_capsule_at(o.origin)
            if cap is None:
                continue
            route = self.topo.find_path(o.origin, o.dest, o.cargo_class)
            if not route:
                self.log.append(f"[{self.now:6.1f}s] {o.oid} 경로 없음 {o.origin}->{o.dest}")
                continue

            self.queue.remove(o)
            o.state = OrderState.ASSIGNED
            o.assigned_capsule = cap.cid
            o.start_s = self.now
            cap.order = o
            cap.route = route
            cap.state = CapsuleState.LOADING
            cap.timer = o.load_s if o.load_s is not None else float(self.p["load_s"])
            self._emit("order_event", order_id=o.oid, state=o.state.value,
                       capsule=cap.cid, route=route,
                       eta_s=round(self.topo.eta(route), 1))

    def _free_capsule_at(self, node: str) -> Capsule | None:
        for c in self.capsules.values():
            if c.state == CapsuleState.IDLE and c.node == node:
                return c
        return None

    # =================================================================
    # 3. 계단식 선점 (모드 B 전용) — 프로젝트의 핵심
    # =================================================================
    ACTIVE_STATES = (
        CapsuleState.LOADING, CapsuleState.MOVING,
        CapsuleState.RUN_THROUGH, CapsuleState.YIELD_WAIT,
        CapsuleState.EVACUATING,
    )

    def wanted_by_higher(self, bid: str, prio: int, me: str) -> str | None:
        """이 블록을 나보다 상위 등급 캡슐이 (지금 또는 곧) 필요로 하는가?

        이것이 '예약 테이블'의 역할을 대신합니다. 상위 캡슐의 남은 경로 전체를
        임시 긴급 회랑(Emergency Corridor)으로 보고, 하위는 그 안에 발을 들이지
        않습니다. 상위 → 하위 방향으로만 양보하므로 교착(deadlock)이 생기지 않습니다.
        """
        for c in self.capsules.values():
            if c.cid == me or c.order is None or c.state not in self.ACTIVE_STATES:
                continue
            if c.order.priority < prio and bid in c.remaining_blocks():
                return c.order.oid
        return None

    def _preempt(self) -> None:
        """상위 캡슐의 회랑을 '이미 점유하고 있는' 하위 캡슐을 처분한다.

        처분은 3가지뿐이고, 고르는 기준은 물리적으로 무엇이 가능한가입니다.
          (a) 아직 그 블록에 안 들어갔다   → YIELD_WAIT  (양보 대기)
          (b) 들어갔는데 곧 나온다/이탈불가 → RUN_THROUGH (완주 허용) ※ 역주행 불가
          (c) 들어갔고 오래 걸리며 지선 有 → EVACUATING  (대피)
        """
        actives = sorted(
            [c for c in self.capsules.values()
             if c.order and c.state in (CapsuleState.MOVING, CapsuleState.LOADING,
                                        CapsuleState.YIELD_WAIT, CapsuleState.RUN_THROUGH)],
            key=lambda c: c.order.pedd_key,
        )
        for hi in actives:
            for bid in hi.remaining_blocks():
                owner_cid = self.block_owner.get(bid)
                if owner_cid in (None, hi.cid):
                    continue
                lo = self.capsules[owner_cid]
                if lo.order is None or lo.order.priority <= hi.order.priority:
                    continue
                self._resolve_conflict(hi, lo, bid)

    def _resolve_conflict(self, hi: Capsule, lo: Capsule, block_id: str) -> None:
        blk = self.topo.blocks[block_id]

        # (a) 아직 그 블록에 진입 전 → 양보 대기
        if lo.cur_block != block_id:
            if lo.state not in (CapsuleState.YIELD_WAIT, CapsuleState.EVACUATING):
                self._set_capsule(lo, CapsuleState.YIELD_WAIT)
                self._emit("preempt", action="YIELD_WAIT", by=hi.order.oid,
                           target=lo.order.oid, block=block_id,
                           reason="구간 진입 전이므로 정차 후 양보")
            return

        if lo.state in (CapsuleState.RUN_THROUGH, CapsuleState.EVACUATING):
            return  # 이미 처분됨

        remain = lo.remain_in_block(blk.traverse_s)
        siding = self.topo.siding_of(blk.other_end(lo.node))
        can_evac = blk.allow_evacuate and siding is not None

        # (b) 잔여 통과시간이 짧거나, 애초에 중간 이탈이 불가능한 구간(수직 쉬프트)
        if remain <= float(self.p["runthrough_threshold_s"]) or not can_evac:
            self._set_capsule(lo, CapsuleState.RUN_THROUGH)
            self._emit("preempt", action="RUN_THROUGH", by=hi.order.oid,
                       target=lo.order.oid, block=block_id,
                       remain_s=round(remain, 1),
                       reason=("역주행·중간 이탈 불가 구간" if not can_evac
                               else "잔여 통과시간이 짧아 통과가 더 빠름"))
            return

        # (c) 대피
        lo.evac_target = siding
        self._set_capsule(lo, CapsuleState.EVACUATING)
        self._emit("preempt", action="EVACUATE", by=hi.order.oid,
                   target=lo.order.oid, block=block_id, siding=siding,
                   remain_s=round(remain, 1),
                   reason="상위 화물의 목적 구간을 막고 있어 지선으로 회피")

    # =================================================================
    # 4. 캡슐 FSM 전진
    # =================================================================
    def _advance_capsules(self, dt: float) -> None:
        for cap in self.capsules.values():
            st = cap.state

            if st == CapsuleState.IDLE:
                continue

            if st == CapsuleState.LOADING:
                cap.timer -= dt
                if cap.timer <= 0:
                    cap.order.state = OrderState.RUNNING
                    self._set_capsule(cap, CapsuleState.MOVING)
                continue

            if st in (CapsuleState.MOVING, CapsuleState.RUN_THROUGH):
                self._move(cap, dt)
                continue

            if st == CapsuleState.YIELD_WAIT:
                # 지연 비용 누적 — "선점은 공짜가 아니다"를 숫자로 만드는 지점
                cap.order.wait_s += dt
                if cap.cur_block is not None:
                    self._move(cap, dt)          # 지금 있는 구간은 끝까지 빠져나온다
                    continue
                nxt = cap.route[0] if cap.route else None
                if nxt and self.block_state[nxt] == BlockState.FREE \
                        and not self.wanted_by_higher(nxt, cap.order.priority, cap.cid):
                    self._set_capsule(cap, CapsuleState.MOVING)
                    self._emit("preempt", action="RESUME", target=cap.order.oid,
                               block=nxt, reason="상위 화물 통과 완료 → 자동 복구")
                continue

            if st == CapsuleState.EVACUATING:
                cap.order.wait_s += dt
                self._evacuate(cap, dt)
                continue

            if st == CapsuleState.HANDOVER:
                cap.timer -= dt
                if cap.timer <= 0:
                    o = cap.order
                    o.state = OrderState.DELIVERED
                    o.finish_s = self.now
                    self._emit("order_event", order_id=o.oid, state=o.state.value,
                               lead_time_s=round(o.lead_time_s, 1),
                               wait_s=round(o.wait_s, 1),
                               slack_s=round(o.slack_s(self.now), 1),
                               overdue=o.slack_s(self.now) < 0)
                    cap.order = None
                    self._set_capsule(cap, CapsuleState.RETURNING)
                    cap.timer = float(self.p["return_s"])
                continue

            if st == CapsuleState.RETURNING:
                cap.timer -= dt
                if cap.timer <= 0:
                    self._set_capsule(cap, CapsuleState.IDLE)
                continue

    # -----------------------------------------------------------------
    def _move(self, cap: Capsule, dt: float) -> None:
        # 블록에 아직 안 들어갔으면 진입 시도
        if cap.cur_block is None:
            if not cap.route:
                self._arrive(cap)
                return
            nxt = cap.route[0]

            # 상위 등급이 이 블록을 회랑으로 잡고 있으면 진입 금지(계단식 양보)
            if self.mode == "B" and cap.state == CapsuleState.MOVING:
                blocker = self.wanted_by_higher(nxt, cap.order.priority, cap.cid)
                if blocker:
                    self._set_capsule(cap, CapsuleState.YIELD_WAIT)
                    self._emit("preempt", action="YIELD_WAIT", by=blocker,
                               target=cap.order.oid, block=nxt,
                               reason="구간 진입 전이므로 정차 후 양보")
                    return

            if not self._acquire_block(nxt, cap.cid):
                cap.order.wait_s += dt          # 블록이 안 비어서 대기 → 이것도 지연
                return
            cap.cur_block = nxt
            cap.block_elapsed = 0.0
            self._emit("block_state", block=nxt, state=BlockState.OCCUPIED.value,
                       capsule=cap.cid, order=cap.order.oid)
            return

        # 블록 안에서 전진
        blk = self.topo.blocks[cap.cur_block]
        cap.block_elapsed += dt
        if cap.block_elapsed < blk.traverse_s:
            return

        # 블록 통과 완료
        cap.node = blk.other_end(cap.node)
        self._release_block(cap.cur_block, cap.cid)
        cap.route.pop(0)
        cap.cur_block = None
        cap.block_elapsed = 0.0
        if cap.state == CapsuleState.RUN_THROUGH:
            self._set_capsule(cap, CapsuleState.MOVING)
        if not cap.route:
            self._arrive(cap)

    def _arrive(self, cap: Capsule) -> None:
        self._set_capsule(cap, CapsuleState.HANDOVER)
        cap.timer = float(self.p["handover_s"])

    def _evacuate(self, cap: Capsule, dt: float) -> None:
        """지선으로 빠졌다가, 길이 열리면 원래 목적지로 다시 경로를 짜서 복귀합니다."""
        if cap.cur_block is not None:          # 아직 본선 블록 안 → 일단 빠져나간다
            self._move(cap, dt)
            return

        # 지선까지 아직 못 갔으면 지선 경로로 갈아탄다
        if cap.evac_target and cap.node != cap.evac_target:
            path = self.topo.find_path(cap.node, cap.evac_target, cap.order.cargo_class)
            if path:
                cap.route = path
                self._move(cap, dt)
            return

        # 지선 도착 → 상위 화물이 다 지나갔는지 확인 후 재경로 산출
        onward = self.topo.find_path(cap.node, cap.order.dest, cap.order.cargo_class)
        if onward and not any(
            self.wanted_by_higher(b, cap.order.priority, cap.cid) for b in onward
        ):
            cap.route = onward
            cap.evac_target = None
            self._set_capsule(cap, CapsuleState.MOVING)
            self._emit("preempt", action="RESUME", target=cap.order.oid,
                       reason="상위 화물 통과 완료 → 대피지점에서 본선 복귀")

    # =================================================================
    # 블록 자원 관리
    # =================================================================
    def _acquire_block(self, bid: str, cid: str) -> bool:
        if self.block_state[bid] == BlockState.FREE or self.block_owner[bid] == cid:
            self.block_state[bid] = BlockState.OCCUPIED
            self.block_owner[bid] = cid
            return True
        return False

    def _release_block(self, bid: str, cid: str) -> None:
        if self.block_owner.get(bid) == cid:
            self.block_state[bid] = BlockState.FREE
            self.block_owner[bid] = None
            self._emit("block_state", block=bid, state=BlockState.FREE.value, capsule=None)

    # =================================================================
    # 유틸
    # =================================================================
    def _set_capsule(self, cap: Capsule, new: CapsuleState) -> None:
        if cap.state == new:
            return
        old, cap.state = cap.state, new
        self._emit("capsule_state", capsule=cap.cid,
                   order=cap.order.oid if cap.order else None,
                   frm=old.value, to=new.value, node=cap.node)

    def _emit(self, kind: str, **payload) -> None:
        ev = Event(t=self.now, kind=kind, payload=payload)
        self.events.append(ev)
        self.log.append(f"[{self.now:6.1f}s] {kind:<14} {payload}")

    # ---------------- KPI ----------------
    def makespan(self) -> float:
        fin = [o.finish_s for o in self.orders.values() if o.finish_s is not None]
        return max(fin) if fin else float("inf")

    def kpi(self) -> dict:
        done = [o for o in self.orders.values() if o.finish_s is not None]
        lead = [o.lead_time_s for o in done]
        return {
            "mode": self.mode,
            "makespan_s": round(self.makespan(), 1) if done else None,
            "avg_lead_time_s": round(sum(lead) / len(lead), 1) if lead else None,
            "p0_lead_time_s": next(
                (round(o.lead_time_s, 1) for o in done if o.priority == 0), None),
            "total_wait_s": round(sum(o.wait_s for o in self.orders.values()), 1),
            "overdue_orders": [o.oid for o in done if o.finish_s > o.due_s],
            "per_order": {
                o.oid: {
                    "P": o.priority,
                    "lead_s": round(o.lead_time_s, 1) if o.lead_time_s else None,
                    "wait_s": round(o.wait_s, 1),
                }
                for o in sorted(self.orders.values(), key=lambda x: x.oid)
            },
        }

    def snapshot(self) -> dict:
        """UI(D 담당자)에게 통째로 넘길 현재 상태."""
        return {
            "t": round(self.now, 2),
            "mode": self.mode,
            "blocks": {b: {"state": s.value, "owner": self.block_owner[b]}
                       for b, s in self.block_state.items()},
            "capsules": {c.cid: {"state": c.state.value, "node": c.node,
                                 "order": c.order.oid if c.order else None,
                                 "block": c.cur_block}
                         for c in self.capsules.values()},
            "orders": {o.oid: {"state": o.state.value, "P": o.priority,
                               "wait_s": round(o.wait_s, 1),
                               "slack_s": round(o.slack_s(self.now), 1)}
                       for o in self.orders.values()},
        }


# =====================================================================
# 단독 실행: python3 -m rail_control_core.engine
# =====================================================================
def build_engine(mode: str = "B", cfg_dir: str | Path | None = None) -> RailEngine:
    cfg = Path(cfg_dir) if cfg_dir else Path(__file__).resolve().parent.parent / "config"
    topo = Topology(cfg / "topology.yaml")
    params = yaml.safe_load((cfg / "params.yaml").read_text(encoding="utf-8"))
    p = params["control_core_node"]["ros__parameters"]
    p["mode"] = mode
    eng = RailEngine(topo, p)
    eng.load_scenario(cfg / "scenario_main.yaml")
    return eng


def main() -> None:
    for mode in ("A", "B"):
        eng = build_engine(mode)
        eng.run_until_done()
        print(f"\n{'='*62}\n  모드 {mode}  ({'FCFS·선점없음' if mode=='A' else 'P-EDD·계단식 선점'})\n{'='*62}")
        for line in eng.log:
            if any(k in line for k in ("preempt", "order_event")):
                print(" ", line)
        print("  KPI:", eng.kpi())


if __name__ == "__main__":
    main()
