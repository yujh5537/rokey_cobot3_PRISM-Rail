"""
control_core_node.py — 엔진을 ROS2로 감싸는 얇은 껍데기.

설계 원칙: 이 파일에는 관제 '판단'이 한 줄도 없습니다.
판단은 전부 engine.py 에 있고, 여기서는 타이머로 engine.tick() 을 부르고
결과를 토픽으로 뿌리기만 합니다. 그래야 엔진을 ROS 없이 테스트할 수 있습니다.
(engine/scenario 는 순수 파이썬 — test/test_regression.py 는 ROS 없이 돕니다.)

발행 (Publish)
--------------
  /order_event   std_msgs/String  (JSON)  오더 상태 변화
  /block_state   std_msgs/String  (JSON)  블록 점유·선점 잠금 상태표 (변화 시에만)
  /capsule_cmd   std_msgs/String  (JSON)  캡슐 FSM 전이 / 선점·대피 연출 이벤트
  /control_state std_msgs/String  (JSON)  전체 스냅샷 → D(UI) 대시보드용
  /kpi           std_msgs/String  (JSON)  KPI 요약 (시나리오 종료 시 1회)

페이로드 스키마는 src/rail_bridge/rail_bridge/interface_schema.json 과 동기입니다.
한쪽만 고치지 마세요.

서비스 (Service)
----------------
  /code_red        std_srvs/Trigger  P0 Code Crimson 콘보이 수동 발령 (시연용 버튼)
  /reset           std_srvs/Trigger  시나리오 리셋
  /toggle_mode     std_srvs/Trigger  모드 A ↔ B 전환

왜 커스텀 .msg 를 안 쓰고 String+JSON 인가요?
---------------------------------------------
커스텀 메시지는 별도 CMake 패키지를 만들고 4대 PC 전원이 재빌드해야 합니다.
6일 일정에서는 그 시간이 아깝고, rosbridge로 D에게 넘길 때도 JSON이 그대로 통과합니다.
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import scenario
from . import topology as T
from .engine import DT
from .fsm import Capsule, CapsuleState, Order, OrderState

QOS = 10

# 엔진 이벤트 → 발행 토픽 분류
ORDER_EVENTS = {"ORDER_RELEASE", "ORDER_ARRIVE"}
PREEMPT_EVENTS = {"YIELD": "YIELD", "EVAC_LANE": "EVACUATE", "EVAC_SPUR": "EVACUATE",
                  "FINISH_ALLOWED": "FINISH_ALLOWED", "RESUME": "RESUME"}
PREEMPT_REASON = {
    "YIELD": "상위 등급 통과 대기 — 진입 전 양보 (R2)",
    "EVAC_LANE": "Code Crimson 콘보이 회랑 확보를 위해 대피 레인으로 회피 (R3)",
    "EVAC_SPUR": "Code Crimson 콘보이 회랑 확보를 위해 지선으로 회피 (R3)",
    "FINISH_ALLOWED": "이미 진입한 블록은 역주행 불가 — 완주 허용 (R4)",
    "RESUME": "선점 파도 통과 완료 — 주행 재개",
}
ITEM_NAME = {0: "응급 수혈혈액(Code Crimson 콘보이)", 1: "응급 약품",
             2: "항암제", 3: "오염 기구 회수"}


class ControlCoreNode(Node):
    def __init__(self) -> None:
        super().__init__("control_core_node")

        # ---------- 파라미터 ----------
        pkg_share = Path(__file__).resolve().parent.parent
        self.declare_parameter("config_dir", str(pkg_share / "config"))
        self.declare_parameter("mode", "B")
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("auto_start", True)

        self.cfg_dir = Path(self.get_parameter("config_dir").value)
        self.mode = str(self.get_parameter("mode").value).upper()

        self._load_engine_params()

        # ---------- 발행자 ----------
        self.pub_order = self.create_publisher(String, "/order_event", QOS)
        self.pub_block = self.create_publisher(String, "/block_state", QOS)
        self.pub_cmd = self.create_publisher(String, "/capsule_cmd", QOS)
        self.pub_state = self.create_publisher(String, "/control_state", QOS)
        self.pub_kpi = self.create_publisher(String, "/kpi", QOS)

        # ---------- 서비스 ----------
        self.create_service(Trigger, "/code_red", self.on_code_red)
        self.create_service(Trigger, "/reset", self.on_reset)
        self.create_service(Trigger, "/toggle_mode", self.on_toggle_mode)

        # ---------- 엔진 ----------
        self._build_engine()

        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.on_tick)
        self.running = bool(self.get_parameter("auto_start").value)

        errs = T.validate()
        if errs:
            self.get_logger().error(f"토폴로지 경로 무결성 실패: {errs}")
        self.get_logger().info(
            f"관제 코어 기동 — 모드 {self.mode}, 블록 {len(T.BLOCKS)}개 / "
            f"캡슐 {len(self.engine.capsules)}대 / 오더 {len(self.engine.orders)}건"
        )

    # ------------------------------------------------------------------
    def _load_engine_params(self) -> None:
        """config/params.yaml 의 engine 블록으로 scenario.PARAMS 를 덮어씁니다.

        scenario.PARAMS 가 엔진 파라미터의 단일 출처이므로, 여기서 한 번만
        갱신하면 build() 가 그대로 사용합니다.
        """
        path = self.cfg_dir / "params.yaml"
        if not path.exists():
            self.get_logger().warn(f"{path} 없음 — scenario.PARAMS 기본값 사용")
            return
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        eng = raw.get("engine") or {}
        unknown = set(eng) - set(scenario.PARAMS)
        if unknown:
            self.get_logger().warn(f"params.yaml engine 블록의 미사용 키: {sorted(unknown)}")
        scenario.PARAMS.update({k: v for k, v in eng.items() if k in scenario.PARAMS})

    def _build_engine(self) -> None:
        self.engine = scenario.build(self.mode)
        self._ev_idx = 0                       # 발행한 engine.events 인덱스
        self._prev_cap = {}                    # cid -> CapsuleState (전이 감지용)
        self._prev_block_snap = None           # 변화 시에만 발행
        self._kpi_sent = False
        self._cr_seq = 0

    # ------------------------------------------------------------------
    def on_tick(self) -> None:
        if not self.running:
            return

        # 벽시계 1주기 동안 진행할 시뮬레이션 시간 = (1/publish_hz) × 배속
        hz = float(self.get_parameter("publish_hz").value)
        scale = max(0.1, float(self.get_parameter("speed_scale").value))
        steps = max(1, round((scale / hz) / DT))
        for _ in range(steps):
            if self._all_done():
                break
            self.engine.tick()

        self._publish_events()
        self._publish_capsule_transitions()
        self._publish_block_state()
        self.pub_state.publish(
            String(data=json.dumps(self._snapshot(), ensure_ascii=False))
        )

        if self._all_done() and not self._kpi_sent:
            kpi = self._kpi()
            self.pub_kpi.publish(String(data=json.dumps(kpi, ensure_ascii=False)))
            self.get_logger().info(f"시나리오 종료 — KPI: {kpi}")
            self._kpi_sent = True
            self.running = False

    # ------------------------------------------------------------------
    def _all_done(self) -> bool:
        return bool(self.engine.orders) and all(
            o.state == OrderState.DONE for o in self.engine.orders.values()
        )

    def _p0_source(self) -> str:
        """현재 선점을 유발 중인 P0 오더 id (연출 이벤트의 'by' 필드)."""
        for o in self.engine.orders.values():
            if o.prio == 0 and o.state == OrderState.EN_ROUTE:
                return o.oid
        return ""

    def _publish_events(self) -> None:
        """engine.events 의 신규분을 /order_event · /capsule_cmd 로 흘려보냅니다."""
        new = self.engine.events[self._ev_idx:]
        self._ev_idx = len(self.engine.events)
        for t, ev, subj, detail in new:
            if ev in ORDER_EVENTS:
                o = self.engine.orders.get(subj)
                self.pub_order.publish(String(data=json.dumps({
                    "t": t, "kind": "order_event", "order_id": subj,
                    "priority": o.prio if o else None,
                    "item": ITEM_NAME.get(o.prio if o else -1, ""),
                    "state": o.state.value if o else None,
                    "code_red": bool(o and o.prio == 0),
                }, ensure_ascii=False)))
            elif ev in PREEMPT_EVENTS:
                c = self.engine.capsules.get(subj)
                block, siding = self._preempt_blocks(ev, detail, c)
                self.pub_cmd.publish(String(data=json.dumps({
                    "t": t, "kind": "preempt", "action": PREEMPT_EVENTS[ev],
                    "by": self._p0_source(),
                    "target": c.order.oid if c and c.order else subj,
                    "capsule": subj, "block": block, "siding": siding,
                    "reason": PREEMPT_REASON[ev],
                }, ensure_ascii=False)))
                if ev != "RESUME":
                    self.get_logger().warn(
                        f"[선점] {PREEMPT_EVENTS[ev]} {self._p0_source()} → {subj} ({detail})"
                    )

    @staticmethod
    def _preempt_blocks(ev: str, detail: str, c: Capsule | None):
        """엔진 로그의 detail 문자열에서 (블록, 대피레인) 을 뽑습니다."""
        if ev in ("EVAC_LANE", "EVAC_SPUR") and "->" in detail:
            a, b = detail.split("->", 1)
            return a, b
        if ev == "YIELD":
            return detail.split(":", 1)[0], None
        return (detail or (c.block if c else None)), None

    def _publish_capsule_transitions(self) -> None:
        """캡슐 FSM 상태가 바뀐 것만 /capsule_cmd 로 발행합니다."""
        for cid, c in self.engine.capsules.items():
            prev = self._prev_cap.get(cid)
            if prev == c.state:
                continue
            self._prev_cap[cid] = c.state
            if prev is None:                   # 초기 상태는 전이가 아님
                continue
            self.pub_cmd.publish(String(data=json.dumps({
                "t": round(self.engine.t, 2), "kind": "capsule_state",
                "capsule": cid, "order": c.order.oid if c.order else None,
                "from": prev.value, "to": c.state.value,
                "block": c.block,
                "node": T.exit_node(c.block, c.fwd) if c.block else None,
            }, ensure_ascii=False)))

    def _publish_block_state(self) -> None:
        snap = self.engine.block_snapshot()
        if snap == self._prev_block_snap:
            return
        self._prev_block_snap = snap
        self.pub_block.publish(String(data=json.dumps(
            {"t": round(self.engine.t, 2), "kind": "block_state", "blocks": snap},
            ensure_ascii=False)))

    def _snapshot(self) -> dict:
        """D(UI) 대시보드가 재조립할 필요 없도록 매 틱 전체 상태를 통째로 보냅니다."""
        return {
            "t": round(self.engine.t, 2), "kind": "control_state", "mode": self.mode,
            "running": self.running,
            "orders": {
                o.oid: {"priority": o.prio, "state": o.state.value,
                        "release": o.release_t, "due": o.due_t,
                        "arrive": round(o.arrive_t, 2) if o.arrive_t else None,
                        "wait": round(o.wait_total, 2),
                        "capsules": o.capsule_ids}
                for o in self.engine.orders.values()
            },
            "capsules": {
                c.cid: {"state": c.state.value, "block": c.block,
                        "pos": round(c.pos, 3), "fwd": c.fwd,
                        "order": c.order.oid if c.order else None,
                        "detour": c.detour}
                for c in self.engine.capsules.values()
            },
            "blocks": self.engine.block_snapshot(),
        }

    def _kpi(self) -> dict:
        """명세서 §6-2 회귀 기준값과 같은 형식의 요약."""
        orders = self.engine.orders.values()
        return {
            "kind": "kpi", "mode": self.mode,
            "makespan": round(max(o.arrive_t for o in orders), 2),
            "orders": {o.oid: {"priority": o.prio,
                               "arrive": round(o.arrive_t, 2),
                               "wait": round(o.wait_total, 2)} for o in orders},
            "arrival_order": [o.oid for o in sorted(orders, key=lambda x: x.arrive_t)],
        }

    # ------------------------------------------------------------------
    # 서비스 콜백
    # ------------------------------------------------------------------
    def on_code_red(self, _req, res):
        """시연 중 아무 때나 Code Crimson 콘보이를 밀어넣습니다.

        디포(BB-07)에서 유휴 상태(DOCKED)인 캡슐을 최대 4대까지 징발해
        대기열(BB-08) → 출동 램프(BB-09) 경로로 수술장에 보냅니다.
        """
        idle = [c for c in self.engine.capsules.values()
                if c.state == CapsuleState.DOCKED and c.block == "BB-07"][:4]
        if not idle:
            res.success = False
            res.message = "디포에 유휴 캡슐이 없습니다 — 발령 불가"
            self.get_logger().warn(res.message)
            return res

        self._cr_seq += 1
        oid = f"CR-{self._cr_seq}"
        now = self.engine.t
        due = now + scenario.PARAMS["priority_due_sec"][0]
        order = Order(oid, 0, "convoy", now, due, [c.cid for c in idle])
        self.engine.orders[oid] = order

        # 디포에서 출발하므로 대기열(BB-08)을 앞에 붙인 경로를 부여합니다.
        for i, c in enumerate(idle):
            rname = "P0_OR1" if i % 2 == 0 else "P0_OR2"
            c.order = order
            c.route = [("BB-08", True)] + list(T.ROUTES[rname])
            c.idx = -1
            c.fwd = True
            c.state = CapsuleState.QUEUED     # 발령 틱에 MOVING 으로 전이
            c.req_t = float("inf")

        self.running = True
        self._kpi_sent = False
        res.success = True
        res.message = f"Code Crimson 발령: {oid} — 캡슐 {[c.cid for c in idle]} (디포 → 수술장)"
        self.get_logger().error(f"🔴 {res.message}")
        return res

    def on_reset(self, _req, res):
        self._load_engine_params()
        self._build_engine()
        self.running = True
        res.success = True
        res.message = f"시나리오 리셋 (모드 {self.mode})"
        self.get_logger().info(res.message)
        return res

    def on_toggle_mode(self, _req, res):
        self.mode = "A" if self.mode == "B" else "B"
        self._build_engine()
        self.running = True
        res.success = True
        res.message = (f"모드 {self.mode} "
                       f"({'FCFS·선점없음' if self.mode == 'A' else 'P-EDD·계단식 선점'})")
        self.get_logger().info(res.message)
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlCoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
