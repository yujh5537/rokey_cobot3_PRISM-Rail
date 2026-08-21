"""
control_core_node.py — 관제 코어 ROS2 노드 (얇은 어댑터).

설계 원칙 (포트-어댑터)
----------------------
판단 로직은 전부 bridge.Bridge 에 있고, 이 파일은 dict 페이로드를 JSON 으로
바꿔 발행하기만 합니다. 덕분에 ROS 없이 test/test_bridge.py 로 노드 로직을
검증할 수 있고, 통합 중 문제가 생기면 "브리지 테스트 통과 여부"로 관제 책임과
ROS 경계 책임(빌드·QoS·네트워크)을 즉시 분리할 수 있습니다.

발행 (std_msgs/String, 페이로드는 JSON)
--------------------------------------
  /control_state  BEST_EFFORT  depth 1   rate_hz(기본 30Hz) 전체 스냅샷
  /block_state    RELIABLE+TRANSIENT_LOCAL depth 1   변화 시에만
  /order_event    RELIABLE     depth 50  오더 발령·도착·SIM_DONE
  /capsule_cmd    RELIABLE     depth 50  캡슐 FSM 전이 + 선점/대피 연출
  /kpi            RELIABLE+TRANSIENT_LOCAL depth 1   완주 시 1회

왜 커스텀 .msg 가 아니라 String+JSON 인가요?
--------------------------------------------
팀이 06c0783 에서 커스텀 msg 패키지를 삭제하고 이 스키마로 합의했습니다.
커스텀 메시지는 4대 PC 전원이 재빌드해야 하고, rosbridge 로 D 에게 넘길 때도
JSON 이 그대로 통과합니다. 페이로드 스키마는
src/rail_bridge/rail_bridge/interface_schema.json 과 동기입니다 — 한쪽만 고치지 마세요.

⚠️ QoS 주의: /control_state 는 BEST_EFFORT 입니다. RELIABLE 구독자로 받으면
매칭이 안 돼 **아무것도 오지 않습니다**. 안 보이면 구독 QoS 부터 확인하세요.

서비스 (std_srvs/Trigger — 커스텀 srv 없이 ack 를 받기 위해 명령당 하나씩)
-------------------------------------------------------------------------
  /sim_start  /sim_pause  /sim_reset  /sim_status  /sim_mode_a  /sim_mode_b
  /code_crimson  시나리오의 예약 P0 발령을 지금으로 앞당김 (아직 미발령일 때만)
  /code_red      디포 유휴 캡슐로 추가 P0 오더를 새로 만듦 (아무 때나)
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import scenario
from . import topology as T
from .bridge import Bridge
from .fsm import CapsuleState, Order, OrderState

# 엔진 이벤트 → 발행 토픽 분류
ORDER_EVENTS = {"ORDER_RELEASE", "ORDER_ARRIVE", "CODE_CRIMSON", "SIM_DONE"}
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

QOS_STREAM = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.BEST_EFFORT)
QOS_LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_EVENT = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE)


class ControlCoreNode(Node):
    def __init__(self) -> None:
        super().__init__("control_core_node")

        pkg_share = Path(__file__).resolve().parent.parent
        self.declare_parameter("config_dir", str(pkg_share / "config"))
        self.declare_parameter("mode", "B")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("autostart", True)

        self.cfg_dir = Path(self.get_parameter("config_dir").value)
        mode = str(self.get_parameter("mode").value).upper()
        rate = float(self.get_parameter("rate_hz").value)
        autostart = bool(self.get_parameter("autostart").value)

        self._load_engine_params()

        self.pub_state = self.create_publisher(String, "/control_state", QOS_STREAM)
        self.pub_block = self.create_publisher(String, "/block_state", QOS_LATCHED)
        self.pub_order = self.create_publisher(String, "/order_event", QOS_EVENT)
        self.pub_cmd = self.create_publisher(String, "/capsule_cmd", QOS_EVENT)
        self.pub_kpi = self.create_publisher(String, "/kpi", QOS_LATCHED)

        for name, fn in (("start", self._svc_start), ("pause", self._svc_pause),
                         ("reset", self._svc_reset), ("status", self._svc_status),
                         ("mode_a", self._svc_mode_a), ("mode_b", self._svc_mode_b)):
            self.create_service(Trigger, f"/sim_{name}", fn)
        self.create_service(Trigger, "/code_crimson", self.on_code_crimson)
        self.create_service(Trigger, "/code_red", self.on_code_red)

        self.bridge = Bridge(mode=mode, autostart=autostart)
        self._reset_pub_state()

        self.create_timer(1.0 / rate, self.on_tick)

        errs = T.validate()
        if errs:
            self.get_logger().error(f"토폴로지 경로 무결성 실패: {errs}")
        self.get_logger().info(
            f"관제 코어 기동 — mode={mode} rate={rate}Hz autostart={autostart} "
            f"| 블록 {len(T.BLOCKS)}개 / 캡슐 {len(self.bridge.eng.capsules)}대 "
            f"(수동 시작: ros2 service call /sim_start std_srvs/srv/Trigger)")

    # ------------------------------------------------------------------
    def _load_engine_params(self) -> None:
        """config/params.yaml 의 engine 블록으로 scenario.PARAMS 를 덮어씁니다.

        scenario.PARAMS 가 엔진 파라미터의 단일 출처이므로 여기서 한 번만
        갱신하면 Bridge → scenario.build() 가 그대로 사용합니다.
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

    def _reset_pub_state(self) -> None:
        self._prev_cap: dict[str, str] = {}
        self._kpi_sent = False
        self._cr_seq = 0

    # ------------------------------------------------------------------
    def on_tick(self) -> None:
        # 배속: 타이머 1주기에 브리지를 여러 틱 돌리고 페이로드를 합칩니다.
        steps = max(1, round(float(self.get_parameter("speed_scale").value)))
        caps, blocks, events, sim_t = None, None, [], self.bridge.eng.t
        for _ in range(steps):
            out = self.bridge.step()
            caps, sim_t = out["capsules"], out["sim_t"]
            if out["blocks"] is not None:
                blocks = out["blocks"]
            events += out["events"]

        self._publish_events(events, sim_t)
        self._publish_capsule_transitions(caps, sim_t)
        if blocks is not None:
            self.pub_block.publish(String(data=json.dumps(
                {"sim_t": round(sim_t, 2), "kind": "block_state", "blocks": blocks},
                ensure_ascii=False)))
        self.pub_state.publish(String(data=json.dumps(
            self._snapshot(caps, sim_t), ensure_ascii=False)))

        if any(e["event"] == "SIM_DONE" for e in events) and not self._kpi_sent:
            kpi = self._kpi()
            self.pub_kpi.publish(String(data=json.dumps(kpi, ensure_ascii=False)))
            self.get_logger().info(f"시나리오 종료 — KPI: {kpi}")
            self._kpi_sent = True

    # ------------------------------------------------------------------
    def _p0_source(self) -> str:
        """현재 선점을 유발 중인 P0 오더 id (연출 이벤트의 'by' 필드)."""
        for o in self.bridge.eng.orders.values():
            if o.prio == 0 and o.state == OrderState.EN_ROUTE:
                return o.oid
        return ""

    def _publish_events(self, events: list[dict], sim_t: float) -> None:
        for e in events:
            ev, subj, detail, t = e["event"], e["subject"], e["detail"], e["sim_t"]
            if ev in ORDER_EVENTS:
                o = self.bridge.eng.orders.get(subj)
                self.pub_order.publish(String(data=json.dumps({
                    "sim_t": t, "kind": "order_event", "event": ev,
                    "order_id": subj,
                    "priority": o.prio if o else None,
                    "item": ITEM_NAME.get(o.prio, "") if o else "",
                    "state": o.state.value if o else None,
                    "code_red": bool(o and o.prio == 0),
                    "detail": detail,
                }, ensure_ascii=False)))
                if ev in ("CODE_CRIMSON", "SIM_DONE"):
                    self.get_logger().info(f"[{ev}] {subj} {detail}")
            elif ev in PREEMPT_EVENTS:
                c = self.bridge.eng.capsules.get(subj)
                block, siding = self._preempt_blocks(ev, detail, c)
                self.pub_cmd.publish(String(data=json.dumps({
                    "sim_t": t, "kind": "preempt", "action": PREEMPT_EVENTS[ev],
                    "by": self._p0_source(),
                    "target": c.order.oid if c and c.order else subj,
                    "capsule": subj, "block": block, "siding": siding,
                    "reason": PREEMPT_REASON[ev],
                }, ensure_ascii=False)))
                if ev != "RESUME":
                    self.get_logger().warn(
                        f"[선점] {PREEMPT_EVENTS[ev]} {self._p0_source()} → {subj} ({detail})")

    @staticmethod
    def _preempt_blocks(ev: str, detail: str, c) -> tuple[str, str]:
        """엔진 로그의 detail 문자열에서 (블록, 대피레인) 을 뽑습니다."""
        if ev in ("EVAC_LANE", "EVAC_SPUR") and "->" in detail:
            a, b = detail.split("->", 1)
            return a, b
        if ev == "YIELD":
            return detail.split(":", 1)[0], ""
        return (detail or (c.block if c else "") or ""), ""

    def _publish_capsule_transitions(self, caps: list[dict] | None, sim_t: float) -> None:
        """캡슐 FSM 상태가 바뀐 것만 /capsule_cmd 로 발행합니다 (C → Isaac Sim 중계용)."""
        for c in caps or []:
            cid, now = c["capsule_id"], c["state"]
            prev = self._prev_cap.get(cid)
            self._prev_cap[cid] = now
            if prev is None or prev == now:      # 초기 상태는 전이가 아님
                continue
            # 문자열 필드에는 null 대신 "" 를 보낸다. 수신측이 흔히 쓰는
            # data.get("node", 기본값) 은 키가 '없을 때만' 기본값을 주므로,
            # null 을 보내면 None 이 그대로 흘러들어가 파싱이 깨진다.
            block = c["block_id"]
            self.pub_cmd.publish(String(data=json.dumps({
                "sim_t": round(sim_t, 2), "kind": "capsule_state",
                "capsule": cid, "order": c["order_id"],
                "from": prev, "to": now, "block": block,
                "node": T.exit_node(block, c["forward"]) if block else "",
            }, ensure_ascii=False)))

    def _snapshot(self, caps: list[dict] | None, sim_t: float) -> dict:
        """D(UI)·B(씬) 가 재조립할 필요 없도록 매 틱 전체 상태를 통째로 보냅니다."""
        return {
            "sim_t": round(sim_t, 2), "kind": "control_state",
            "mode": self.bridge.mode, "running": self.bridge.running,
            "orders": {
                o.oid: {"priority": o.prio, "state": o.state.value,
                        "release": o.release_t, "due": o.due_t,
                        "arrive": round(o.arrive_t, 2) if o.arrive_t else None,
                        "wait": round(o.wait_total, 2), "capsules": o.capsule_ids}
                for o in self.bridge.eng.orders.values()
            },
            "capsules": caps or [],
        }

    def _kpi(self) -> dict:
        """명세서 §6-2 회귀 기준값과 같은 형식의 요약."""
        orders = list(self.bridge.eng.orders.values())
        return {
            "kind": "kpi", "mode": self.bridge.mode,
            "makespan": round(max(o.arrive_t for o in orders), 2),
            "orders": {o.oid: {"priority": o.prio, "arrive": round(o.arrive_t, 2),
                               "wait": round(o.wait_total, 2)} for o in orders},
            "arrival_order": [o.oid for o in sorted(orders, key=lambda x: x.arrive_t)],
        }

    # ------------------------------------------------------------------
    # 서비스 콜백 — 전부 std_srvs/Trigger
    # ------------------------------------------------------------------
    def _cmd(self, cmd: str, res):
        res.success, res.message = self.bridge.command(cmd)
        if cmd in ("reset", "mode_a", "mode_b"):
            self._reset_pub_state()
        self.get_logger().info(f"/sim_{cmd}: {res.message}")
        return res

    def _svc_start(self, _q, res):   return self._cmd("start", res)
    def _svc_pause(self, _q, res):   return self._cmd("pause", res)
    def _svc_reset(self, _q, res):   return self._cmd("reset", res)
    def _svc_status(self, _q, res):  return self._cmd("status", res)
    def _svc_mode_a(self, _q, res):  return self._cmd("mode_a", res)
    def _svc_mode_b(self, _q, res):  return self._cmd("mode_b", res)

    def on_code_crimson(self, _q, res):
        """시나리오의 예약 P0 발령을 지금으로 앞당깁니다 (아직 미발령일 때만)."""
        res.success, oid, res.message = self.bridge.code_crimson("manual trigger")
        # rclpy 로거는 심각도를 '호출 소스 라인' 단위로 캐싱한다. 한 줄에서
        # error/warn 을 번갈아 부르면 ValueError 로 노드가 죽으므로 줄을 나눈다.
        if res.success:
            self.get_logger().error(f"🔴 /code_crimson: {res.message}")
        else:
            self.get_logger().warn(f"/code_crimson 거부: {res.message}")
        return res

    def on_code_red(self, _q, res):
        """예약 P0 가 이미 나간 뒤에도 쓸 수 있는 추가 발령 버튼.

        디포(BB-07)에서 유휴 상태인 캡슐을 최대 4대까지 징발해
        대기열(BB-08) → 출동 램프(BB-09) 경로로 수술장에 보냅니다.
        """
        eng = self.bridge.eng
        idle = [c for c in eng.capsules.values()
                if c.state == CapsuleState.DOCKED and c.block == "BB-07"][:4]
        if not idle:
            res.success = False
            res.message = "디포에 유휴 캡슐이 없습니다 — 발령 불가"
            self.get_logger().warn(res.message)
            return res

        self._cr_seq += 1
        oid = f"CR-{self._cr_seq}"
        now = eng.t
        order = Order(oid, 0, "convoy", now,
                      now + scenario.PARAMS["priority_due_sec"][0],
                      [c.cid for c in idle])
        eng.orders[oid] = order
        # 디포에서 출발하므로 대기열(BB-08)을 앞에 붙인 경로를 부여합니다.
        for i, c in enumerate(idle):
            c.order = order
            c.route = [("BB-08", True)] + list(T.ROUTES["P0_OR1" if i % 2 == 0 else "P0_OR2"])
            c.idx, c.fwd = -1, True
            c.state = CapsuleState.QUEUED      # 발령 틱에 MOVING 으로 전이
            c.req_t = float("inf")

        self.bridge.running = True
        self.bridge._done_notified = False
        self._kpi_sent = False
        res.success = True
        res.message = f"Code Crimson 추가 발령: {oid} — 캡슐 {[c.cid for c in idle]} (디포 → 수술장)"
        self.get_logger().error(f"🔴 {res.message}")
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
