"""
control_core_node.py — 엔진을 ROS2로 감싸는 얇은 껍데기(3일차 산출물).

설계 원칙: 이 파일에는 관제 '판단'이 한 줄도 없습니다.
판단은 전부 engine.py에 있고, 여기서는 타이머로 engine.step()을 부르고
결과를 토픽으로 뿌리기만 합니다. 그래야 엔진을 ROS 없이 테스트할 수 있습니다.

발행 (Publish)
--------------
  /order_event   std_msgs/String  (JSON)  오더 상태 변화
  /block_state   std_msgs/String  (JSON)  구간 점유 변화
  /capsule_cmd   std_msgs/String  (JSON)  캡슐 명령 → C(연동)가 Isaac Sim으로 중계
  /control_state std_msgs/String  (JSON)  전체 스냅샷 → D(UI) 대시보드용
  /kpi           std_msgs/String  (JSON)  KPI 요약

서비스 (Service)
----------------
  /code_red        std_srvs/Trigger  P0 응급 수혈 오더 수동 발생 (시연용 버튼)
  /reset           std_srvs/Trigger  시나리오 리셋
  /toggle_mode     std_srvs/Trigger  모드 A ↔ B 전환

왜 커스텀 .msg 를 안 쓰고 String+JSON 인가요?
---------------------------------------------
커스텀 메시지는 별도 CMake 패키지를 만들고 4대 PC 전원이 재빌드해야 합니다.
6일 일정에서는 그 시간이 아깝고, rosbridge로 D에게 넘길 때도 JSON이 그대로 통과합니다.
스키마가 굳은 뒤(5~6일차) 커스텀 메시지로 바꾸는 편이 안전합니다.
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .engine import RailEngine
from .fsm import Order
from .topology import Topology

import yaml

QOS = 10


class ControlCoreNode(Node):
    def __init__(self) -> None:
        super().__init__("control_core_node")

        # ---------- 파라미터 ----------
        pkg_share = Path(__file__).resolve().parent.parent
        default_cfg = str(pkg_share / "config")
        self.declare_parameter("config_dir", default_cfg)
        self.declare_parameter("mode", "B")
        self.declare_parameter("tick_s", 0.1)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("auto_start", True)

        self.cfg_dir = Path(self.get_parameter("config_dir").value)
        self.mode = str(self.get_parameter("mode").value).upper()

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
        self.engine: RailEngine | None = None
        self._build_engine()

        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.on_tick)
        self.running = bool(self.get_parameter("auto_start").value)
        self._kpi_sent = False

        self.get_logger().info(
            f"관제 코어 기동 — 모드 {self.mode}, "
            f"노드 {len(self.engine.topo.nodes)}개 / 블록 {len(self.engine.topo.blocks)}개"
        )

    # ------------------------------------------------------------------
    def _build_engine(self) -> None:
        topo = Topology(self.cfg_dir / "topology.yaml")
        raw = yaml.safe_load((self.cfg_dir / "params.yaml").read_text(encoding="utf-8"))
        params = dict(raw["control_core_node"]["ros__parameters"])
        params["mode"] = self.mode
        params["tick_s"] = float(self.get_parameter("tick_s").value)

        self.engine = RailEngine(topo, params)
        self.engine.load_scenario(self.cfg_dir / "scenario_main.yaml")
        self._kpi_sent = False

    # ------------------------------------------------------------------
    def on_tick(self) -> None:
        if not self.running or self.engine is None:
            return

        scale = float(self.get_parameter("speed_scale").value)
        dt = float(self.engine.p["tick_s"]) * max(0.1, scale)

        for ev in self.engine.step(dt):
            msg = String(data=json.dumps(ev.as_dict(), ensure_ascii=False))
            if ev.kind == "order_event":
                self.pub_order.publish(msg)
            elif ev.kind == "block_state":
                self.pub_block.publish(msg)
            elif ev.kind in ("capsule_state", "preempt"):
                self.pub_cmd.publish(msg)          # C가 Isaac Sim으로 중계
                if ev.kind == "preempt":
                    act = ev.payload.get("action")
                    tgt = ev.payload.get("target")
                    why = ev.payload.get("reason")
                    if act == "RESUME":
                        # 복구는 '누가 선점했다'가 아니라 '길이 열려 스스로 재개'입니다
                        self.get_logger().info(f"[복구] {tgt} 재개 — {why}")
                    else:
                        self.get_logger().warn(
                            f"[선점] {act} {ev.payload.get('by')} → {tgt} ({why})"
                        )

        # 전체 스냅샷은 매 틱 통째로 (D의 대시보드가 재조립할 필요 없게)
        self.pub_state.publish(
            String(data=json.dumps(self.engine.snapshot(), ensure_ascii=False))
        )

        if self.engine.all_done() and not self._kpi_sent:
            kpi = self.engine.kpi()
            self.pub_kpi.publish(String(data=json.dumps(kpi, ensure_ascii=False)))
            self.get_logger().info(f"시나리오 종료 — KPI: {kpi}")
            self._kpi_sent = True
            self.running = False

    # ------------------------------------------------------------------
    # 서비스 콜백
    # ------------------------------------------------------------------
    def on_code_red(self, _req, res):
        """시연 중 아무 때나 P0 응급 수혈 오더를 밀어넣습니다."""
        n = sum(1 for o in self.engine.orders if o.startswith("CR-"))
        oid = f"CR-{n + 1}"
        self.engine.add_order(Order(
            oid=oid, priority=0, item="응급 수혈혈액(Code Red)",
            origin="N1", dest="N4",
            release_s=self.engine.now, due_s=self.engine.now + 25.0,
            cargo_class="clean", code_red=True,
        ))
        self.running = True
        self._kpi_sent = False
        res.success = True
        res.message = f"Code Red 발령: {oid} (혈액은행 → 수술장)"
        self.get_logger().error(f"🔴 {res.message}")
        return res

    def on_reset(self, _req, res):
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
        node.get_logger().info("종료 요청 — 관제 코어를 정리합니다")
    finally:
        node.destroy_node()
        # Jazzy에서는 SIGINT 시 rclpy가 이미 shutdown을 호출한 상태일 수 있습니다.
        # 중복 호출하면 RCLError가 나므로 컨텍스트가 살아있을 때만 정리합니다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
