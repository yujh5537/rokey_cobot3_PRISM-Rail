#!/usr/bin/env python3
"""mock_sim — 가상 Isaac Sim (씬 역할). 관제 코어 v3.2 규격.

v3.2 변경 (docs/C연동_매핑표.md §1·§5):
  - 위치의 진실 소스는 관제 코어입니다. 씬은 /capsule_pose 를 **구독**만 하고
    수신한 x/y/z 를 /World/Capsules/{capsule_id} 트랜스폼에 그대로 적용합니다.
    (예전처럼 씬이 좌표를 지어내지 않습니다 — 코어와 씬이 어긋날 여지 제거)
  - /capsule_cmd 는 폐기되었습니다. 선점·대피 연출은 /order_event 로 옵니다.

⚠️ QoS: /capsule_pose 는 코어가 BEST_EFFORT depth1 로 발행합니다.
   RELIABLE 로 구독하면 매칭이 안 되어 **아무것도 오지 않습니다**.
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import String

QOS_POSE = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT)
QOS_EVENT = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE)

SCENE_EVENTS = {"YIELD", "EVAC_LANE", "EVAC_SPUR", "FINISH_ALLOWED",
                "RESUME", "CODE_CRIMSON", "SIM_DONE"}


class MockSimNode(Node):
    def __init__(self):
        super().__init__('mock_sim_node')
        self.create_subscription(String, '/capsule_pose', self.on_pose, QOS_POSE)
        self.create_subscription(String, '/order_event', self.on_event, QOS_EVENT)

        # 프림 상태 캐시 — 상태가 바뀔 때만 로그 (30Hz 스팸 방지)
        self.prims: dict[str, dict] = {}
        self.get_logger().info(
            "🎮 [Mock Sim v3.2] 씬 노드 시작 — /capsule_pose 구독(BEST_EFFORT), "
            "수신 xyz 를 /World/Capsules/* 에 적용")

    # ------------------------------------------------------------------
    def on_pose(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"파싱 에러: {e}")
            return

        for c in data.get("capsules", []):
            cid = c["capsule_id"]
            prim = f"/World/Capsules/{cid}"
            visible = c["state"] != "REMOVED"

            # 실제 Isaac Sim 에서는 이 두 줄이 프림 조작으로 바뀝니다.
            #   set_world_transform(prim, (c["x"], c["y"], c["z"]))
            #   set_visibility(prim, visible)
            prev = self.prims.get(cid)
            self.prims[cid] = {"state": c["state"], "block": c["block_id"],
                               "xyz": (c["x"], c["y"], c["z"]), "visible": visible}

            if prev is None or prev["state"] != c["state"]:
                self.get_logger().info(
                    f"📍 {prim} | {c['state']:<10} | 블록 {c['block_id'] or '-':<7} "
                    f"| xyz=({c['x']}, {c['y']}, {c['z']})"
                    + ("" if visible else "  [프림 숨김]"))

    # ------------------------------------------------------------------
    def on_event(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"파싱 에러: {e}")
            return

        ev = data.get("event", "")
        if ev not in SCENE_EVENTS:
            return
        subj = data.get("subject", "")
        detail = data.get("detail", "")
        if ev in ("YIELD", "EVAC_LANE", "EVAC_SPUR", "FINISH_ALLOWED"):
            self.get_logger().warn(
                f"📢 [연출] {ev} | 대상: {subj} | {detail} | {data.get('reason', '')}")
        else:
            self.get_logger().info(f"🏁 [{ev}] {subj} {detail}")


def main(args=None):
    rclpy.init(args=args)
    node = MockSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
