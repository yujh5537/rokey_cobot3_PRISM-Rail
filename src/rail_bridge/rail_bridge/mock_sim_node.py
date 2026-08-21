#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MockSimNode(Node):
    def __init__(self):
        super().__init__('mock_sim_node')
        self.sub_cmd = self.create_subscription(String, '/capsule_cmd', self.on_cmd, 10)
        self.pub_pose = self.create_publisher(String, '/capsule_pose', 10)
        self.caps = {f"C{i:02d}": {"x": 0.0, "y": 0.0, "z": 4.0, "block_id": "BB-07", "state": "DOCKED"} for i in range(1, 11)}
        self.get_logger().info("🎮 [Mock Sim] JSON String 호환 모의 시뮬레이터 시작")

    def on_cmd(self, msg: String):
        try:
            data = json.loads(msg.data)
            cid = data.get("capsule") or data.get("capsule_id") or data.get("target") or "C01"
            cid = cid.replace("-", "")
            kind = data.get("kind")
            
            if kind == "capsule_state" or "to" in data:
                blk = data.get("block") or data.get("block_id", "")
                to_st = data.get("to") or data.get("to_state", "MOVING")
                cap = self.caps.get(cid, {"x": 0.0, "y": 0.0, "z": 4.0})
                cap["x"] += 1.0
                cap["z"] = 13.0 if "B2-" in blk or "2F" in data.get("node", "") else 4.0
                cap["block_id"] = blk
                cap["state"] = to_st

                pose_data = {
                    "capsule_id": cid,
                    "block_id": blk,
                    "node_id": data.get("node") or data.get("node_id", ""),
                    "x": round(cap["x"], 2),
                    "y": 0.0,
                    "z": cap["z"],
                    "state": to_st
                }
                out = String()
                out.data = json.dumps(pose_data, ensure_ascii=False)
                self.pub_pose.publish(out)
                self.get_logger().info(f"📍 [위치 보고] {cid} -> {blk} (X:{pose_data['x']}, Z:{pose_data['z']})")
            elif kind == "preempt":
                act = data.get("action")
                reason = data.get("reason")
                self.get_logger().warn(f"📢 [선점 수신] 액션: {act} | 대상: {cid} | 사유: {reason}")
        except Exception as e:
            self.get_logger().error(f"파싱 에러: {e}")

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