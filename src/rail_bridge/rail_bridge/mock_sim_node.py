#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MockSimNode(Node):
    def __init__(self):
        super().__init__('mock_sim_node')
        
        # 1. 구독자 설정 (/capsule_cmd 수신)
        self.cmd_sub = self.create_subscription(
            String,
            '/capsule_cmd',
            self.cmd_callback,
            10
        )
        
        # 2. 발행자 설정 (/capsule_pose 역발행)
        self.pose_pub = self.create_publisher(String, '/capsule_pose', 10)
        
        # 3. v3.0 캡슐별 가상 위치 상태 테이블
        self.capsules = {
            f"C-{i:02d}": {"x": 0.0, "y": 0.0, "z": 0.0, "node": "BB-07", "status": "IDLE"}
            for i in range(1, 11)
        }
        
        self.get_logger().info("🎮 [Mock Sim v3.0] 가상 Isaac Sim 시뮬레이터 노드가 시작되었습니다. (캡슐 10대 지원)")

    def cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            kind = data.get("kind")

            # -------------------------------------------------------------
            # [규칙 1] kind == "capsule_state" -> 실제 물리 위치 갱신 및 역발행
            # -------------------------------------------------------------
            if kind == "capsule_state":
                capsule_id = data.get("capsule", "C-01")
                to_state = data.get("to", "MOVING")
                node = data.get("node", "N-W1")
                block = data.get("block", "BB-01")

                # 가상 좌표 갱신 (시뮬레이션 전진)
                cap = self.capsules.get(capsule_id, {"x": 0.0, "y": 0.0, "z": 0.0})
                cap["x"] = round(cap["x"] + 1.2, 2)
                if "2F" in node or "B2-" in block:
                    cap["z"] = 4.0  # 2층 높이
                else:
                    cap["z"] = 0.0  # B1F 높이
                cap["node"] = node
                cap["status"] = to_state

                # /capsule_pose 역발행
                pose_payload = {
                    "capsule_id": capsule_id,
                    "current_block": block,
                    "node": node,
                    "x": cap["x"],
                    "y": cap["y"],
                    "z": cap["z"],
                    "status": to_state
                }
                
                pose_msg = String()
                pose_msg.data = json.dumps(pose_payload, ensure_ascii=False)
                self.pose_pub.publish(pose_msg)
                
                self.get_logger().info(
                    f"📍 [위치 역발행] {capsule_id} | 상태: {to_state} | "
                    f"블록: {block} | 노드: {node} (X:{cap['x']}, Z:{cap['z']})"
                )

            # -------------------------------------------------------------
            # [규칙 2] kind == "preempt" -> 물리 이동 없이 연출/로그만 처리
            # -------------------------------------------------------------
            elif kind == "preempt":
                action = data.get("action")
                target = data.get("target")
                block = data.get("block")
                siding = data.get("siding")
                reason = data.get("reason")
                
                self.get_logger().warn(
                    f"📢 [선점 연출] 액션: {action} | 대상: {target} | "
                    f"구간: {block} (대피로: {siding}) | 사유: {reason}"
                )

        except Exception as e:
            self.get_logger().error(f"❌ 메시지 파싱 오류: {e}")

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