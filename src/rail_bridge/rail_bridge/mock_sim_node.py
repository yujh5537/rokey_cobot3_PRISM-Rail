import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MockSimNode(Node):
    def __init__(self):
        super().__init__('mock_sim_node')
        # 관제 명령 수신
        self.cmd_sub = self.create_subscription(String, '/capsule_cmd', self.cmd_callback, 10)
        # 위치 피드백 발행 (B/C 역할)
        self.pose_pub = self.create_publisher(String, '/capsule_pose', 10)
        self.current_x = 0.0
        self.get_logger().info("🎮 [Mock Sim] 가상 Isaac Sim 시뮬레이터 노드가 시작되었습니다.")

    def cmd_callback(self, msg):
        data = json.loads(msg.data)
        kind = data.get("kind")  # 최상위 kind 필드 확인

        # 1. capsule_state 일 때만 물리 위치 이동
        if kind == "capsule_state":
            capsule_id = data.get("capsule")
            to_state = data.get("to")
            node = data.get("node")
            
            self.current_x += 1.5
            pose_data = {
                "capsule_id": capsule_id,
                "node": node,
                "x": round(self.current_x, 2),
                "y": 0.0,
                "z": 2.0,
                "status": to_state
            }
            pose_msg = String(data=json.dumps(pose_data))
            self.pose_pub.publish(pose_msg)
            self.get_logger().info(f"📍 [위치 갱신] {capsule_id} -> 상태: {to_state}, 노드: {node}")

        # 2. preempt 일 때는 이동하지 않고 로그만 출력
        elif kind == "preempt":
            action = data.get("action")
            target = data.get("target")
            self.get_logger().warn(f"📢 [선점 연출 로그] {action} -> 대상: {target}")

def main(args=None):
    rclpy.init(args=args)
    node = MockSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
