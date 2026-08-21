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
        capsule_id = data.get("capsule_id")
        action = data.get("action")
        self.get_logger().info(f"📩 [Mock Sim <- Cmd 수신] Capsule: {capsule_id}, Action: {action}")

        # 수신 반응 위치 피드백 발행
        self.current_x += 1.5
        pose_data = {
            "capsule_id": capsule_id,
            "current_block": data.get("block_id", "B01"),
            "x": round(self.current_x, 2),
            "y": 0.0,
            "z": 2.0,
            "status": action
        }
        pose_msg = String()
        pose_msg.data = json.dumps(pose_data)
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f"📍 [Mock Sim -> Pose 역발행] {pose_msg.data}")

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
