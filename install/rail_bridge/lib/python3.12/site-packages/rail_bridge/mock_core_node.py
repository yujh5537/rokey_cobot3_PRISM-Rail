import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class MockCoreNode(Node):
    def __init__(self):
        super().__init__('mock_core_node')
        # 명령 토픽 발행 (A 역할)
        self.cmd_pub = self.create_publisher(String, '/capsule_cmd', 10)
        # 위치 피드백 구독
        self.pose_sub = self.create_subscription(String, '/capsule_pose', self.pose_callback, 10)
        self.timer = self.create_timer(3.0, self.publish_mock_command)
        self.step = 0
        self.get_logger().info("🚀 [Mock Core] 가상 관제 코어 노드가 시작되었습니다.")

    def publish_mock_command(self):
        actions = ["MOVING", "YIELD_WAIT", "RUN_THROUGH", "EVACUATE", "RESUME"]
        action = actions[self.step % len(actions)]
        
        payload = {
            "capsule_id": "C_1",
            "action": action,
            "target_node": "E1" if action == "EVACUATE" else "N4",
            "block_id": "B08" if action == "EVACUATE" else "B05",
            "timestamp": time.time()
        }
        
        msg = String()
        msg.data = json.dumps(payload)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"📤 [Mock Core -> Cmd 전송] Action: {action} | Data: {msg.data}")
        self.step += 1

    def pose_callback(self, msg):
        self.get_logger().info(f"📥 [Mock Core <- Pose 수신] {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MockCoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
