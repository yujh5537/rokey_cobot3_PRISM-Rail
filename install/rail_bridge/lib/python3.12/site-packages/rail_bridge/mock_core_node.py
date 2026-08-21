#!/usr/bin/env python3
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MockCoreNode(Node):
    def __init__(self):
        super().__init__('mock_core_node')
        
        # 1. 발행자 설정
        self.cmd_pub = self.create_publisher(String, '/capsule_cmd', 10)
        self.order_pub = self.create_publisher(String, '/order_event', 10)
        
        # 2. 구독자 설정 (C/B의 위치 피드백 수신)
        self.pose_sub = self.create_subscription(String, '/capsule_pose', self.pose_callback, 10)
        
        # 3. v3.0 시연 시나리오 이벤트 타임라인 정의
        self.scenario_steps = [
            # [T=0s] P3 오염기구 (C-05) 수술장1 출발
            {
                "kind": "capsule_state",
                "capsule": "C-05",
                "order": "O-4",
                "to": "MOVING",
                "node": "ST-OR1",
                "block": "B2-08"
            },
            # [T=5s] P2 항암제 (C-02) 주사조제실 출발
            {
                "kind": "capsule_state",
                "capsule": "C-02",
                "order": "O-2",
                "to": "MOVING",
                "node": "ST-INJ",
                "block": "SP-INJ"
            },
            # [T=12s] P1 응급약품 (C-03) 발생 -> P2에 진입 전 양보(YIELD_WAIT) 선점
            {
                "kind": "preempt",
                "action": "YIELD_WAIT",
                "by": "O-3",
                "target": "O-2",
                "block": "SB-UP",
                "reason": "상행 쉬프트 진입 전이므로 정차 후 양보"
            },
            {
                "kind": "capsule_state",
                "capsule": "C-02",
                "order": "O-2",
                "to": "YIELD_WAIT",
                "node": "N-W1",
                "block": "BB-01"
            },
            # [T=20s] P0 Code Crimson 콘보이(C-01~C-04) 발령!
            # - P1(C-03)은 샤프트 내 완주 허용 (RUN_THROUGH)
            # - P3(C-05)은 루프 B2 대피 레인으로 대피 (EVACUATE)
            {
                "kind": "preempt",
                "action": "RUN_THROUGH",
                "by": "CR-1",
                "target": "O-3",
                "block": "SB-UP",
                "reason": "쉬프트 통과 중이므로 완주 허용"
            },
            {
                "kind": "preempt",
                "action": "EVACUATE",
                "by": "CR-1",
                "target": "O-4",
                "block": "B2-07a",
                "siding": "B2-07b",
                "reason": "Code Crimson 회랑 확보를 위해 대피 레인으로 회피"
            },
            {
                "kind": "capsule_state",
                "capsule": "C-05",
                "order": "O-4",
                "to": "EVACUATING",
                "node": "N-L7",
                "block": "B2-07b"
            },
            {
                "kind": "capsule_state",
                "capsule": "C-01",
                "order": "CR-1",
                "to": "MOVING",
                "node": "N-B1",
                "block": "BB-09"
            },
            # [T=40s] Code Crimson 통과 완료 후 대피/대기 캡슐 복귀 (RESUME)
            {
                "kind": "preempt",
                "action": "RESUME",
                "target": "O-4",
                "block": "B2-07a",
                "reason": "Code Crimson 통과 완료 -> 본선 복귀"
            },
            {
                "kind": "capsule_state",
                "capsule": "C-05",
                "order": "O-4",
                "to": "MOVING",
                "node": "N-L8",
                "block": "B2-06"
            }
        ]
        
        self.current_step = 0
        self.timer = self.create_timer(3.5, self.publish_next_event)
        self.get_logger().info("🚀 [Mock Core v3.0] 가상 관제 코어가 시작되었습니다. (Code Crimson 시나리오)")

    def publish_next_event(self):
        if self.current_step >= len(self.scenario_steps):
            self.get_logger().info("🏁 모든 시나리오 이벤트 발행 완료. 처음부터 반복합니다.")
            self.current_step = 0
            return

        event_data = self.scenario_steps[self.current_step]
        event_data["t"] = round(time.time(), 2)

        msg = String()
        msg.data = json.dumps(event_data, ensure_ascii=False)
        self.cmd_pub.publish(msg)

        if event_data.get("kind") == "preempt":
            self.get_logger().warn(
                f"⚡ [선점 명령] {event_data.get('action')} | "
                f"대상: {event_data.get('target')} | 사유: {event_data.get('reason')}"
            )
        else:
            self.get_logger().info(
                f"📤 [상태 변경] 캡슐: {event_data.get('capsule')} -> "
                f"{event_data.get('to')} (노드: {event_data.get('node')})"
            )

        self.current_step += 1

    def pose_callback(self, msg):
        self.get_logger().info(f"📥 [Mock Core <- Pose 수신] {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MockCoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()