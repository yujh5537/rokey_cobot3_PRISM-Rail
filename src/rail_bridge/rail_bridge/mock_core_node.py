#!/usr/bin/env python3
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MockCoreNode(Node):
    def __init__(self):
        super().__init__('mock_core_node')
        self.pub_cmd = self.create_publisher(String, '/capsule_cmd', 10)
        self.pub_order = self.create_publisher(String, '/order_event', 10)

        # M2 4대 장면 타임라인 (C01 규격 및 code_crimson 명칭 적용)
        self.events = [
            ("capsule_state", "C05", "O-1", "MOVING", "B2-08", "ST-OR1", "", "", ""),
            ("capsule_state", "C06", "O-2", "MOVING", "SP-INJ", "ST-INJ", "", "", ""),
            ("preempt", "C06", "O-2", "YIELD_WAIT", "BB-01", "N-W1", "YIELD", "O-3", "상위 등급 통과 대기 — 진입 전 양보 (R2)"),
            ("preempt", "C07", "O-3", "FINISHING", "SB-UP", "N-W2", "FINISH_ALLOWED", "CR-1", "이미 진입한 블록은 역주행 불가 — 완주 허용 (R4)"),
            ("preempt", "C05", "O-1", "EVACUATED", "B2-04b", "N-L5", "EVACUATE", "CR-1", "Code Crimson 콘보이 회랑 확보를 위해 대피 레인으로 회피 (R3)"),
            ("preempt", "C05", "O-1", "MOVING", "B2-04a", "N-L6", "RESUME", "", "선점 파도 통과 완료 — 주행 재개"),
        ]

        self.idx = 0
        self.timer = self.create_timer(3.0, self.publish_step)
        self.get_logger().info("🚀 [Mock Core] 공지 규격(C01 / code_crimson) 적용 모의 관제 노드 시작")

    def publish_step(self):
        if self.idx >= len(self.events):
            self.get_logger().info("🏁 M2 전체 시나리오 발행 완료. 반복합니다.")
            self.idx = 0
            return

        kind, cid, oid, to_st, blk, node, act, by, reason = self.events[self.idx]
        payload = {
            "sim_t": round(time.time(), 2),
            "kind": kind,
            "capsule": cid,
            "capsule_id": cid,
            "order": oid,
            "order_id": oid,
            "to": to_st,
            "to_state": to_st,
            "block": blk,
            "block_id": blk,
            "node": node,
            "node_id": node,
            "action": act,
            "by": by,
            "code_crimson": True if "CR-" in by else False,
            "reason": reason
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_cmd.publish(msg)

        if kind == "preempt":
            self.get_logger().warn(f"⚡ [선점 액션] {act} | 캡슐: {cid} | 사유: {reason}")
        else:
            self.get_logger().info(f"📤 [상태 전이] 캡슐: {cid} -> {to_st} (블록: {blk})")

        self.idx += 1

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