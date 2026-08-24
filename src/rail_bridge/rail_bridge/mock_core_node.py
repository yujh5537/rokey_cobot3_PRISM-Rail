#!/usr/bin/env python3
"""mock_core — 관제 코어 대역. 실코어 없이 씬(B)·UI(D) 를 단독으로 붙여볼 때 씁니다.

관제 코어 v3.6 규격(docs/C연동_매핑표.md)으로 발행합니다:
  /capsule_pose  BEST_EFFORT depth1  30Hz  캡슐 위치 + 씬 월드 xyz
  /order_event   RELIABLE   depth50        오더·선점·대피 이벤트
  /block_state   RELIABLE + TRANSIENT_LOCAL depth1  변화 시에만

⚠️ 실코어(rail_control_core) 와 **동시에 켜지 마세요** — 같은 토픽을 두 노드가
   발행해 수신측이 뒤섞인 값을 받습니다. 하나만 띄우세요.

시연 5장면 타임라인을 반복 재생합니다 (좌표는 topology 실측값 근사).
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

QOS_POSE = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT)
QOS_EVENT = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE)
QOS_LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

RATE_HZ = 30.0
STEP_SEC = 3.0                      # 장면 전환 간격

# (event, subject, detail, block, xyz, state) — 발표 5장면 (v3.6)
# 이벤트 이름은 실코어가 내는 것과 동일해야 UI 가 코드를 안 고칩니다.
#
# ⚠️ 아래 수치(ORDER_ARRIVE / SIM_DONE)는 회귀 기준값 BASE_B 와 같아야 합니다.
#    손으로 맞추다 v3.3→v3.5 구간에서 두 번 어긋났고(플랜 B 전환 시 심사 화면에
#    옛 수치가 뜨는 사고), 지금은 test_regression.py 의
#    test_mock_core_timeline_matches_baseline 이 드리프트를 실패로 잡습니다.
#    수치를 고칠 때는 BASE_B 를 고치고 여기를 맞추면 테스트가 확인해 줍니다.
TIMELINE = [
    ("ORDER_RELEASE",  "O-4", "P0",              "C01", "BB-09",  (6.5, -4.0, 4.0),  "MOVING"),
    ("YIELD",          "C06", "BB-01:capacity",  "C06", "SP-INJ", (-4.5, -2.0, 4.0), "YIELD_WAIT"),
    ("FINISH_ALLOWED", "C07", "SB-UP",           "C07", "SB-UP",  (-7.25, -5.0, 8.5), "FINISHING"),
    ("MEET_PASS",      "C05", "B2-07a->B2-07b",  "C05", "B2-07b", (3.2, 2.6, 13.0),  "MOVING"),
    ("EVAC_LANE",      "C08", "B2-05->B2-04b",   "C08", "B2-04b", (-0.7, -1.4, 13.0), "EVACUATED"),
    ("RESUME",         "C08", "B2-03",           "C08", "B2-03",  (1.0, -2.0, 13.0), "MOVING"),
    ("ORDER_ARRIVE",   "O-4", "t=67.07",         "C01", "B2-09",  (4.5, -0.8, 13.0), "UNLOADING"),
    ("SIM_DONE",       "sim", "makespan=85.77",  "C01", "",       (0.0, 0.0, 0.0),   "REMOVED"),
]

REASON = {
    "YIELD": "상위 등급 통과 대기 — 진입 전 양보 (R2)",
    "EVAC_LANE": "Code Crimson 콘보이 회랑 확보를 위해 대피 레인으로 회피 (R3)",
    "MEET_PASS": "대향 캡슐과 교행 — 쌍둥이 대피 레인으로 치환해 스쳐 지나감 (R13)",
    "EVAC_SPUR": "Code Crimson 콘보이 회랑 확보를 위해 지선으로 회피 (R3)",
    "FINISH_ALLOWED": "이미 진입한 블록은 역주행 불가 — 완주 허용 (R4)",
    "RESUME": "선점 파도 통과 완료 — 주행 재개",
}


class MockCoreNode(Node):
    def __init__(self):
        super().__init__('mock_core_node')
        self.pub_pose = self.create_publisher(String, '/capsule_pose', QOS_POSE)
        self.pub_event = self.create_publisher(String, '/order_event', QOS_EVENT)
        self.pub_block = self.create_publisher(String, '/block_state', QOS_LATCHED)

        # 캡슐 10대 초기 상태 — 디포 도크
        self.caps = {f"C{i:02d}": {"block_id": "BB-07", "pos_m": 0.0, "forward": True,
                                   "state": "DOCKED", "order_id": "",
                                   "x": 4.0, "y": round(4.6 - (i - 1) * 0.3, 2), "z": 4.0}
                     for i in range(1, 11)}
        self.sim_t = 0.0
        self.idx = 0
        self._published_blocks = False

        self.create_timer(1.0 / RATE_HZ, self.on_tick)
        self.create_timer(STEP_SEC, self.on_step)
        self.get_logger().info(
            "🚀 [Mock Core v3.2] 관제 코어 대역 시작 — /capsule_pose 30Hz 발행. "
            "실코어와 동시에 켜지 마세요.")

    # ------------------------------------------------------------------
    def on_tick(self):
        self.sim_t = round(self.sim_t + 1.0 / RATE_HZ, 2)
        self.pub_pose.publish(String(data=json.dumps(
            {"sim_t": self.sim_t, "kind": "capsule_pose",
             "capsules": [{"capsule_id": cid, **v} for cid, v in self.caps.items()]},
            ensure_ascii=False)))
        if not self._published_blocks:
            self._publish_blocks()
            self._published_blocks = True

    def _publish_blocks(self):
        blocks = [{"block_id": b, "state": "FREE", "occupancy": 0, "capacity": 1,
                   "locked": False, "corridor": "", "dir": 0, "capsule_ids": []}
                  for b in ("BB-01", "BB-09", "SB-UP", "B2-04a", "B2-04b", "B2-08")]
        self.pub_block.publish(String(data=json.dumps(
            {"sim_t": self.sim_t, "kind": "block_state", "blocks": blocks},
            ensure_ascii=False)))

    # ------------------------------------------------------------------
    def on_step(self):
        if self.idx >= len(TIMELINE):
            self.get_logger().info("🏁 5장면 전체 발행 완료. 반복합니다.")
            self.idx = 0
            return

        ev, subj, detail, cid, blk, (x, y, z), state = TIMELINE[self.idx]
        cap = self.caps[cid]
        cap.update({"block_id": blk, "state": state, "x": x, "y": y, "z": z})
        if ev == "ORDER_RELEASE":
            cap["order_id"] = subj

        payload = {"sim_t": self.sim_t, "kind": "order_event",
                   "event": ev, "subject": subj, "detail": detail}
        if ev in REASON:
            payload.update({"reason": REASON[ev], "by": "O-4", "capsule": subj})
        self.pub_event.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        if ev in REASON and ev != "RESUME":
            self.get_logger().warn(f"⚡ [선점] {ev} | 대상: {subj} | {detail}")
        else:
            self.get_logger().info(f"📤 [{ev}] {subj} {detail}")
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
