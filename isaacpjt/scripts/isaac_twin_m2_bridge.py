"""
isaac_twin_m2_bridge.py — ROS_DOMAIN_ID 강제 주입 및 수신 보장형 최종 브릿지
"""
import os
import sys
import json
import math
import omni.usd
import omni.kit.app
from pxr import UsdGeom, Gf

# 1. [핵심] Isaac Sim 내부 환경변수 강제 동기화 (Domain 136 및 FastDDS)
TARGET_DOMAIN_ID = "136"  # PC A에서 export한 값과 동일
os.environ["ROS_DOMAIN_ID"] = TARGET_DOMAIN_ID
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["ROS_LOCALHOST_ONLY"] = "0"

whitelist_path = os.path.expanduser("~/.ros/fastdds_whitelist.xml")
if os.path.exists(whitelist_path):
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = whitelist_path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# 2. 이전 인스턴스 완전 초기화
if "_m2_bridge_sub" in globals() and _m2_bridge_sub is not None:
    try: _m2_bridge_sub.unsubscribe()
    except Exception: pass
    _m2_bridge_sub = None

if "_m2_bridge_node" in globals() and _m2_bridge_node is not None:
    try: _m2_bridge_node.destroy_node()
    except Exception: pass
    _m2_bridge_node = None

INITIAL_POSITIONS = {
    "C01": (4.5, 1.0, 3.75), "C02": (5.4, 1.0, 3.75),
    "C03": (4.5, 1.9, 3.75), "C04": (5.4, 1.9, 3.75),
    "C05": (4.5, 2.8, 3.75), "C06": (5.4, 2.8, 3.75),
    "C07": (4.5, 3.7, 3.75), "C08": (5.4, 3.7, 3.75),
    "C09": (4.5, 4.6, 3.75), "C10": (5.4, 4.6, 3.75),
}

class CapsuleActor:
    def __init__(self, stage, prim_path, cid, init_pos):
        self.stage = stage
        self.prim_path = prim_path
        self.cid = cid
        self.prim = stage.GetPrimAtPath(prim_path)
        if not self.prim.IsValid():
            print(f"⚠️ [경고] {prim_path} 없음")
            return
        self.xform = UsdGeom.Xformable(self.prim)
        self.translate_op = None
        self.rotate_op = None
        for op in self.xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate: self.translate_op = op
            elif op.GetOpType() in (UsdGeom.XformOp.TypeRotateXYZ, UsdGeom.XformOp.TypeRotateZ): self.rotate_op = op
        if not self.translate_op: self.translate_op = self.xform.AddTranslateOp()
        if not self.rotate_op: self.rotate_op = self.xform.AddRotateXYZOp()
        self.prev_pos = None
        self.yaw = 0.0

    def update_pose(self, block_id, x, y, z):
        if not self.prim.IsValid(): return
        curr = (x, y, z)
        b_id = str(block_id).upper().replace("_", "-")
        if "SB" in b_id or "SHAFT" in b_id:
            rot_z = 0.0
        else:
            if self.prev_pos is not None:
                dx = curr[0] - self.prev_pos[0]
                dy = curr[1] - self.prev_pos[1]
                if math.hypot(dx, dy) > 0.001:
                    self.yaw = math.degrees(math.atan2(dy, dx))
            rot_z = self.yaw

        self.translate_op.Set(Gf.Vec3d(curr[0], curr[1], curr[2]))
        op_type = self.rotate_op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeRotateXYZ:
            self.rotate_op.Set(Gf.Vec3d(0.0, 0.0, rot_z))
        else:
            try: self.rotate_op.Set(rot_z)
            except Exception: self.rotate_op.Set(Gf.Vec3d(0.0, 0.0, rot_z))
        self.prev_pos = curr

class FixedBridgeNode(Node):
    def __init__(self, stage):
        super().__init__("fixed_bridge_node")
        self.actors = {}
        for i in range(1, 11):
            cid = f"C{i:02d}"
            prim_path = f"/World/Capsules/Capsule_{i:02d}"
            self.actors[cid] = CapsuleActor(stage, prim_path, cid, INITIAL_POSITIONS[cid])

        qos_best = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
        
        # QoS 및 토픽명 다중 바인딩
        self.sub1 = self.create_subscription(String, "/capsule_pose", self.on_msg, qos_best)
        self.sub2 = self.create_subscription(String, "/capsule_pose", self.on_msg, qos_rel)
        self.sub3 = self.create_subscription(String, "/control_state", self.on_msg, qos_best)
        self.sub4 = self.create_subscription(String, "/control_state", self.on_msg, qos_rel)
        
        self.rx_cnt = 0
        print(f"✅ [M2 브릿지 가동] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} 수신 대기 중...")

    def on_msg(self, msg):
        self.rx_cnt += 1
        if self.rx_cnt % 30 == 1:
            print(f"📡 [수신 성공!] 데이터 수신 중 (프레임 #{self.rx_cnt})")
        try:
            data = json.loads(msg.data)
            caps_list = data.get("capsules", [data]) if isinstance(data, dict) else []
            for c in caps_list:
                cid = c.get("capsule_id", "").replace("-", "")
                if cid in self.actors:
                    bid = c.get("block_id") or c.get("block")
                    if bid and "x" in c:
                        self.actors[cid].update_pose(bid, float(c["x"]), float(c["y"]), float(c["z"]))
        except Exception as e:
            print(f"⚠️ [M2 브릿지] 에러: {e}")

# rclpy 강제 리셋 후 재생성
if rclpy.ok():
    rclpy.shutdown()
rclpy.init()

stage = omni.usd.get_context().get_stage()
_m2_bridge_node = FixedBridgeNode(stage)

app = omni.kit.app.get_app()
update_stream = app.get_update_event_stream()

def on_render_update(e):
    if rclpy.ok() and _m2_bridge_node is not None:
        rclpy.spin_once(_m2_bridge_node, timeout_sec=0.0)

_m2_bridge_sub = update_stream.create_subscription_to_pop(on_render_update)