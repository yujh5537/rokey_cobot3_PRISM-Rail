"""
isaac_twin_m2_bridge.py — CC 콘보이(C01~C04) 단일 슈퍼오더 집결 완결판
- Capsule 01~04 콘보이 간 차두 간격 제한 없음 (단일 이동 폐색으로 동시 진입/초밀착 통과)
- C06, C07 일반 레일 주행 시 전면 90도 오차 보정
- 샤프트 구간(SB-UP, SB-DN) 진입 시 전 캡슐 (0, 0, 90) 강제 수직 정렬
- ROS_DOMAIN_ID=136 및 FastDDS 화이트리스트 강제 동기화
"""

import os
import sys
import json
import math
import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdGeom, Gf

# 1. Isaac Sim 프로세스 환경변수 강제 동기화
TARGET_DOMAIN_ID = "136"
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

# 2. 이전 인스턴스 정리
if "_m2_bridge_sub" in globals() and _m2_bridge_sub is not None:
    try: _m2_bridge_sub.unsubscribe()
    except Exception: pass
    _m2_bridge_sub = None

if "_m2_bridge_node" in globals() and _m2_bridge_node is not None:
    try: _m2_bridge_node.destroy_node()
    except Exception: pass
    _m2_bridge_node = None

# 3. 확정 실측 초기 좌표
Z_OFFSET = 0.0

INITIAL_POSITIONS = {
    "C01": (4.5, 1.0, 3.75), "C02": (5.4, 1.0, 3.75),
    "C03": (4.5, 1.9, 3.75), "C04": (5.4, 1.9, 3.75),
    "C05": (4.5, 2.8, 3.75), "C06": (5.4, 2.8, 3.75),
    "C07": (4.5, 3.7, 3.75), "C08": (5.4, 3.7, 3.75),
    "C09": (4.5, 4.6, 3.75), "C10": (5.4, 4.6, 3.75),
}

# 4. 캡슐 3D 액터 핸들러
class CapsuleActor:
    def __init__(self, stage, prim_path, cid, init_pos):
        self.stage = stage
        self.prim_path = prim_path
        self.cid = cid
        self.prim = stage.GetPrimAtPath(prim_path)

        # C06, C07 전용 90도 헤딩 오차 보정 (C01~C04는 0도)
        self.heading_offset = 90.0 if self.cid in ("C06", "C07") else 0.0

        if not self.prim.IsValid():
            print(f"⚠️ [경고] {prim_path} 를 찾지 못했습니다.")
            return

        # 자식 Prim 로컬 오프셋 초기화
        for child_name in ["Body", "Model"]:
            child_prim = stage.GetPrimAtPath(f"{prim_path}/{child_name}")
            if child_prim.IsValid():
                child_xform = UsdGeom.Xformable(child_prim)
                for op in child_xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
                    elif op.GetOpType() in (UsdGeom.XformOp.TypeRotateX, UsdGeom.XformOp.TypeRotateY, UsdGeom.XformOp.TypeRotateZ):
                        op.Set(0.0)
                    elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        op.Set(Gf.Vec3d(0.0, 0.0, 0.0))

        self.xform = UsdGeom.Xformable(self.prim)
        self.translate_op = None
        self.rotate_op = None

        for op in self.xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                self.translate_op = op
            elif op.GetOpType() in (UsdGeom.XformOp.TypeRotateXYZ, UsdGeom.XformOp.TypeRotateZ):
                self.rotate_op = op

        if not self.translate_op:
            self.translate_op = self.xform.AddTranslateOp()
        if not self.rotate_op:
            self.rotate_op = self.xform.AddRotateXYZOp()

        self.init_translate = Gf.Vec3d(*init_pos)
        self.prev_pos = None
        self.yaw = self.heading_offset
        self.reset()

    def reset(self):
        if not self.prim.IsValid(): return
        if self.translate_op and self.init_translate:
            self.translate_op.Set(self.init_translate)
        if self.rotate_op:
            op_type = self.rotate_op.GetOpType()
            if op_type == UsdGeom.XformOp.TypeRotateXYZ:
                self.rotate_op.Set(Gf.Vec3d(0.0, 0.0, self.heading_offset))
            else:
                try: self.rotate_op.Set(self.heading_offset)
                except: self.rotate_op.Set(Gf.Vec3d(0.0, 0.0, self.heading_offset))
        self.prev_pos = None
        self.yaw = self.heading_offset
        self.set_visible(True)

    def set_visible(self, visible):
        if not self.prim.IsValid(): return
        imageable = UsdGeom.Imageable(self.prim)
        imageable.MakeVisible() if visible else imageable.MakeInvisible()

    def update_pose(self, block_id, x, y, z, fwd=True):
        if not self.prim.IsValid(): return

        curr = (x, y, z)
        b_id = str(block_id).upper().replace("_", "-")

        # 1) 샤프트 구간 (SB-UP, SB-DN): 전 캡슐 (0, 0, 90) 수직 정렬 강제
        if "SB" in b_id or "SHAFT" in b_id:
            rot_x = 0.0
            rot_y = 0.0
            rot_z = 90.0
        # 2) 일반 레일 구간: 방향 추종 (C01~C04는 관제 실측 델타를 그대로 추종)
        else:
            rot_x = 0.0
            rot_y = 0.0
            if self.prev_pos is not None:
                dx = curr[0] - self.prev_pos[0]
                dy = curr[1] - self.prev_pos[1]
                dist = math.hypot(dx, dy)
                if dist > 0.001:
                    self.yaw = math.degrees(math.atan2(dy, dx)) + self.heading_offset
            rot_z = self.yaw

        # 위치 갱신
        self.translate_op.Set(Gf.Vec3d(curr[0], curr[1], curr[2] + Z_OFFSET))

        op_type = self.rotate_op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeRotateXYZ:
            self.rotate_op.Set(Gf.Vec3d(rot_x, rot_y, rot_z))
        else:
            try: self.rotate_op.Set(rot_z)
            except Exception: self.rotate_op.Set(Gf.Vec3d(rot_x, rot_y, rot_z))

        self.prev_pos = curr

# 5. ROS 2 브릿지 노드
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

        self.sub1 = self.create_subscription(String, "/capsule_pose", self.on_msg, qos_best)
        self.sub2 = self.create_subscription(String, "/capsule_pose", self.on_msg, qos_rel)
        self.sub3 = self.create_subscription(String, "/control_state", self.on_msg, qos_best)
        self.sub4 = self.create_subscription(String, "/control_state", self.on_msg, qos_rel)

        self.rx_cnt = 0
        print("✅ [M2 고정 브릿지] CC 콘보이(C01~C04) 차두 해제 및 단일 슈퍼오더 집결 완결판 준비 완료!")

    def on_msg(self, msg):
        self.rx_cnt += 1
        if self.rx_cnt % 30 == 1:
            print(f"📡 [수신 정상] 실시간 텔레메트리 수신 중 (프레임 #{self.rx_cnt})")
        try:
            data = json.loads(msg.data)
            caps_list = data.get("capsules", [data]) if isinstance(data, dict) else []
            for c in caps_list:
                cid = c.get("capsule_id", "").replace("-", "")
                if cid in self.actors:
                    actor = self.actors[cid]
                    st = c.get("state", "")
                    bid = c.get("block_id") or c.get("block")
                    if st in ("REMOVED", "DOCKED") and not bid:
                        actor.set_visible(False)
                    else:
                        actor.set_visible(True)
                        if bid and "x" in c:
                            actor.update_pose(bid, float(c["x"]), float(c["y"]), float(c["z"]), c.get("forward", True))
        except Exception as e:
            pass

# 6. 실행 및 렌더 루프 바인딩
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