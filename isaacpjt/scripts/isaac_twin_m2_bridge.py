"""
isaac_twin_m2_bridge.py — M2 고정 경로 (/World/Capsules/Capsule_01~10) 완주 브릿지
- 6번, 7번 포함 "모든 캡슐" 샤프트 구간(SB-UP, SB-DN) 진입 시 (0, 0, 90)도 완벽 고정
- C06, C07 일반 레일 주행 시 90도 전면 오차 보정 추종
- 자식 객체(Body, Model)의 로컬 위치를 (0, 0, 0)으로 정렬
"""

import json
import math
import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdGeom, Gf

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# 0. 이전 실행 정리
if "_m2_bridge_sub" in globals() and _m2_bridge_sub is not None:
    try: _m2_bridge_sub.unsubscribe()
    except: pass
    _m2_bridge_sub = None

if "_m2_bridge_node" in globals() and _m2_bridge_node is not None:
    try: _m2_bridge_node.destroy_node()
    except: pass
    _m2_bridge_node = None

if "_timeline_sub" in globals() and _timeline_sub is not None:
    _timeline_sub = None

# 1. 확정 실측 좌표 및 오프셋
Z_OFFSET = 0.0

INITIAL_POSITIONS = {
    "C01": (4.5, 1.0, 3.75),
    "C02": (5.4, 1.0, 3.75),
    "C03": (4.5, 1.9, 3.75),
    "C04": (5.4, 1.9, 3.75),
    "C05": (4.5, 2.8, 3.75),
    "C06": (5.4, 2.8, 3.75),
    "C07": (4.5, 3.7, 3.75),
    "C08": (5.4, 3.7, 3.75),
    "C09": (4.5, 4.6, 3.75),
    "C10": (5.4, 4.6, 3.75),
}

# 2. 캡슐 제어 객체
class CapsuleActor:
    def __init__(self, stage, prim_path, cid, init_pos):
        self.stage = stage
        self.prim_path = prim_path
        self.cid = cid
        self.prim = stage.GetPrimAtPath(prim_path)
        
        if not self.prim.IsValid():
            print(f"⚠️ [경고] {prim_path} 를 찾지 못했습니다. Stage 패널을 확인하세요.")
            return

        # 자식 객체(Body, Model) 위치 (0,0,0) 및 회전 (0,0,0) 초기화
        for child_name in ["Body", "Model"]:
            child_prim = stage.GetPrimAtPath(f"{prim_path}/{child_name}")
            if child_prim.IsValid():
                child_xform = UsdGeom.Xformable(child_prim)
                
                # Translate 정렬 (0, 0, 0)
                has_translate = False
                for op in child_xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
                        has_translate = True
                if not has_translate:
                    child_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
                
                # 회전 (0, 0, 0) 초기화
                for op in child_xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeRotateX:
                        op.Set(0.0)
                    elif op.GetOpType() == UsdGeom.XformOp.TypeRotateY:
                        op.Set(0.0)
                    elif op.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
                        op.Set(0.0)
                    elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        op.Set(Gf.Vec3d(0.0, 0.0, 0.0))

        self.xform = UsdGeom.Xformable(self.prim)
        self.translate_op = None
        self.rotate_op = None
        
        for op in self.xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                self.translate_op = op
            elif op.GetOpType() in (UsdGeom.XformOp.TypeRotateZ, UsdGeom.XformOp.TypeRotateXYZ):
                self.rotate_op = op
                
        if not self.translate_op:
            self.translate_op = self.xform.AddTranslateOp()
        if not self.rotate_op:
            self.rotate_op = self.xform.AddRotateXYZOp()
            
        self.init_translate = Gf.Vec3d(*init_pos)
        self.init_visibility = UsdGeom.Imageable(self.prim).ComputeVisibility()
            
        self.prev_pos = None
        self.yaw = 0.0
        
        self.reset()

    def reset(self):
        if not self.prim.IsValid(): return
        if self.translate_op and self.init_translate is not None:
            self.translate_op.Set(self.init_translate)
        
        # 기본 대기 상태 회전 설정
        if self.rotate_op:
            op_type = self.rotate_op.GetOpType()
            if op_type == UsdGeom.XformOp.TypeRotateXYZ:
                self.rotate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
            else:
                try: self.rotate_op.Set(0.0)
                except: self.rotate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
        
        if self.init_visibility == "invisible":
            UsdGeom.Imageable(self.prim).MakeInvisible()
        else:
            UsdGeom.Imageable(self.prim).MakeVisible()
            
        self.prev_pos = None
        self.yaw = 0.0

    def set_visible(self, visible):
        if not self.prim.IsValid(): return
        UsdGeom.Imageable(self.prim).MakeVisible() if visible else UsdGeom.Imageable(self.prim).MakeInvisible()

    def update_pose(self, block_id, x, y, z, fwd=True):
        if not self.prim.IsValid(): return

        curr = (x, y, z)
        b_id = str(block_id).upper().replace("_", "-")

        # 📌 1) 샤프트 구간 (SB-UP / SB-DN): 6번, 7번 포함 '모든 캡슐' 무조건 (0, 0, 90)도 고정!
        if "SB" in b_id or "SHAFT" in b_id:
            rot_x = 0.0
            rot_y = 0.0
            rot_z = 90.0
        # 📌 2) 일반 주행 구간: 방향 추종
        else:
            rot_x = 0.0
            rot_y = 0.0
            if self.prev_pos is not None:
                dx = curr[0] - self.prev_pos[0]
                dy = curr[1] - self.prev_pos[1]
                dist = math.hypot(dx, dy)
                if dist > 0.001:
                    calc_yaw = math.degrees(math.atan2(dy, dx))
                    if not fwd:
                        calc_yaw += 180.0
                    self.yaw = calc_yaw
            rot_z = self.yaw

        # 위치 갱신
        self.translate_op.Set(Gf.Vec3d(curr[0], curr[1], curr[2] + Z_OFFSET))
        
        # 회전 갱신
        op_type = self.rotate_op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeRotateXYZ:
            self.rotate_op.Set(Gf.Vec3d(rot_x, rot_y, rot_z))
        else:
            try:
                self.rotate_op.Set(rot_z)
            except Exception:
                self.rotate_op.Set(Gf.Vec3d(rot_x, rot_y, rot_z))
                
        self.prev_pos = curr

# 3. ROS 2 노드
class FixedBridgeNode(Node):
    def __init__(self, stage):
        super().__init__("fixed_bridge_node")
        self.actors = {}
        for i in range(1, 11):
            cid = f"C{i:02d}"
            prim_path = f"/World/Capsules/Capsule_{i:02d}"
            init_pos = INITIAL_POSITIONS[cid]
            self.actors[cid] = CapsuleActor(stage, prim_path, cid, init_pos)
            
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, "/capsule_pose", self.on_state, qos)
        print("✅ [M2 고정 브릿지] 6·7번 포함 전 캡슐 샤프트 (0,0,90) 적용 완료!")
        
        self.last_msg_time = self.get_clock().now()
        self.is_reset = False
        self.timeout_timer = self.create_timer(1.0, self.check_timeout)

    def reset_all(self):
        for actor in self.actors.values():
            actor.reset()

    def check_timeout(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 2.0 and not self.is_reset:
            print("⏳ [M2 브릿지] 관제 신호 끊김! 초기 위치로 복귀합니다.")
            self.reset_all()
            self.is_reset = True

    def on_state(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.is_reset = False
        try:
            data = json.loads(msg.data)
            for c in data.get("capsules", []):
                try:
                    cid = c.get("capsule_id", "").replace("-", "")
                    if cid in self.actors:
                        actor = self.actors[cid]
                        st = c.get("state")
                        bid = c.get("block_id")
                        if st in ("REMOVED", "DOCKED") and not bid:
                            actor.set_visible(False)
                        else:
                            actor.set_visible(True)
                            if bid:
                                x = c.get("x", 0.0)
                                y = c.get("y", 0.0)
                                z = c.get("z", 0.0)
                                fwd = c.get("forward", True)
                                actor.update_pose(bid, x, y, z, fwd)
                except Exception as e:
                    print(f"⚠️ [M2 브릿지] 캡슐 {c.get('capsule_id')} 업데이트 실패: {e}")
        except Exception as e:
            print(f"⚠️ [M2 브릿지] 메시지 파싱 실패: {e}")

# 4. 실행 등록
if not rclpy.ok():
    rclpy.init()

stage = omni.usd.get_context().get_stage()
_m2_bridge_node = FixedBridgeNode(stage)

def on_render_update(e):
    if rclpy.ok() and _m2_bridge_node is not None:
        rclpy.spin_once(_m2_bridge_node, timeout_sec=0.0)

_m2_bridge_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_render_update)

def on_timeline_event(e):
    if e.type == int(omni.timeline.TimelineEventType.STOP):
        if "_m2_bridge_node" in globals() and _m2_bridge_node is not None:
            _m2_bridge_node.reset_all()

_timeline_sub = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(on_timeline_event)