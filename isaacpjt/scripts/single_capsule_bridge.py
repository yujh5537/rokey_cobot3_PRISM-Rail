"""
test_single_bridge.py — USD 캡슐(Capsule_ClearanceTest) 단독 연동 안정화 버전
- USD 타입 충돌(double3 vs float3) 완벽 해결
- Stage의 캡슐 1대와 관제 코어 C07 실시간 연동
"""

import json
import math
import omni.usd
import omni.kit.app
from pxr import UsdGeom, Gf

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ==============================================================================
# 0. 이전 리스너 및 노드 정리
# ==============================================================================
if "_test_bridge_sub" in globals() and _test_bridge_sub is not None:
    try:
        _test_bridge_sub.unsubscribe()
    except Exception:
        pass
    _test_bridge_sub = None

if "_test_bridge_node" in globals() and _test_bridge_node is not None:
    try:
        _test_bridge_node.destroy_node()
    except Exception:
        pass
    _test_bridge_node = None

# ==============================================================================
# 1. 설정
# ==============================================================================
STAGE_PRIM_PATH = "/World/Capsules/Capsule_ClearanceTest"
TRACK_CID = "C04"  # 4초 후 SP-PHM(약제부)에서 즉시 출발

CAPSULE_HEIGHT_M = 0.50
Z_OFFSET_M = -(CAPSULE_HEIGHT_M / 2.0)  # -0.25m
CAPSULE_SCALE = Gf.Vec3d(0.60, 0.25, 0.50)  # double3 규격 호환

# (기존 좌표계 및 경로 데이터는 코어에서 /capsule_pose로 x,y,z를 직접 전달하도록 v3.2에서 변경되어 삭제됨)

# ==============================================================================
# 2. 액추에이터 제어 (안전한 Xform Op 핸들링)
# ==============================================================================
class SingleCapsuleActor:
    def __init__(self, stage, prim_path):
        self.stage = stage
        self.prim_path = prim_path
        self.prim = self.stage.GetPrimAtPath(self.prim_path)
        
        if not self.prim.IsValid():
            print(f"⚠️ [{self.prim_path}] 객체가 없어 새로 생성합니다.")
            cube = UsdGeom.Cube.Define(self.stage, self.prim_path)
            cube.GetSizeAttr().Set(1.0)
            self.prim = cube.GetPrim()
        else:
            print(f"✅ 기존 Stage 캡슐 오브젝트 매핑 성공: {self.prim_path}")

        self.xform = UsdGeom.Xformable(self.prim)
        
        # 기존 XformOp 탐색 및 재사용
        self.translate_op = None
        self.rotate_op = None
        self.scale_op = None
        
        for op in self.xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                self.translate_op = op
            elif op.GetOpType() in (UsdGeom.XformOp.TypeRotateZ, UsdGeom.XformOp.TypeRotateXYZ):
                self.rotate_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                self.scale_op = op

        # 누락된 Op만 생성
        if self.translate_op is None:
            self.translate_op = self.xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
        if self.rotate_op is None:
            self.rotate_op = self.xform.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionFloat)
        if self.scale_op is None:
            self.scale_op = self.xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble)
            
        try:
            self.scale_op.Set(CAPSULE_SCALE)
        except Exception:
            pass
            
        self.prev_pos = None
        self.yaw = 0.0

    def set_pose(self, block_id, x, y, z):
        if not self.prim.IsValid(): return
        
        curr = (x, y, z)

        # 샤프트인 경우 회전 없음
        if block_id in ("SB-UP", "SB-DN"):
            self.yaw = 0.0
        elif self.prev_pos is not None:
            dx = curr[0] - self.prev_pos[0]
            dy = curr[1] - self.prev_pos[1]
            dist = math.hypot(dx, dy)
            if dist > 0.001:
                self.yaw = math.degrees(math.atan2(dy, dx))
                
        capsule_pos = Gf.Vec3d(curr[0], curr[1], curr[2] + Z_OFFSET_M)
        self.translate_op.Set(capsule_pos)

        # 안전한 회전 값 적용
        op_type = self.rotate_op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeRotateXYZ:
            curr_rot = self.rotate_op.Get()
            if curr_rot is None:
                curr_rot = (0.0, 0.0, 0.0)
            self.rotate_op.Set(Gf.Vec3d(curr_rot[0], curr_rot[1], self.yaw))
        else:
            try:
                self.rotate_op.Set(self.yaw)
            except Exception:
                self.rotate_op.Set(Gf.Vec3d(0.0, 0.0, self.yaw))
                
        self.prev_pos = curr

# ==============================================================================
# 3. 브릿지 노드
# ==============================================================================
class SingleTestBridgeNode(Node):
    def __init__(self, stage):
        super().__init__("single_test_bridge")
        self.actor = SingleCapsuleActor(stage, STAGE_PRIM_PATH)
        self.sub = self.create_subscription(String, "/capsule_pose", self.callback, 10)
        self.log_count = 0
        print(f"🎯 [연동 대기] USD [{STAGE_PRIM_PATH}] ↔ 관제 [{TRACK_CID}]")

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            for c in data.get("capsules", []):
                cid = c.get("capsule_id", "").replace("-", "")
                if cid == TRACK_CID:
                    block = c.get("block_id")
                    x = c.get("x", 0.0)
                    y = c.get("y", 0.0)
                    z = c.get("z", 0.0)
                    state = c.get("state")
                    
                    self.log_count += 1
                    if self.log_count % 10 == 0:
                        print(f"📡 수신 중! [{TRACK_CID}] State: {state} | Block: {block} | Pos: ({x:.2f}, {y:.2f}, {z:.2f})")

                    if block and state not in ("REMOVED", "DOCKED"):
                        self.actor.set_pose(block, x, y, z)
        except Exception as e:
            print(f"파싱 에러: {e}")

# ==============================================================================
# 4. 실행 및 렌더 루프 바인딩
# ==============================================================================
if not rclpy.ok():
    rclpy.init()

stage = omni.usd.get_context().get_stage()
_test_bridge_node = SingleTestBridgeNode(stage)

app = omni.kit.app.get_app()
update_stream = app.get_update_event_stream()

def on_render_tick(e):
    if rclpy.ok() and _test_bridge_node is not None:
        rclpy.spin_once(_test_bridge_node, timeout_sec=0.0)

_test_bridge_sub = update_stream.create_subscription_to_pop(on_render_tick)
print("🚀 [준비 완료] 관제 코어를 켜면 화면의 Capsule_ClearanceTest가 에러 없이 이동합니다.")