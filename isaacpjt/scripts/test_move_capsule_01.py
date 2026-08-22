import omni.usd
import omni.kit.app
import omni.timeline
import math
from pxr import UsdGeom, Gf

# 1. 이동 경로 설정 (지하 N-W1 -> 약제부 ST-PHM)
PATH_POINTS = [
    Gf.Vec3d(-7.0, -5.0, 4.0),  # N-W1 (지하 좌측 하단 출발점)
    Gf.Vec3d(-7.0, -2.0, 4.0),  # BB-01 코너 진입
    Gf.Vec3d(-4.5, -2.0, 4.0),  # N-S1
    Gf.Vec3d(-3.5, -2.0, 4.0),  # N-S2
    Gf.Vec3d(-3.5, 4.0, 4.0),   # ST-PHM (약제부 도착점)
]

SPEED = 2.0  # 이동 속도 (m/s)
Z_OFFSET = 0.0

# 2. 캡슐 애니메이터
class SimpleCapsuleAnimator:
    def __init__(self, stage, prim_path):
        self.prim = stage.GetPrimAtPath(prim_path)
        if not self.prim.IsValid():
            print(f"⚠️ {prim_path} 를 찾을 수 없습니다!")
            return
            
        # 📌 자식 객체(Body, Model)의 로컬 Translate 좌표를 (0, 0, 0)으로 강제 정렬
        for child_name in ["Body", "Model"]:
            child_prim = stage.GetPrimAtPath(f"{prim_path}/{child_name}")
            if child_prim.IsValid():
                child_xform = UsdGeom.Xformable(child_prim)
                has_translate = False
                for op in child_xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
                        has_translate = True
                # 만약 기존에 Translate Op가 없다면 새로 추가하여 (0, 0, 0) 고정
                if not has_translate:
                    child_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

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
            self.rotate_op = self.xform.AddRotateZOp()
            
        self.is_playing = False
        self.current_idx = 0
        self.current_pos = PATH_POINTS[0]
        
        self.reset()
        
    def reset(self):
        self.is_playing = False
        self.current_idx = 0
        self.current_pos = PATH_POINTS[0]
        self.apply_pose(self.current_pos, 0.0)
        
    def start(self):
        self.reset()
        self.is_playing = True
        
    def apply_pose(self, pos, yaw):
        self.translate_op.Set(Gf.Vec3d(pos[0], pos[1], pos[2] + Z_OFFSET))
        
        # 안전한 회전 값 적용
        op_type = self.rotate_op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeRotateXYZ:
            curr_rot = self.rotate_op.Get()
            if curr_rot is None:
                curr_rot = Gf.Vec3d(0, 0, 0)
            self.rotate_op.Set(Gf.Vec3d(curr_rot[0], curr_rot[1], yaw))
        else:
            try:
                self.rotate_op.Set(yaw)
            except Exception:
                self.rotate_op.Set(Gf.Vec3d(0, 0, yaw))

    def update(self, dt):
        if not self.is_playing or not self.prim.IsValid() or self.current_idx >= len(PATH_POINTS) - 1:
            return
            
        target = PATH_POINTS[self.current_idx + 1]
        vec = target - self.current_pos
        dist = vec.GetLength()
        
        move_dist = SPEED * dt
        
        if move_dist >= dist:
            # 타겟 노드 도착, 다음 노드로
            self.current_pos = target
            self.current_idx += 1
            if self.current_idx >= len(PATH_POINTS) - 1:
                print("🏁 [테스트] 약제부 도착 완료!")
                self.is_playing = False
        else:
            # 목표 지점 방향으로 이동
            dir_norm = vec.GetNormalized()
            self.current_pos = self.current_pos + dir_norm * move_dist
            
            # 진행 방향에 맞게 앞머리(Yaw) 회전
            yaw = math.degrees(math.atan2(dir_norm[1], dir_norm[0]))
            self.apply_pose(self.current_pos, yaw)


# 3. Isaac Sim 이벤트 연결
stage = omni.usd.get_context().get_stage()
animator = SimpleCapsuleAnimator(stage, "/World/Capsules/Capsule_01")

def on_update(e):
    dt = e.payload["dt"]
    animator.update(dt)

def on_timeline_event(e):
    if e.type == int(omni.timeline.TimelineEventType.PLAY):
        animator.start()
    elif e.type == int(omni.timeline.TimelineEventType.STOP):
        animator.reset()

# 리스너 중복 방지
if "_simple_update_sub" in globals() and _simple_update_sub is not None:
    try: _simple_update_sub.unsubscribe()
    except: pass
if "_simple_timeline_sub" in globals() and _simple_timeline_sub is not None:
    try: _simple_timeline_sub.unsubscribe()
    except: pass

_simple_update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
_simple_timeline_sub = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(on_timeline_event)

print("🚀 [수정 완료] Capsule_01의 Body 및 Model 위치 정렬 후 주행 준비가 완료되었습니다.")