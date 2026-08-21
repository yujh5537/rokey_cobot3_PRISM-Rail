import math
import omni.usd
import omni.timeline
import omni.kit.app
from pxr import UsdGeom, Gf

# ==============================================================================
# 0. 이전 실행 리스너 자동 정리 (중복 실행 방지)
# ==============================================================================
if "_rail_driver_sub" in globals() and _rail_driver_sub is not None:
    _rail_driver_sub.unsubscribe()
    _rail_driver_sub = None

if "_timeline_sub" in globals() and _timeline_sub is not None:
    _timeline_sub = None

print("🛑 [완료] 캡슐 주행 스크립트가 완전히 종료되었습니다.")

# ==============================================================================
# 1. 층별 확정 좌표 테이블 (노션 동결 좌표)
# ==============================================================================
WAYPOINTS_B1F = {
    "N-W1": (-7.0, -5.0, 4.0),
    "N-S1": (-6.0, -2.0, 4.0),
    "N-S2": (-2.0, -2.0, 4.0),
    "N-L1": (0.0, -2.0, 4.0),
    "N-L2": (3.0, -2.0, 4.0),
    "N-E1": (4.0, -2.0, 4.0),
    "N-L3": (4.0, 0.0, 4.0),
    "N-L4": (4.0, 3.0, 4.0),
    "N-D1": (5.0, 4.5, 4.0),
    "N-D2": (7.0, 4.5, 4.0),
    "N-B1": (7.0, -3.0, 4.0),
    "ST-INJ": (-6.0, 4.0, 4.0),
    "ST-PHM": (-2.0, 4.0, 4.0),
}

WAYPOINTS_2F = {
    "N-W2": (-7.0, -5.0, 13.0),
    "N-S3": (-6.0, -2.0, 13.0),
    "N-S4": (-2.0, -2.0, 13.0),
    "N-L5": (0.0, -2.0, 13.0),
    "N-L6": (3.0, -2.0, 13.0),
    "N-E2": (4.0, -2.0, 13.0),
    "N-L7": (4.0, 0.0, 13.0),
    "N-L8": (4.0, 3.0, 13.0),
    "ST-CSR": (-6.0, 4.0, 13.0),
    "ST-ICU": (-2.0, 4.0, 13.0),
    "ST-OR1": (6.0, 4.0, 13.0),
    "ST-OR2": (6.0, -2.0, 13.0),
}

# ==============================================================================
# 2. 캡슐 이동 컨트롤러
# ==============================================================================
class TimelineTrackCapsule:
    def __init__(self, stage, prim_path, route_nodes, waypoint_dict, speed=1.5, loop=True):
        self.stage = stage
        self.prim_path = prim_path
        self.route = route_nodes
        self.waypoints = waypoint_dict
        self.speed = speed
        self.loop = loop
        self.current_idx = 0
        self.progress = 0.0

        self.prim = self.stage.GetPrimAtPath(self.prim_path)
        if not self.prim.IsValid():
            print(f"⚠️ [경고] Prim을 찾을 수 없습니다: {self.prim_path}")
            return

        self.xform = UsdGeom.Xformable(self.prim)
        self.reset_to_start()

    def set_pose(self, pos, direction):
        self.xform.ClearXformOpOrder()
        self.xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
        yaw = math.degrees(math.atan2(direction[1], direction[0]))
        self.xform.AddRotateZOp().Set(yaw)

    def reset_to_start(self):
        """출발 노드 위치로 캡슐을 원복시키는 함수"""
        self.current_idx = 0
        self.progress = 0.0
        start_pos = self.waypoints[self.route[0]]
        
        # 첫 번째 이동 방향으로 회전각 설정
        if len(self.route) > 1:
            p_start = Gf.Vec3d(*self.waypoints[self.route[0]])
            p_next = Gf.Vec3d(*self.waypoints[self.route[1]])
            dir_vec = (p_next - p_start).GetNormalized()
            direction = (dir_vec[0], dir_vec[1], dir_vec[2])
        else:
            direction = (1, 0, 0)
            
        self.set_pose(start_pos, direction)
        print(f"🔄 [{self.prim_path}] 출발 위치({self.route[0]})로 리셋 완료!")

    def update(self, dt):
        if not self.prim.IsValid():
            return

        if self.current_idx >= len(self.route) - 1:
            if self.loop:
                self.current_idx = 0
                self.progress = 0.0
            else:
                return

        p_start = Gf.Vec3d(*self.waypoints[self.route[self.current_idx]])
        p_end = Gf.Vec3d(*self.waypoints[self.route[self.current_idx + 1]])

        seg_vec = p_end - p_start
        seg_len = seg_vec.GetLength()

        if seg_len == 0:
            self.current_idx += 1
            return

        direction = (seg_vec[0] / seg_len, seg_vec[1] / seg_len, seg_vec[2] / seg_len)
        self.progress += (self.speed * dt) / seg_len

        if self.progress >= 1.0:
            self.progress = 0.0
            self.current_idx += 1
            cur_pos = p_end
        else:
            cur_pos = p_start + seg_vec * self.progress

        self.set_pose(cur_pos, direction)

# ==============================================================================
# 3. 설정 및 타임라인 바인딩
# ==============================================================================
stage = omni.usd.get_context().get_stage()

# Prim 경로 설정 (Stage 패널에 있는 캡슐 이름)
TARGET_CAPSULE_PATH = "/World/Capsules/Capsule_ClearanceTest"
SELECTED_FLOOR = "WAYPOINTS_2F"

if SELECTED_FLOOR == "B1F":
    waypoint_data = WAYPOINTS_B1F
    route = ["N-B1", "N-E1", "N-L2", "N-L1", "N-S2", "N-S1", "N-W1", "N-S1", "N-S2", "N-L1", "N-L2", "N-E1", "N-B1"]
else:
    waypoint_data = WAYPOINTS_2F
    route = ["N-W2", "N-S3", "N-S4", "N-L5", "N-L6", "N-E2", "N-L7", "N-L8", "ST-OR1", "N-L8", "N-L7", "N-E2", "N-W2"]

capsule_driver = TimelineTrackCapsule(stage, TARGET_CAPSULE_PATH, route, waypoint_data, speed=2.0, loop=True)

# 타임라인(Play/Stop/Reset) 인터페이스 연결
timeline = omni.timeline.get_timeline_interface()

# 1) Stop 또는 Reset 버튼 누를 때 시작 위치로 원복
def on_timeline_event(e):
    if e.type == int(omni.timeline.TimelineEventType.STOP):
        capsule_driver.reset_to_start()

timeline_stream = timeline.get_timeline_event_stream()
_timeline_sub = timeline_stream.create_subscription_to_pop(on_timeline_event)

# 2) Play(▶) 중일 때만 프레임 이동 진행
app = omni.kit.app.get_app()
update_stream = app.get_update_event_stream()

def on_render_tick(e):
    if timeline.is_playing():
        dt = e.payload.get("dt", 0.016)
        capsule_driver.update(dt)

_rail_driver_sub = update_stream.create_subscription_to_pop(on_render_tick)
print(f"🎬 [타임라인 연동 완료] Play(▶) 시 주행 / Stop(⏹) 시 리셋됩니다.")