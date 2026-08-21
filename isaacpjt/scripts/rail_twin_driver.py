# 레일 주행 & 시각화 스크립트

import math
import omni.usd
import omni.kit.app
from pxr import UsdGeom, Gf

# ==============================================================================
# 1. 노션 확정 좌표 테이블 (World Coordinates)
# ==============================================================================
WAYPOINTS = {
    # --- B1F 노드 (Z = 4.0) ---
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

    # --- 2F 노드 (Z = 13.0) ---
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
# 2. 캡슐 트랙 추종 컨트롤러
# ==============================================================================
class TrackCapsule:
    def __init__(self, stage, capsule_id, route_nodes, speed=2.0):
        self.stage = stage
        self.capsule_id = capsule_id
        self.route = route_nodes
        self.speed = speed
        self.current_idx = 0
        self.progress = 0.0
        
        self.prim_path = f"/World/Capsules/{capsule_id}"
        self.prim = self.stage.GetPrimAtPath(self.prim_path)
        
        # 캡슐이 없으면 큐브 프림으로 자동 생성
        if not self.prim.IsValid():
            cube = UsdGeom.Cube.Define(self.stage, self.prim_path)
            cube.GetSizeAttr().Set(0.5)
            self.prim = cube.GetPrim()
            
        self.xform = UsdGeom.Xformable(self.prim)
        self.set_pose(WAYPOINTS[self.route[0]], (1, 0, 0))

    def set_pose(self, pos, direction):
        self.xform.ClearXformOpOrder()
        self.xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
        
        # 진행 방향에 따른 Yaw 회전 계산
        yaw = math.degrees(math.atan2(direction[1], direction[0]))
        self.xform.AddRotateZOp().Set(yaw)

    def set_route(self, new_route):
        """관제 명령(대피/경로변경) 수신 시 실시간 경로 갈아타기"""
        self.route = new_route
        self.current_idx = 0
        self.progress = 0.0

    def update(self, dt):
        if self.current_idx >= len(self.route) - 1:
            return

        p_start = Gf.Vec3d(*WAYPOINTS[self.route[self.current_idx]])
        p_end = Gf.Vec3d(*WAYPOINTS[self.route[self.current_idx + 1]])
        
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
# 3. 레일 선로 3D 시각화 (BasisCurves 생성)
# ==============================================================================
def create_rail_lines(stage):
    # 주요 연결 블록 선로 리스트
    rail_segments = [
        # B1F 본선 및 지선
        ["N-W1", "N-S1", "N-S2", "N-L1", "N-L2", "N-E1"],
        ["N-S1", "ST-INJ"],
        ["N-S2", "ST-PHM"],
        ["N-E1", "N-L3", "N-L4", "N-D1", "N-D2", "N-B1", "N-E1"],
        # 층간 쉬프트
        ["N-W1", "N-W2"],
        # 2F 본선 및 지선
        ["N-W2", "N-S3", "N-S4", "N-L5", "N-L6", "N-E2"],
        ["N-S3", "ST-CSR"],
        ["N-S4", "ST-ICU"],
        ["N-E2", "N-L7", "N-L8", "ST-OR1"],
        ["N-E2", "ST-OR2"],
    ]

    for idx, path in enumerate(rail_segments):
        curve_path = f"/World/Rails/Rail_Segment_{idx:02d}"
        curves = UsdGeom.BasisCurves.Define(stage, curve_path)
        curves.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
        
        points = [Gf.Vec3f(*WAYPOINTS[node]) for node in path]
        curves.CreatePointsAttr().Set(points)
        curves.CreateCurveVertexCountsAttr().Set([len(points)])
        curves.CreateWidthsAttr().Set([0.08] * len(points)) # 두께 8cm


# ==============================================================================
# 4. 시뮬레이션 기동 (Code Crimson P0 주행 테스트)
# ==============================================================================
stage = omni.usd.get_context().get_stage()

# 1. 선로 그리기
create_rail_lines(stage)

# 2. P0 메인 경로: 혈액은행(N-B1) -> 수술실1(ST-OR1)
# 경로: N-B1 -> N-E1 -> N-L2 -> N-L1 -> N-S2 -> N-S1 -> N-W1 -> N-W2 -> N-S3 -> N-S4 -> N-L5 -> N-L6 -> N-E2 -> N-L7 -> N-L8 -> ST-OR1
p0_route = [
    "N-B1", "N-E1", "N-L2", "N-L1", "N-S2", "N-S1", "N-W1",
    "N-W2", "N-S3", "N-S4", "N-L5", "N-L6", "N-E2", "N-L7", "N-L8", "ST-OR1"
]

capsule_c01 = TrackCapsule(stage, "C-01", p0_route, speed=2.5)

# Isaac Sim 프레임 렌더 루프 연결
app = omni.kit.app.get_app()
update_stream = app.get_update_event_stream()

def on_tick(e):
    dt = e.payload.get("dt", 0.016)
    capsule_c01.update(dt)

sub = update_stream.create_subscription_to_pop(on_tick)
print("✅ [성공] 노션 확정 좌표가 적용된 레일 트랙 및 C-01 주행이 활성화되었습니다.")