import omni.kit.app
import numpy as np
from omni.isaac.core.articulations import Articulation

# ==============================================================================
# 1. 사용자 커스텀 설정 영역
# ==============================================================================
ROBOT_PRIM_PATH = "/World/m0609" # Stage 내 M0609 경로 확인 필요

# [조인트 이름 정의]
# M0609의 6개 조인트 이름과 그리퍼 핑거 조인트 이름을 명시합니다.
# (본인의 USD 씬에 설정된 실제 조인트 이름과 다를 경우 아래 문자열을 수정하세요)
JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",  # 로봇 팔 조인트 (1~6)
    "finger_joint"                     # 그리퍼 핑거 조인트 (예시 이름)
]

# [조인트 이동 순서 시퀀스]
# 각 스텝별로 [J1, J2, J3, J4, J5, J6, Finger1, Finger2]의 목표 각도(Degree)를 순서대로 정의합니다.
# 핑거 조인트의 열고 닫는 각도(또는 거리)는 그리퍼 스펙에 맞게 조절하세요.
JOINT_STEPS = [
    {"angles": [100.8, 3.9, -4.1, -3.0, 80.1, -61.9, 0.0], "duration": 2.0},     # Step 0: 초기 자세 (그리퍼 열림/기본)
    {"angles": [76.6, 3.9, 79.5, -3.0, 80.1, -61.9, 0.0], "duration": 2.0},   # Step 1: 1번 조인트만 30도 회전
    {"angles": [52.4, 21.2, 33.4, -3.0, 100.8, -61.9, 19.9], "duration": 5.0}, # Step 2: 2, 3번 조인트 순차 회전
    {"angles": [287.8, 3.9, 79.5, -3.0, 80.1, -61.9, 19.0], "duration": 5.0},# Step 3: 목표물 도달 후 핑거 조인트 작동 (그리퍼 닫힘)
    {"angles": [166.6, 3.9, 0.2, -3.0, 83.5, -61.9, 0.0], "duration": 2.0},
    {"angles": [76.6, 3.9, 79.5, -3.0, 80.1, -61.9, 0.0], "duration": 2.0},     # Step 4: 원위치 복귀
]

# ==============================================================================
# 2. 순차 제어 컨트롤러 클래스
# ==============================================================================
class M0609WithGripperController:
    def __init__(self, prim_path, joint_names, steps):
        self.robot = Articulation(prim_path=prim_path)
        self.robot.initialize()
        
        self.steps = steps
        self.current_step_idx = 0
        self.time_in_step = 0.0
        self.is_finished = False
        
        # M0609 전체 조인트 중 우리가 제어할 조인트들의 인덱스 매핑 찾기
        all_joint_names = self.robot.get_joint_names()
        self.target_indices = []
        for name in joint_names:
            if name in all_joint_names:
                self.target_indices.append(all_joint_names.index(name))
            else:
                print(f"⚠️ [경고] 조인트 '{name}'를 로봇에서 찾을 수 없습니다. 이름을 확인하세요.")

        print(f"🤖 [M0609 + Finger] 제어 대상 조인트 인덱스 매핑 완료: {self.target_indices}")
        
        # 현재 조인트 각도 가져오기 (전체 조인트 기준)
        current_full_positions = self.robot.get_joint_positions()
        
        # 시작 앵글 설정 (지정된 조인트들만 추출)
        self.start_angles = np.array([current_full_positions[i] for i in self.target_indices])
        self.target_angles = np.radians(self.steps[0]["angles"])
        self.step_duration = self.steps[0]["duration"]

    def update(self, dt):
        if self.is_finished or len(self.steps) == 0:
            return

        self.time_in_step += dt
        progress = self.time_in_step / self.step_duration

        if progress >= 1.0:
            progress = 1.0
            # 현재 스텝 완료 후 다음 스텝으로 전환
            self.current_step_idx += 1
            
            if self.current_step_idx < len(self.steps):
                # 직전 스텝의 끝점을 시작점으로 고정하기 위해 현재 로봇 상태 반영
                current_full_positions = self.robot.get_joint_positions()
                self.start_angles = np.array([current_full_positions[i] for i in self.target_indices])
                
                self.target_angles = np.radians(self.steps[self.current_step_idx]["angles"])
                self.step_duration = self.steps[self.current_step_idx]["duration"]
                self.time_in_step = 0.0
                print(f"🔄 [M0609 + Finger] 스텝 {self.current_step_idx} 진입 (목표: {self.steps[self.current_step_idx]['angles']})")
            else:
                self.is_finished = True
                print("🎉 [M0609 + Finger] 모든 조인트 및 그리퍼 시퀀스 동작이 완료되었습니다!")
                return

        # 선형 보간(Linear Interpolation) 계산
        interpolated_angles = self.start_angles + (self.target_angles - self.start_angles) * progress
        
        # 전체 조인트 상태를 가져온 뒤, 제어할 조인트 값만 덮어씌움 (다른 조인트 간섭 방지)
        full_targets = self.robot.get_joint_position_targets()
        for idx, target_val in zip(self.target_indices, interpolated_angles):
            full_targets[idx] = target_val
            
        # Isaac Sim Articulation에 타겟 주입
        self.robot.set_joint_position_targets(full_targets)

# ==============================================================================
# 3. 렌더 루프 바인딩
# ==============================================================================
if "_m0609_gripper_sub" in globals() and _m2_bridge_sub is not None:
    try:
        _m0609_gripper_sub.unsubscribe()
    except:
        pass
    _m0609_gripper_sub = None

controller = M0609WithGripperController(ROBOT_PRIM_PATH, JOINT_NAMES, JOINT_STEPS)

app = omni.kit.app.get_app()
update_stream = app.get_update_event_stream()

def on_render_tick(e):
    dt = e.payload.get("dt", 0.016)
    controller.update(dt)

_m0609_gripper_sub = update_stream.create_subscription_to_pop(on_render_tick)
print("🚀 [준비 완료] Isaac Sim Play(▶) 버튼을 누르면 로봇과 핑거 조인트가 순서대로 제어됩니다.")