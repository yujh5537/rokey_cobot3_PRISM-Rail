# Isaac 세션 기동용 런처 — Play 전에 이것 하나만 File>Open > Run
# 순서 중요: 브릿지가 rclpy 컨텍스트 소유자라 반드시 첫 번째
import os
BASE = "/home/rokey/rokey_cobot3/isaacpjt/scripts"
SCRIPTS = [
    "isaac_twin_m2_bridge.py",                        # 1) 관제 좌표 수신
    "capsule_button_follow_and_door_controller.py",   # 2) 버튼·문 (석형)
    "or_label_capture.py",                            # 3) 라벨 캡처
    "c05_pack_owner.py",                              # 4) 화물 소유권 (석형)
]
for i, name in enumerate(SCRIPTS, 1):
    path = os.path.join(BASE, name)
    print(f"\n{'='*50}\n[{i}/{len(SCRIPTS)}] {name}\n{'='*50}")
    if not os.path.exists(path):
        print("  !! 파일 없음 — 건너뜀:", path); continue
    try:
        exec(compile(open(path).read(), path, "exec"), globals())
    except Exception as e:
        import traceback; traceback.print_exc()
        print(f"  !! {name} 실패 — 이후 스크립트는 계속 진행합니다")
print("\n>>> 이제 ▶ Play, 그다음 or_unload_orchestrator.py 를 Run 하세요")
