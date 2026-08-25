import os
import glob
import omni.replicator.core as rep

TARGET_PRIM_PATH = "/World/box" 
OUTPUT_DIR = "/home/rokey/rokey_cobot3/isaacpjt/assets/test_data"
TOTAL_FRAMES = 10  
IMAGE_RESOLUTION = (640, 640)

os.makedirs(OUTPUT_DIR, exist_ok=True)

with rep.new_layer():
    package = rep.get.prims(path_pattern=TARGET_PRIM_PATH)

    # 1. 표준 왜곡 없는 렌즈(35mm) 설정
    camera = rep.create.camera(focal_length=35.0)
    render_product = rep.create.render_product(camera, IMAGE_RESOLUTION)

    distant_light = rep.create.light(light_type="distant")
    spot_light = rep.create.light(light_type="sphere")

    with rep.trigger.on_frame(num_frames=TOTAL_FRAMES):
        # 2. 카메라: 라벨지가 위치한 '좌측 전면'을 안정적 원거리에서 조망
        # X: -2.5 ~ -0.8 (라벨지 보이는 좌측), Y: 3.5 ~ 5.0 (뒤로 충분히 물러남), Z: 0.8 ~ 2.2 (살짝 내려다보는 구도)
        with camera:
            rep.modify.pose(
                position=rep.distribution.uniform((10.0, 9.0, 0.8), (30, 11.0, 7.0)),
                look_at=TARGET_PRIM_PATH
            )

        # 3. 조명 전방위 랜덤화
        with distant_light:
            rep.modify.attribute("intensity", rep.distribution.uniform(1500.0, 3500.0))
            rep.modify.pose(rotation=rep.distribution.uniform((-60, -60, 0), (60, 60, 360)))

        with spot_light:
            rep.modify.pose(position=rep.distribution.uniform((-3.0, 2.0, 2.0), (1.0, 4.0, 4.0)))
            rep.modify.attribute("intensity", rep.distribution.uniform(6000.0, 20000.0))

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=OUTPUT_DIR, rgb=True, bounding_box_2d_tight=False)
    writer.attach([render_product])

def cleanup_after_rendering():
    for f in glob.glob(os.path.join(OUTPUT_DIR, "*.npy")) + glob.glob(os.path.join(OUTPUT_DIR, "*.json")):
        try: os.remove(f)
        except Exception: pass
    print(f"✅ [촬영 완료] 라벨지 노출 전체 샷 생성 완료! ({OUTPUT_DIR})")

rep.orchestrator.register_status_callback(
    lambda status: cleanup_after_rendering() if status == rep.Status.STOPPED else None
)

rep.orchestrator.run()