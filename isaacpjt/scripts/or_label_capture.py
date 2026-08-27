# ============================================================
# [Step 10-2] Isaac 라벨 캡처 모듈
#   capture("C05") -> 2560x1440 PNG 저장 + UDP 47137로 OCR 노드에 통지
#   Isaac 내장 Python(3.11)에서 실행. rclpy 불필요 (UDP만 사용)
#   사용: File > Open > Run -> Play -> capture("C05")
#   검증용: preview()  = 라벨을 카메라 앞에 임시 배치 (Ctrl+S 금지!)
#           restore()  = 원위치 복구
# ============================================================
import json, os, socket, time
import omni.usd, omni.kit.app
from pxr import Usd, UsdGeom, Gf

CFG = json.load(open("/home/rokey/rokey_cobot3/isaacpjt/config/label_ocr.json"))
CAM = CFG["camera_prim"]
RES = tuple(CFG.get("capture_res", [2560, 1440]))
OUT = CFG.get("capture_dir", "/home/rokey/rokey_cobot3/isaacpjt/output/captures")
PORT = CFG.get("udp_port", 47137)
PACK_ROOT = "/World/Cargo_OR2"
LABEL = CFG.get("label_prim", "/World/Cargo_OR2/SurgicalPack/Label")

os.makedirs(OUT, exist_ok=True)
stage = omni.usd.get_context().get_stage()
_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_S = {"rp": None, "saved": None}

def _wpos(path):
    return UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        stage.GetPrimAtPath(path)).ExtractTranslation()

def dist_label_cam():
    """카메라 렌즈 <-> 라벨 거리(m). 700px 조건 판정에 씀"""
    try:
        d = (_wpos(LABEL) - _wpos(CAM)).GetLength()
        print(f"[cap] label-camera distance = {d:.3f} m")
        return d
    except Exception as e:
        print("[cap] distance failed:", e); return None

def _render_product():
    if _S["rp"] is None:
        import omni.replicator.core as rep
        _S["rp"] = rep.create.render_product(CAM, RES)
        _S["writer"] = rep.WriterRegistry.get("BasicWriter")
        _S["writer"].initialize(output_dir=OUT, rgb=True)
        _S["writer"].attach([_S["rp"]])
        print(f"[cap] render product ready: {RES[0]}x{RES[1]} on {CAM}")
    return _S["rp"]

def capture(capsule_id="C05", notify=True, warmup=None):
    """카메라로 1장 찍어 저장하고 OCR 노드에 통지. 저장 경로 반환"""
    import omni.replicator.core as rep
    _render_product()
    n = warmup if warmup is not None else CFG.get("warmup_frames", 3)
    for _ in range(n + 1):                      # 워밍업: 첫 프레임은 미완성일 수 있음
        rep.orchestrator.step(rt_subframes=8)
    # BasicWriter가 rgb_XXXX.png 로 저장 -> 최신 파일을 찾아 이름 정리
    time.sleep(0.4)
    files = sorted([f for f in os.listdir(OUT) if f.startswith("rgb_") and f.endswith(".png")],
                   key=lambda f: os.path.getmtime(os.path.join(OUT, f)))
    if not files:
        print("[cap] ERROR: no image written to", OUT); return None
    src = os.path.join(OUT, files[-1])
    dst = os.path.join(OUT, f"{capsule_id}_{time.strftime('%H%M%S')}.png")
    os.rename(src, dst)
    print(f"[cap] saved: {dst}")
    if notify:
        msg = json.dumps({"capsule_id": capsule_id, "image": dst})
        _udp.sendto(msg.encode(), ("127.0.0.1", PORT))
        print(f"[cap] notified OCR node (UDP {PORT})")
    return dst

# ---------- 검증 헬퍼: 팩을 카메라 앞에 임시 배치 (9단계 미완 상태에서 단독 시험용) ----------
def preview(dist=0.25, yaw=0.0):
    """팩을 카메라 정면 dist(m)에 배치. pitch=0 유지가 핵심(내성 시험 결론)"""
    prim = stage.GetPrimAtPath(PACK_ROOT)
    xf = UsdGeom.Xformable(prim)
    if _S["saved"] is None:
        _S["saved"] = xf.GetLocalTransformation()
        print("[cap] original transform saved (restore() to undo)")
    cam = _wpos(CAM)
    # 카메라는 -Y를 바라본다(5단계 확정) -> 팩을 -Y 방향 dist 앞, 같은 높이에 둔다
    pos = Gf.Vec3d(cam[0], cam[1] - dist, cam[2])
    m = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw))
    m.SetTranslateOnly(pos)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    print(f"[cap] pack placed at {[round(v,3) for v in pos]} (dist={dist}, yaw={yaw})")
    print("      *** DO NOT SAVE THE SCENE (Ctrl+S) *** call restore() when done")

def restore():
    if _S["saved"] is None:
        print("[cap] nothing to restore"); return
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(PACK_ROOT))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(_S["saved"])
    _S["saved"] = None
    print("[cap] pack transform restored")

print("=" * 50)
print("LABEL CAPTURE MODULE ACTIVE")
print(f"  camera: {CAM}")
print(f"  res: {RES[0]}x{RES[1]} -> {OUT}")
print("  -> Play, then: capture('C05')")
print("  -> standalone test: preview(0.25) / capture('C05') / restore()")
print("=" * 50)


# ---------- 카메라 시선 방향을 씬에서 직접 읽기 (추측 금지) ----------
def cam_forward():
    """카메라 로컬 -Z가 시선. 월드 변환에서 실제 방향 벡터를 뽑는다."""
    m = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        stage.GetPrimAtPath(CAM))
    fwd = m.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    up  = m.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized()
    print(f"[cam] forward = {[round(v,3) for v in fwd]}")
    print(f"[cam] up      = {[round(v,3) for v in up]}")
    return fwd

def preview3(dist=0.20):
    """카메라 시선 방향으로 dist 앞에 팩 배치 (방향을 씬에서 읽으므로 부호 실수 없음)"""
    prim = stage.GetPrimAtPath(PACK_ROOT)
    xf = UsdGeom.Xformable(prim)
    if _S["saved"] is None:
        _S["saved"] = xf.GetLocalTransformation()
        print("[cap] original transform saved")
    else:
        print("[cap] WARNING: 이전 저장본 유지 — restore() 를 먼저 호출하세요")
    fwd = cam_forward()
    cam = _wpos(CAM)
    pos = Gf.Vec3d(cam[0] + fwd[0]*dist, cam[1] + fwd[1]*dist, cam[2] + fwd[2]*dist)
    m = Gf.Matrix4d(); m.SetTranslateOnly(pos)
    xf.ClearXformOpOrder(); xf.AddTransformOp().Set(m)
    print(f"[cap] pack -> {[round(v,3) for v in pos]} (카메라 시선 {dist}m 앞)")
    print("      *** Ctrl+S 금지 *** 끝나면 restore()")


# ---------- GUI 호환 캡처 (orchestrator.step 은 standalone 전용이라 사용 불가) ----------
import numpy as _np
_A2 = {"rp": None, "annot": None, "res": None}

def _ensure_rp(res):
    import omni.replicator.core as rep
    if _A2["rp"] is None or _A2["res"] != res:
        _A2["rp"] = rep.create.render_product(CAM, res)
        _A2["annot"] = rep.AnnotatorRegistry.get_annotator("rgb")
        _A2["annot"].attach([_A2["rp"]])
        _A2["res"] = res
        print(f"[cap] render product {res[0]}x{res[1]} attached")

def capture2(tag="C05", res=None, warmup=12, notify=True):
    """카메라 1장 촬영 -> PNG 저장 -> OCR 노드에 UDP 통지 (비동기)"""
    import asyncio, omni.kit.app
    from PIL import Image
    res = tuple(res or RES)
    _ensure_rp(res)

    async def _go():
        app = omni.kit.app.get_app()
        for _ in range(warmup):
            await app.next_update_async()
        data = _A2["annot"].get_data()
        if data is None or len(data) == 0:
            print("[cap] ERROR: empty frame"); return None
        arr = _np.array(data)
        if arr.ndim == 3 and arr.shape[2] == 4:
            arr = arr[:, :, :3]
        path = os.path.join(OUT, f"{tag}_{time.strftime('%H%M%S')}.png")
        Image.fromarray(arr.astype(_np.uint8)).save(path)
        print(f"[cap] saved: {path} shape={arr.shape}")
        if notify:
            _udp.sendto(json.dumps({"capsule_id": tag, "image": path}).encode(),
                        ("127.0.0.1", PORT))
            print(f"[cap] notified OCR node (UDP {PORT})")
        return path

    return asyncio.ensure_future(_go())
