# ============================================================
# Pack attach module: kinematic follow grab / settle-to-tray release
#   attach(): remember pack pose relative to gripper, follow every tick
#   release(): stop following, lerp pack down onto tray top
# ============================================================
import omni.usd, omni.kit.app
from pxr import Usd, UsdGeom, Gf

stage = omni.usd.get_context().get_stage()
EE   = "/World/m0609/onrobot_rg2ft"          # gripper base (follow anchor)
PACK = "/World/Cargo_OR2"                     # wrapper xform (no physics on it)
TRAY_TOP_Z = 9.3 + 0.3                        # tray center z + half height = 9.6
PACK_HALF_Z = 0.125

_S = {"mode": "idle", "rel": None, "op": None, "lerp": None}

def _wxf(path):
    return UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        stage.GetPrimAtPath(path))

def _ensure_op():
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(PACK))
    if _S["op"] is None:
        m = xf.GetLocalTransformation()       # keep current pose
        xf.ClearXformOpOrder()
        _S["op"] = xf.AddTransformOp()
        _S["op"].Set(m)
    return _S["op"]

def attach():
    _ensure_op()
    _S["rel"] = _wxf(PACK) * _wxf(EE).GetInverse()   # pack = rel * ee
    _S["mode"] = "follow"
    print("[attach] pack follows gripper")

def release():
    if _S["mode"] != "follow":
        print("[attach] release ignored (not following)"); return
    m = _wxf(PACK)
    t = m.ExtractTranslation()
    goal = Gf.Vec3d(t[0], t[1], TRAY_TOP_Z + PACK_HALF_Z + 0.001)
    _S["lerp"] = {"m0": Gf.Matrix4d(m), "z0": t[2], "z1": goal[2], "t": 0.0, "dur": 0.5}
    _S["mode"] = "settle"
    print(f"[attach] release: settle z {t[2]:.3f} -> {goal[2]:.3f}")

def busy():
    return _S["mode"] == "settle"

def _tick(e):
    if _S["mode"] == "follow":
        _ensure_op().Set(_S["rel"] * _wxf(EE))
    elif _S["mode"] == "settle":
        L = _S["lerp"]
        dt = e.payload["dt"] if hasattr(e, "payload") and "dt" in e.payload else 1/60
        L["t"] += dt
        r = min(L["t"] / L["dur"], 1.0)
        s = r * r * (3 - 2 * r)
        m = Gf.Matrix4d(L["m0"])
        t = m.ExtractTranslation()
        m.SetTranslateOnly(Gf.Vec3d(t[0], t[1], L["z0"] + (L["z1"] - L["z0"]) * s))
        _ensure_op().Set(m)
        if r >= 1.0:
            _S["mode"] = "idle"
            print("[attach] settled on tray")

_sub = (omni.kit.app.get_app().get_update_event_stream()
        .create_subscription_to_pop(_tick, name="or_pack_attach"))
print("[attach] module active (EE=", EE, ")")
