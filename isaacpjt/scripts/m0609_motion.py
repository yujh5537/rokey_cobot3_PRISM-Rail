# ============================================================
# M0609 motion module: teach-pose playback via drive-target interpolation
#   - same mechanism the scene already uses (DriveAPI angular targets)
#   - cosine ramp: no jerk with 12kg payload
#   API: goto(name_or_list, duration_s) / grip(close, duration_s) / busy()
# ============================================================
import json, math, os
import omni.usd, omni.kit.app
from pxr import UsdPhysics

def _find_cfg():
    cands = [os.environ.get("OR_STATION_CFG")]
    try:
        cands.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "..", "config", "or_station.json"))
    except NameError:
        pass
    cands += ["/home/rokey/cobot3_ws/medical-rail-twin/isaacpjt/config/or_station.json",
              "/home/rokey/rokey_cobot3/isaacpjt/config/or_station.json"]
    for p in cands:
        if p and os.path.exists(p):
            return os.path.normpath(p)
    raise FileNotFoundError("or_station.json not found — set OR_STATION_CFG")

CFG = json.load(open(_find_cfg()))
stage = omni.usd.get_context().get_stage()

def _drive(path, kind="angular", min_stiff=None, min_damp=None):
    prim = stage.GetPrimAtPath(path)
    assert prim and prim.IsValid(), f"missing joint {path}"
    drv = UsdPhysics.DriveAPI.Get(prim, kind) or UsdPhysics.DriveAPI.Apply(prim, kind)
    st = drv.GetStiffnessAttr().Get() or 0.0
    dp = drv.GetDampingAttr().Get() or 0.0
    # 유령 값(0.63 같은) 방어: 최소치 미달이면 끌어올린다
    if min_stiff and st < min_stiff:
        (drv.GetStiffnessAttr() or drv.CreateStiffnessAttr(0)).Set(min_stiff)
        print(f"[drive-fix] {path.split('/')[-1]} stiffness {st:.3g} -> {min_stiff}")
    if min_damp and dp < min_damp:
        (drv.GetDampingAttr() or drv.CreateDampingAttr(0)).Set(min_damp)
        print(f"[drive-fix] {path.split('/')[-1]} damping {dp:.3g} -> {min_damp}")
    return drv.GetTargetPositionAttr() or drv.CreateTargetPositionAttr(0.0)

_ARM = [_drive(p) for p in CFG["arm_joints"]]          # 팔: 임포트 값 존중(관찰만)
_FING = _drive(CFG["finger_joint"], min_stiff=300.0, min_damp=30.0)  # 0.63 유령값 교정
GRIP_CLOSE_DEG = 19.9      # teach ②
GRIP_OPEN_DEG = 0.0

_state = {"active": False, "t": 0.0, "dur": 1.0, "start": None, "goal": None,
          "fstart": None, "fgoal": None}

def _cur_arm():
    return [a.Get() or 0.0 for a in _ARM]

def busy():
    return _state["active"]

def goto(pose, duration_s=2.5):
    """pose: teach name ('button','grasp','camera','tray','button2') or 6-list"""
    goal = CFG["teach_deg"][pose][:6] if isinstance(pose, str) else list(pose)[:6]
    _state.update(active=True, t=0.0, dur=max(duration_s, 0.2),
                  start=_cur_arm(), goal=goal, fstart=None, fgoal=None)
    print(f"[motion] goto {pose} in {duration_s}s")

def grip(close, duration_s=0.8):
    g = GRIP_CLOSE_DEG if close else GRIP_OPEN_DEG
    _state.update(active=True, t=0.0, dur=max(duration_s, 0.1),
                  start=None, goal=None,
                  fstart=_FING.Get() or 0.0, fgoal=g)
    print(f"[motion] grip {'CLOSE' if close else 'OPEN'} -> {g} deg")

def _tick(e):
    if not _state["active"]:
        return
    dt = e.payload["dt"] if hasattr(e, "payload") and "dt" in e.payload else 1/60
    _state["t"] += dt
    r = min(_state["t"] / _state["dur"], 1.0)
    s = 0.5 - 0.5 * math.cos(math.pi * r)          # cosine ease in/out
    if _state["goal"] is not None:
        for a, q0, q1 in zip(_ARM, _state["start"], _state["goal"]):
            a.Set(q0 + (q1 - q0) * s)
    if _state["fgoal"] is not None:
        _FING.Set(_state["fstart"] + (_state["fgoal"] - _state["fstart"]) * s)
    if r >= 1.0:
        _state["active"] = False
        print("[motion] done")

_sub = (omni.kit.app.get_app().get_update_event_stream()
        .create_subscription_to_pop(_tick, name="m0609_motion"))
print("[motion] module active - joints:", len(_ARM), "+finger")
