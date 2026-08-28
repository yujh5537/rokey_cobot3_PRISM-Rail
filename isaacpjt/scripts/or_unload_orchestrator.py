# ============================================================
# OR Unload Orchestrator (step 7 + step 8 motion integrated)
#   지휘: 버튼 누름 -> (석형님 컨트롤러가 문 개폐) -> 취출/제시/적재 -> 재누름
#   문은 여기서 절대 구동하지 않는다 (감시만)
#   ROS: /or_station_event (std_msgs/String, JSON)
#   사용: File > Open > Run  -> Play -> start("Capsule_01")
# ============================================================
import json, time, sys, os
import omni.usd, omni.kit.app
from pxr import UsdGeom, UsdPhysics, Sdf

# config 경로: 환경변수 > 이 파일 기준 상대 > 알려진 후보들 순으로 탐색
def _find_cfg():
    cands = []
    if os.environ.get("OR_STATION_CFG"):
        cands.append(os.environ["OR_STATION_CFG"])
    try:
        cands.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "..", "config", "or_station.json"))
    except NameError:
        pass
    cands += [
        "/home/rokey/cobot3_ws/medical-rail-twin/isaacpjt/config/or_station.json",
        "/home/rokey/rokey_cobot3/isaacpjt/config/or_station.json",
    ]
    for p in cands:
        if p and os.path.exists(p):
            return os.path.normpath(p)
    raise FileNotFoundError("or_station.json 을 찾지 못함 — OR_STATION_CFG 환경변수로 지정하세요")

_CFG_PATH = _find_cfg()
_SCRIPT_DIR = os.path.dirname(_CFG_PATH).replace("/config", "/scripts")
CFG = json.load(open(_CFG_PATH))
print("[cfg]", _CFG_PATH)
DOOR = CFG["door_mechanism"]
CAP2ID = {f"Capsule_{i:02d}": f"C{i:02d}" for i in range(1, 11)}
ID2CAP = {v: k for k, v in CAP2ID.items()}
stage = omni.usd.get_context().get_stage()

# ---------- 모션 모듈 ----------
for _d in (_SCRIPT_DIR,
           "/home/rokey/cobot3_ws/medical-rail-twin/isaacpjt/scripts",
           "/home/rokey/rokey_cobot3/isaacpjt/scripts"):
    if os.path.isdir(_d):
        sys.path.insert(0, _d)
import m0609_motion as _M
import or_pack_attach as _A

# ── 기동 가드: Play 시작 자세를 접근 포즈로 강제 (씬 저장값과 무관하게 유령 누름 차단)
_AP = CFG["teach_deg"].get("button_approach")
if _AP:
    for _p, _q in zip(CFG["arm_joints"], _AP[:6]):
        _prim = stage.GetPrimAtPath(_p)
        _d = UsdPhysics.DriveAPI.Get(_prim, "angular")
        if _d and _d.GetTargetPositionAttr():
            _d.GetTargetPositionAttr().Set(_q)
        # 시작 '상태'까지 접근 포즈로: Play 순간 버튼 누른 자세로 태어나는 것 방지
        _sa = _prim.GetAttribute("state:angular:physics:position")
        if not _sa:
            _sa = _prim.CreateAttribute("state:angular:physics:position",
                                        Sdf.ValueTypeNames.Float)
        _sa.Set(_q)
    print("[init] arm start targets+STATE -> button_approach (ghost-press guard v2)")
else:
    print("[init] WARNING: button_approach missing in config - run step B first!")

def robot_goto(pose, dur=2.5): _M.goto(pose, dur)
def robot_grip(close):         _M.grip(close)
def robot_busy():              return _M.busy()

# ---------- ROS (없으면 print만) ----------
# 자동 트리거: 관제/대시보드가 발행하는 /order_event 의 SERVICE_START(수술실2 딥 진입)를
# 받으면 해당 캡슐 하역 시퀀스를 자동으로 큐에 넣는다. 대시보드에서 "시작"만 누르면
# 캡슐이 OR2 WORK 지점에 도착하는 순간 이 매니퓰레이터 시퀀스가 이어서 돈다.
_ros = {"ok": False}
# once=True: 씬에 수술팩(/World/Cargo_OR2)이 1개뿐이라 시작당 첫 캡슐 하역만 자동 실행.
#           연속 시연은 auto(once=False), 또는 매번 auto() 재무장.
_AUTO = {"queue": [], "seen": set(), "enabled": True, "once": True, "fired": False}


def _on_order_event(msg):
    try:
        e = json.loads(msg.data)
    except Exception:
        return
    if e.get("event") != "SERVICE_START":
        return
    subj = e.get("subject") or e.get("capsule") or ""
    detail = e.get("detail", "")
    # OR2(B2-09) 딥만 대상 — 씬 매니퓰레이터/수술팩이 OR2에 배치돼 있음
    if "B2-09" not in detail and "OR2" not in detail:
        return
    cap = ID2CAP.get(subj)
    if not cap:
        return
    key = (subj, round(float(e.get("sim_t", 0)), 1))
    if key in _AUTO["seen"]:
        return
    _AUTO["seen"].add(key)
    if not _AUTO["enabled"]:
        return
    if _AUTO["once"] and _AUTO["fired"]:
        return
    if cap not in _AUTO["queue"]:
        _AUTO["queue"].append(cap)
        _AUTO["fired"] = True
        print(f"[auto] SERVICE_START {subj} -> 하역 큐 추가 ({cap}); 대기열 {_AUTO['queue']}")


try:
    import rclpy
    from std_msgs.msg import String
    if not rclpy.ok():
        rclpy.init()
    _node = rclpy.create_node("or_unload_orchestrator")
    _pub = _node.create_publisher(String, "/or_station_event", 10)
    _sub_evt = _node.create_subscription(String, "/order_event", _on_order_event, 50)
    _ros["ok"] = True
    print("[ros] pub /or_station_event  +  sub /order_event (auto-trigger)")
except Exception as e:
    print("[ros] unavailable -> events print only:", type(e).__name__)

import socket as _socket
_udp = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
def publish_event(payload):
    msg = json.dumps(payload)
    print("[EVENT]", msg)
    if _ros["ok"]:
        m = String(); m.data = msg; _pub.publish(m)
    try:
        _udp.sendto(msg.encode("utf-8"), ("127.0.0.1", 47136))   # relay 경유 (rclpy 불능 대비)
    except Exception as _e:
        print("[udp] send fail:", _e)

# ---------- 문 감시 (읽기 전용) ----------
def door_y(cap, side):
    p = stage.GetPrimAtPath(f"/World/Capsules/{cap}/Model/DoorSystem/Door_Neg_{side}")
    if not (p and p.IsValid()): return None
    for op in UsdGeom.Xformable(p).GetOrderedXformOps():
        if "translate" in op.GetOpName():
            return op.Get()[1]
    return None

def door_state(cap, tol=0.012):
    yl = door_y(cap, "L")
    if yl is None: return "UNKNOWN"
    if abs(yl - DOOR["left_open"])   < tol: return "OPEN"
    if abs(yl - DOOR["left_closed"]) < tol: return "CLOSED"
    return "MOVING"

# ---------- 버튼 ----------
def _btn_target(cap):
    prim = stage.GetPrimAtPath(f"/World/CapsuleButtonPhysics/{cap}/Button_Joint")
    drv = UsdPhysics.DriveAPI.Get(prim, "linear") or UsdPhysics.DriveAPI.Apply(prim, "linear")
    if not (drv.GetStiffnessAttr().Get() or 0) >= 500:
        (drv.GetStiffnessAttr() or drv.CreateStiffnessAttr(0)).Set(2000.0)
        (drv.GetDampingAttr()   or drv.CreateDampingAttr(0)).Set(50.0)
    return drv.GetTargetPositionAttr() or drv.CreateTargetPositionAttr(0.0)

# ---------- 상태기계 ----------
S = {"state": "IDLE", "cap": None, "t0": 0, "press_t": 0, "btn": None,
     "queue": [], "hold_until": None}

def start(cap="Capsule_01"):
    if S["state"] not in ("IDLE", "DONE"):
        print("[seq] busy:", S["state"]); return
    if door_state(cap) != "CLOSED":
        print(f"[seq] ABORT: door={door_state(cap)} (must be CLOSED). "
              f"recover: M.goto('button'), wait toggle, M.goto('button_approach')")
        return
    S.update(state="PRESS_OPEN", cap=cap, t0=time.time(), press_t=0,
             btn=None, queue=[], hold_until=None)
    print(f"[seq] start {cap} (door={door_state(cap)})")

def _tick(e):
    # 1) ROS 이벤트 수신 (자동 트리거용) — 논블로킹
    if _ros["ok"]:
        try:
            rclpy.spin_once(_node, timeout_sec=0.0)
        except Exception:
            pass

    st, cap = S["state"], S["cap"]

    # 2) 유휴 상태면 자동 큐에서 다음 캡슐 하역 시작 (문이 닫혀 있을 때만)
    if st in ("IDLE", "DONE") and _AUTO["queue"]:
        nxt = _AUTO["queue"][0]
        ds = door_state(nxt)
        if ds == "CLOSED":
            _AUTO["queue"].pop(0)
            print(f"[auto] 하역 시작 -> {nxt}")
            start(nxt)
        elif time.time() - _AUTO.get("_warn_t", 0) > 3.0:
            _AUTO["_warn_t"] = time.time()
            print(f"[auto] {nxt} 대기 중 — door={ds} (CLOSED 되어야 시작). "
                  f"prim: /World/Capsules/{nxt}/Model/DoorSystem/Door_Neg_L")
        return

    if st in ("IDLE", "DONE"): return
    now = time.time()

    if st == "PRESS_OPEN":
        robot_goto("button", 2.0); S["state"] = "GO_BTN1"       # contact = physical press
    elif st == "GO_BTN1" and not robot_busy():
        S["press_t"] = now; S["state"] = "REL_OPEN"
    elif st == "REL_OPEN" and now - S["press_t"] > 0.6:
        robot_goto("button_approach", 1.5); S["state"] = "RET_OPEN"   # retreat = release
    elif st == "RET_OPEN" and not robot_busy():
        S["state"] = "WAIT_OPEN"; S["t0"] = now
    elif st == "WAIT_OPEN" and door_state(cap) == "OPEN":
        publish_event({"capsule_id": CAP2ID[cap], "event": "DOOR_OPEN"})
        S["queue"] = [("goto","grasp",2.5), ("grip",True,0.8), ("attach",None,0),
                      ("goto","camera",3.0), ("hold",None,2.0),
                      ("goto","tray",3.0),   ("release",None,0), ("grip",False,0.8)]
        S["state"] = "MOVE"
    elif st == "MOVE":
        if robot_busy() or _A.busy(): pass
        elif S["hold_until"] and now < S["hold_until"]: pass
        elif S["queue"]:
            kind, arg, dur = S["queue"].pop(0); S["hold_until"] = None
            if kind == "goto": robot_goto(arg, dur)
            elif kind == "grip": robot_grip(arg)
            elif kind == "attach": _A.attach()
            elif kind == "release": _A.release()
            elif kind == "hold":
                S["hold_until"] = now + dur
                print(f"[seq] hold {dur}s (camera capture point - step 10)")
        else:
            S["state"] = "PRESS_CLOSE"
    elif st == "PRESS_CLOSE":
        robot_goto("button", 2.5); S["state"] = "GO_BTN2"
    elif st == "GO_BTN2" and not robot_busy():
        S["press_t"] = now; S["state"] = "REL_CLOSE"
    elif st == "REL_CLOSE" and now - S["press_t"] > 0.6:
        robot_goto("button_approach", 1.5); S["state"] = "RET_CLOSE"
    elif st == "RET_CLOSE" and not robot_busy():
        S["state"] = "WAIT_CLOSED"; S["t0"] = now
    elif st == "WAIT_CLOSED" and door_state(cap) == "CLOSED":
        publish_event({"capsule_id": CAP2ID[cap], "event": "DOOR_CLOSED"})
        S["state"] = "DONE"
        print("[seq] DONE - control core may now allow departure (BLUE)")

    if st.startswith("WAIT") and now - S["t0"] > 30:
        print(f"[seq] TIMEOUT at {st}, door_state={door_state(cap)}")
        S["state"] = "DONE"

def auto(on=True, once=True):
    """자동 트리거 재무장/설정.
      auto()            -> 다음 SERVICE_START(OR2) 1건 자동 실행 (기본)
      auto(once=False)  -> 도착하는 모든 OR2 캡슐 연속 자동 (수술팩 1개 주의)
      auto(False)       -> 자동 끄고 수동 start() 만
    """
    _AUTO["enabled"] = bool(on)
    _AUTO["once"] = bool(once)
    _AUTO["fired"] = False
    if not on:
        _AUTO["queue"].clear()
    print(f"[auto] enabled={_AUTO['enabled']} once={_AUTO['once']} — 재무장 완료")


_sub = (omni.kit.app.get_app().get_update_event_stream()
        .create_subscription_to_pop(_tick, name="or_unload_orchestrator"))
print("=" * 50)
print("OR UNLOAD ORCHESTRATOR ACTIVE")
print("  door:", DOOR["type"], "| ros:", _ros["ok"], "| motion: m0609_motion")
print("  자동: 대시보드 '시작'(모드 A) -> /order_event SERVICE_START(OR2) -> 하역 자동 실행")
print("  수동: start('Capsule_01')   |   자동 끄기: auto(False)")
print("=" * 50)
