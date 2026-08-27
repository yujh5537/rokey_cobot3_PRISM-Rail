import builtins

import omni.usd
import omni.kit.app

from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf


# ============================================================
# M0609 + VG10 + SURGICAL PACK
#
# PLAY ON
#
# Normal speed
# PhysicsFixedJoint grip
# ============================================================


stage = omni.usd.get_context().get_stage()

# ============================================================
# CONTROL / EVENT OUTPUT
#
# B -> Control:
#   DOOR_OPEN
#   PACK_GRIPPED
#   PACK_UNLOADED
#   DOOR_CLOSED
#
# ROS topic:
#   /or_station_event   (std_msgs/String, JSON)
# ============================================================

import json
import socket as _socket

CAPSULE_NAME = "Capsule_05"
CAPSULE_ID = "C05"

_ros = {"ok": False}

try:
    import rclpy
    from std_msgs.msg import String

    if not rclpy.ok():
        rclpy.init()

    _event_node = rclpy.create_node(
        "or_unload_orchestrator_b"
    )

    _event_pub = _event_node.create_publisher(
        String,
        "/or_station_event",
        10
    )

    _ros["ok"] = True
    print("[ROS] /or_station_event publisher ready")

except Exception as e:
    print(
        "[ROS] unavailable -> print/UDP only:",
        type(e).__name__
    )


_udp = _socket.socket(
    _socket.AF_INET,
    _socket.SOCK_DGRAM
)


def publish_event(event_name):

    payload = {
        "capsule_id": CAPSULE_ID,
        "event": event_name,
    }

    msg = json.dumps(payload)

    print("[EVENT]", msg)

    if _ros["ok"]:
        ros_msg = String()
        ros_msg.data = msg
        _event_pub.publish(ros_msg)

    try:
        _udp.sendto(
            msg.encode("utf-8"),
            ("127.0.0.1", 47136)
        )
    except Exception:
        pass


# ============================================================
# DOOR STATE
# ============================================================

DOOR_LEFT_CLOSED = 0.0605
DOOR_LEFT_OPEN = 0.1755
DOOR_TOL = 0.012

DOOR_LEFT_PATH = (
    "/World/Capsules/Capsule_05/"
    "Model/DoorSystem/Door_Neg_L"
)


def door_state():

    prim = stage.GetPrimAtPath(
        DOOR_LEFT_PATH
    )

    if not prim.IsValid():
        return "UNKNOWN"

    for op in (
        UsdGeom.Xformable(prim)
        .GetOrderedXformOps()
    ):
        if "translate" not in op.GetOpName():
            continue

        value = op.Get()

        if value is None:
            continue

        # Existing door controller uses the Y component.
        y = float(value[1])

        if abs(y - DOOR_LEFT_OPEN) < DOOR_TOL:
            return "OPEN"

        if abs(y - DOOR_LEFT_CLOSED) < DOOR_TOL:
            return "CLOSED"

        return "MOVING"

    return "UNKNOWN"


ROBOT_ROOT = "/World/m0609"
LINK6_PATH = "/World/m0609/link_6"

SUCTION_POINT_PATH = (
    "/World/m0609/link_6/"
    "onrobot_vg10/suction_point"
)

CARGO_WORLD_ROOT = "/World/Cargo_OR2/SurgicalPack"
CARGO_LOADED_ROOT = "/World/Capsules/Capsule_05/SurgicalPack"

if stage.GetPrimAtPath(CARGO_LOADED_ROOT).IsValid():
    CARGO_ROOT = CARGO_LOADED_ROOT
elif stage.GetPrimAtPath(CARGO_WORLD_ROOT).IsValid():
    CARGO_ROOT = CARGO_WORLD_ROOT
else:
    raise RuntimeError(
        "SURGICAL PACK NOT FOUND at loaded/world path"
    )

GRIP_JOINT_PATH = (
    "/World/m0609/link_6/"
    "onrobot_vg10/VG10_GripJoint"
)


# ============================================================
# TIMING
# ============================================================

SINGLE_JOINT_TIME = 0.55
MULTI_JOINT_TIME = 0.90
FULL_POSE_TIME = 1.20

BUTTON_HOLD_TIME = 0.45
GRIP_HOLD_TIME = 0.40
VISION_HOLD_TIME = 1.0
RELEASE_HOLD_TIME = 0.30


# ============================================================
# POSES
#
# [J1, J2, J3, J4, J5, J6]
# ============================================================

BUTTON_READY = [
    90.0,
    35.0,
    108.0,
    -6.5,
    70.0,
    4.0,
]


# J4 changed to 21
VISION_POSE = [
    96.8,
    3.5,
    -86.0,
    0.0,
    -93.0,
    96.8,
]


# ============================================================
# VALIDATE
# ============================================================

for path in [
    ROBOT_ROOT,
    LINK6_PATH,
    SUCTION_POINT_PATH,
    CARGO_ROOT,
]:

    prim = stage.GetPrimAtPath(path)

    if not prim.IsValid():
        raise RuntimeError(
            "NOT FOUND: " + path
        )


# ============================================================
# FIND CARGO RIGID BODY
# ============================================================

def find_cargo_rigid_body():

    root = stage.GetPrimAtPath(
        CARGO_ROOT
    )

    if root.HasAPI(
        UsdPhysics.RigidBodyAPI
    ):
        return root

    for prim in Usd.PrimRange(root):

        if prim.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            return prim

    return None


cargo_body = find_cargo_rigid_body()


if cargo_body is None:
    raise RuntimeError(
        "NO RIGID BODY FOUND UNDER: "
        + CARGO_ROOT
    )


CARGO_BODY_PATH = str(
    cargo_body.GetPath()
)


# ============================================================
# FIND M0609 JOINTS
# ============================================================

joint_prims = []


for prim in stage.Traverse():

    path = str(
        prim.GetPath()
    )

    if not path.startswith(
        ROBOT_ROOT + "/"
    ):
        continue

    if prim.IsA(
        UsdPhysics.RevoluteJoint
    ):
        joint_prims.append(
            prim
        )


def joint_sort_key(prim):

    name = prim.GetName()

    digits = "".join(
        c for c in name
        if c.isdigit()
    )

    if digits:
        return int(digits)

    return 999


joint_prims.sort(
    key=joint_sort_key
)


if len(joint_prims) < 6:
    raise RuntimeError(
        "M0609 JOINT COUNT < 6"
    )


JOINTS = joint_prims[:6]


print("")
print("========================================")
print("M0609 JOINT MAP")
print("========================================")

for i, prim in enumerate(JOINTS):

    print(
        "J%d =" % (i + 1),
        prim.GetPath()
    )


# ============================================================
# DRIVE TARGETS
# ============================================================

def get_drive_target_attr(joint_prim):

    attr = joint_prim.GetAttribute(
        "drive:angular:physics:targetPosition"
    )

    if attr.IsValid():
        return attr

    drive = UsdPhysics.DriveAPI.Apply(
        joint_prim,
        "angular"
    )

    return drive.CreateTargetPositionAttr()


DRIVE_TARGETS = [
    get_drive_target_attr(prim)
    for prim in JOINTS
]


def read_joint_targets():

    result = []

    for attr in DRIVE_TARGETS:

        value = attr.Get()

        if value is None:
            value = 0.0

        result.append(
            float(value)
        )

    return result


def write_joint_targets(values):

    for i in range(6):

        DRIVE_TARGETS[i].Set(
            float(values[i])
        )


# ============================================================
# WORLD TRANSFORM
# ============================================================

def world_matrix(path):

    prim = stage.GetPrimAtPath(path)

    if not prim.IsValid():
        raise RuntimeError(
            "INVALID PRIM: " + path
        )

    return (
        UsdGeom.Xformable(prim)
        .ComputeLocalToWorldTransform(0)
    )


def matrix_quatf(matrix):

    q = matrix.ExtractRotationQuat()
    imag = q.GetImaginary()

    return Gf.Quatf(
        float(q.GetReal()),
        Gf.Vec3f(
            float(imag[0]),
            float(imag[1]),
            float(imag[2])
        )
    )


# ============================================================
# VACUUM ON
# ============================================================

def grip_on():

    if stage.GetPrimAtPath(
        GRIP_JOINT_PATH
    ).IsValid():

        stage.RemovePrim(
            GRIP_JOINT_PATH
        )


    suction_world = world_matrix(
        SUCTION_POINT_PATH
    )

    body0_world = world_matrix(
        LINK6_PATH
    )

    body1_world = world_matrix(
        CARGO_BODY_PATH
    )


    local0 = (
        suction_world
        * body0_world.GetInverse()
    )

    local1 = (
        suction_world
        * body1_world.GetInverse()
    )


    local_pos0 = local0.ExtractTranslation()
    local_pos1 = local1.ExtractTranslation()

    local_rot0 = matrix_quatf(
        local0
    )

    local_rot1 = matrix_quatf(
        local1
    )


    joint = UsdPhysics.FixedJoint.Define(
        stage,
        GRIP_JOINT_PATH
    )


    joint.CreateBody0Rel().SetTargets(
        [
            Sdf.Path(
                LINK6_PATH
            )
        ]
    )


    joint.CreateBody1Rel().SetTargets(
        [
            Sdf.Path(
                CARGO_BODY_PATH
            )
        ]
    )


    joint.CreateLocalPos0Attr().Set(
        Gf.Vec3f(
            float(local_pos0[0]),
            float(local_pos0[1]),
            float(local_pos0[2])
        )
    )


    joint.CreateLocalRot0Attr().Set(
        local_rot0
    )


    joint.CreateLocalPos1Attr().Set(
        Gf.Vec3f(
            float(local_pos1[0]),
            float(local_pos1[1]),
            float(local_pos1[2])
        )
    )


    joint.CreateLocalRot1Attr().Set(
        local_rot1
    )


    print("")
    print("========================================")
    print("VACUUM ON")
    print("========================================")

    # B reports successful robot ownership of the pack.
    publish_event("PACK_GRIPPED")


# ============================================================
# VACUUM OFF
# ============================================================

def grip_off():

    prim = stage.GetPrimAtPath(
        GRIP_JOINT_PATH
    )

    if prim.IsValid():

        stage.RemovePrim(
            GRIP_JOINT_PATH
        )


    print("")
    print("========================================")
    print("VACUUM OFF")
    print("========================================")

    # Pack is now released on the tray.
    publish_event("PACK_UNLOADED")


# ============================================================
# STEP BUILDERS
# ============================================================

steps = []


def add_single(
    joint_index,
    target,
    label,
    duration=SINGLE_JOINT_TIME
):

    steps.append({
        "type": "motion",
        "updates": {
            joint_index: float(target)
        },
        "duration": duration,
        "label": label,
    })


def add_multi(
    updates,
    label,
    duration=MULTI_JOINT_TIME
):

    steps.append({
        "type": "motion",
        "updates": {
            int(k): float(v)
            for k, v in updates.items()
        },
        "duration": duration,
        "label": label,
    })


def add_pose(
    pose,
    label,
    duration=FULL_POSE_TIME
):

    steps.append({
        "type": "pose",
        "pose": list(pose),
        "duration": duration,
        "label": label,
    })


def add_wait(
    duration,
    label
):

    steps.append({
        "type": "wait",
        "duration": float(duration),
        "label": label,
    })


def add_action(
    function,
    label
):

    steps.append({
        "type": "action",
        "function": function,
        "label": label,
    })


def add_wait_door(
    expected,
    label,
    timeout=30.0
):

    steps.append({
        "type": "door_wait",
        "expected": expected,
        "label": label,
        "timeout": float(timeout),
    })


# ============================================================
# SEQUENCE
# ============================================================


# ------------------------------------------------------------
# START BUTTON
# ------------------------------------------------------------

add_pose(
    BUTTON_READY,
    "01 BUTTON READY"
)


add_single(
    3,
    -27.0,
    "02 BUTTON PRESS"
)


add_wait(
    BUTTON_HOLD_TIME,
    "BUTTON HOLD"
)


add_single(
    3,
    -6.5,
    "03 BUTTON RELEASE"
)


add_wait_door(
    "OPEN",
    "WAIT DOOR OPEN"
)


# ------------------------------------------------------------
# GRIP ENTRY
# ------------------------------------------------------------

add_single(
    1,
    0.0,
    "04 J2 -> 0"
)


add_single(
    2,
    70.0,
    "05 J3 -> 70"
)


add_single(
    0,
    45.0,
    "06 J1 -> 45"
)


add_single(
    2,
    88.0,
    "07 J3 -> 88"
)


add_single(
    5,
    45.0,
    "08 J6 -> 45"
)


add_pose(
    [
        45.4,
        17.7,
        79.5,
        7.4,
        80.0,
        45.0,
    ],
    "09 FINAL GRASP"
)


add_action(
    grip_on,
    "VACUUM ON"
)


add_wait(
    GRIP_HOLD_TIME,
    "GRIP HOLD"
)


# ------------------------------------------------------------
# RETREAT
# ------------------------------------------------------------

add_single(
    1,
    0.0,
    "10 J2 -> 0"
)


add_multi(
    {
        2: 81.0,
        4: 73.0,
    },
    "11 J3=81 + J5=73"
)


add_multi(
    {
        2: 40.0,
        4: 107.0,
    },
    "12 J3=40 + J5=107"
)


# ------------------------------------------------------------
# APPROACH
# ------------------------------------------------------------

add_multi(
    {
        0: 97.0,
        2: -81.0,
        4: -83.0,
    },
    "13 J1=97 + J3=-81 + J5=-83"
)


add_single(
    1,
    -10.0,
    "14 J2 -> -10"
)


add_single(
    5,
    97.0,
    "15 J6 -> 97"
)


# ============================================================
# VISION
#
# J4 = 21
#
# Final:
# [97, -17, -85, 21, -83, 97]
# ============================================================

add_single(
    3,
    21.0,
    "16 J4 -> 21"
)


add_pose(
    VISION_POSE,
    "17 VISION"
)


add_wait(
    VISION_HOLD_TIME,
    "VISION HOLD 1 SEC"
)


# ============================================================
# NEW TRAY PATH
# ============================================================


# ------------------------------------------------------------
# Sequential:
# J3 -> -43
# ------------------------------------------------------------

add_single(
    2,
    -43.0,
    "18 TRAY ENTRY J3 -> -43"
)


# ------------------------------------------------------------
# Sequential:
# J2 -> 31
# ------------------------------------------------------------

add_single(
    1,
    31.0,
    "19 TRAY ENTRY J2 -> 31"
)


# ------------------------------------------------------------
# LINEAR MOVE
#
# J1 -> 50
# J2 -> 7.4
# J3 -> -90
#
# J4/J5/J6 unchanged
# ------------------------------------------------------------

add_multi(
    {
        0: 50.0,
        1: 7.4,
        2: -90.0,
    },
    "20 LINEAR J1=50 J2=7.4 J3=-90"
)


# ------------------------------------------------------------
# LINEAR MOVE
#
# J3 -> -63
# J2 -> -20
# J4 -> 0
# J6 -> 60
#
# J1/J5 unchanged
# ------------------------------------------------------------

add_multi(
    {
        2: -63.0,
        1: -20.0,
        3: 0.0,
        5: 60.0,
    },
    "21 LINEAR J3=-63 J2=-20 J4=0 J6=60"
)


# ------------------------------------------------------------
# VACUUM OFF
# ------------------------------------------------------------

add_action(
    grip_off,
    "22 VACUUM OFF"
)


add_wait(
    RELEASE_HOLD_TIME,
    "RELEASE HOLD"
)


# ============================================================
# RETURN
#
# Sequential:
# J3 -> -10
# J1 -> 90
# ============================================================

add_single(
    2,
    -10.0,
    "23 RETURN J3 -> -10"
)


add_single(
    0,
    90.0,
    "24 RETURN J1 -> 90"
)


# ============================================================
# BUTTON READY
# ============================================================

add_pose(
    BUTTON_READY,
    "25 BUTTON READY"
)


# ============================================================
# FINAL BUTTON
# ============================================================

add_single(
    3,
    -27.0,
    "26 FINAL BUTTON PRESS"
)


add_wait(
    BUTTON_HOLD_TIME,
    "FINAL BUTTON HOLD"
)


add_single(
    3,
    -6.5,
    "27 FINAL BUTTON RELEASE"
)


add_wait_door(
    "CLOSED",
    "WAIT DOOR CLOSED"
)


add_action(
    lambda: publish_event("DOOR_CLOSED"),
    "PUBLISH DOOR_CLOSED"
)


# ============================================================
# RUNTIME
# ============================================================

vg10_cycle = {
    "index": 0,
    "active": False,
    "elapsed": 0.0,
    "start": None,
    "target": None,
    "door_start_time": 0.0,
}


# ============================================================
# SMOOTH INTERPOLATION
# ============================================================

def smoothstep(t):

    t = max(
        0.0,
        min(
            1.0,
            t
        )
    )

    return (
        t * t * (3.0 - 2.0 * t)
    )


def begin_motion(step):

    current = read_joint_targets()

    target = list(
        current
    )


    if step["type"] == "pose":

        target = list(
            step["pose"]
        )

    else:

        for joint_index, value in (
            step["updates"].items()
        ):

            target[
                joint_index
            ] = value


    vg10_cycle["start"] = current
    vg10_cycle["target"] = target
    vg10_cycle["elapsed"] = 0.0
    vg10_cycle["active"] = True


    print("")
    print("----------------------------------------")
    print(
        "STEP",
        vg10_cycle["index"] + 1,
        "/",
        len(steps)
    )
    print(
        step["label"]
    )
    print(
        "TARGET =",
        target
    )
    print("----------------------------------------")


# ============================================================
# UPDATE
# ============================================================

def on_vg10_cycle_update(event):

    if vg10_cycle["index"] >= len(steps):
        return


    dt = event.payload.get(
        "dt",
        1.0 / 60.0
    )


    step = steps[
        vg10_cycle["index"]
    ]


    # --------------------------------------------------------
    # ACTION
    # --------------------------------------------------------

    if step["type"] == "action":

        if not vg10_cycle["active"]:

            print("")
            print(
                step["label"]
            )

            step["function"]()

            vg10_cycle["active"] = True


        vg10_cycle["index"] += 1
        vg10_cycle["active"] = False

        return


    # --------------------------------------------------------
    # DOOR WAIT
    # --------------------------------------------------------

    if step["type"] == "door_wait":

        if not vg10_cycle["active"]:

            vg10_cycle["active"] = True
            vg10_cycle["door_start_time"] = __import__("time").time()

            print("")
            print(
                "DOOR WAIT:",
                step["label"]
            )

        current_door = door_state()

        if current_door == step["expected"]:

            print(
                "DOOR STATE =",
                current_door
            )

            if step["expected"] == "OPEN":
                publish_event("DOOR_OPEN")

            vg10_cycle["index"] += 1
            vg10_cycle["active"] = False
            return

        if (
            __import__("time").time()
            - vg10_cycle["door_start_time"]
            >= step["timeout"]
        ):

            print(
                "DOOR WAIT TIMEOUT:",
                step["label"],
                "CURRENT =",
                current_door
            )

            return

        return


    # --------------------------------------------------------
    # WAIT
    # --------------------------------------------------------

    if step["type"] == "wait":

        if not vg10_cycle["active"]:

            vg10_cycle["elapsed"] = 0.0
            vg10_cycle["active"] = True

            print("")
            print(
                "WAIT:",
                step["label"]
            )


        vg10_cycle["elapsed"] += dt


        if (
            vg10_cycle["elapsed"]
            >= step["duration"]
        ):

            vg10_cycle["index"] += 1
            vg10_cycle["active"] = False


        return


    # --------------------------------------------------------
    # MOTION
    # --------------------------------------------------------

    if not vg10_cycle["active"]:

        begin_motion(
            step
        )


    vg10_cycle["elapsed"] += dt


    duration = max(
        0.01,
        float(
            step["duration"]
        )
    )


    t = (
        vg10_cycle["elapsed"]
        / duration
    )


    s = smoothstep(
        t
    )


    start = vg10_cycle["start"]
    target = vg10_cycle["target"]


    values = []


    for i in range(6):

        value = (
            start[i]
            + (
                target[i]
                - start[i]
            )
            * s
        )

        values.append(
            value
        )


    write_joint_targets(
        values
    )


    if t >= 1.0:

        write_joint_targets(
            target
        )

        vg10_cycle["index"] += 1
        vg10_cycle["active"] = False


        if vg10_cycle["index"] >= len(steps):

            print("")
            print("========================================")
            print("M0609 + VG10 CYCLE COMPLETE")
            print("========================================")
            print("FINAL BUTTON COMPLETE")
            print("========================================")


# ============================================================
# STOP OLD CONTROLLER
# ============================================================

KEY = "_m0609_vg10_final_tray_path"


old = getattr(
    builtins,
    KEY,
    None
)


if old is not None:

    try:
        old.unsubscribe()
    except:
        pass


# ============================================================
# REMOVE LEFTOVER GRIP JOINT
# ============================================================

if stage.GetPrimAtPath(
    GRIP_JOINT_PATH
).IsValid():

    stage.RemovePrim(
        GRIP_JOINT_PATH
    )


# ============================================================
# START
# ============================================================

subscription = (
    omni.kit.app
    .get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(
        on_vg10_cycle_update,
        name="M0609_VG10_FINAL_TRAY_PATH"
    )
)


setattr(
    builtins,
    KEY,
    subscription
)


print("")
print("========================================")
print("M0609 + VG10 FINAL PATH STARTED")
print("========================================")
print("PLAY MUST BE ON")
print("")
print("VISION =", VISION_POSE)
print("GRASP  = [45.4, 17.7, 79.5, 7.4, 80, 45]")
print("")
print("VISION")
print("-> J3 -43")
print("-> J2 31")
print("-> LINEAR [J1 50, J2 7.4, J3 -90]")
print("-> LINEAR [J3 -63, J2 -20, J4 0, J6 60]")
print("-> VACUUM OFF")
print("-> J3 -10")
print("-> J1 90")
print("-> BUTTON READY")
print("-> BUTTON PRESS")
print("-> BUTTON RELEASE")
print("========================================")