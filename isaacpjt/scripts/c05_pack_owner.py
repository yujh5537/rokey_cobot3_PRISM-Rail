import builtins
import json

import omni.usd
import omni.kit.app
import omni.kit.commands

from pxr import UsdGeom, Gf

import rclpy
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from std_msgs.msg import String


# ============================================================
# C05 SURGICAL PACK OWNERSHIP
#
# START:
#   /World/Cargo_OR2/SurgicalPack
#        ↓
#   /World/Capsules/Capsule_05/SurgicalPack
#
# RELEASE:
#   ONLY when:
#   {"capsule_id":"C05","event":"PACK_GRIPPED"}
#
# DOOR_OPEN DOES NOT RELEASE CARGO.
# ============================================================


stage = omni.usd.get_context().get_stage()


# ============================================================
# PATHS
# ============================================================

SOURCE_PACK_PATH = "/World/Cargo_OR2"          # 래퍼째 이동 (참조 자식은 MovePrim 불가)

C05_ROOT_PATH = "/World/Capsules/Capsule_05"

C05_PACK_PATH = "/World/Capsules/Capsule_05/Cargo_OR2"

FREE_PARENT_PATH = "/World/Cargo_OR2"

FREE_PACK_PATH = "/World/Cargo_OR2"


# ============================================================
# C05 LOADED LOCAL POSE
# ============================================================

PACK_LOCAL_POSITION = Gf.Vec3d(
    -0.0399535,
    0.00299,
    -0.39,
)

PACK_LOCAL_ROTATION = Gf.Vec3d(
    0.0,
    0.0,
    0.0,
)


# ============================================================
# STATE
# ============================================================

OWNER = {
    "state": "UNKNOWN",
    "released": False,
}


# ============================================================
# TRANSFORM HELPERS
# ============================================================

def get_world_matrix(path):

    prim = stage.GetPrimAtPath(path)

    if not prim.IsValid():
        return None

    return (
        UsdGeom.Xformable(prim)
        .ComputeLocalToWorldTransform(
            UsdGeom.XformCache.GetTime()
        )
    )


def set_local_pose(
    path,
    position,
    rotation,
):

    prim = stage.GetPrimAtPath(path)

    if not prim.IsValid():
        print(
            "[C05 CARGO] INVALID:",
            path
        )
        return False

    xformable = UsdGeom.Xformable(prim)

    # Clear only the xform op order on SurgicalPack root.
    # Children Body / Label are untouched.
    xformable.ClearXformOpOrder()

    translate_op = (
        xformable.AddTranslateOp()
    )

    rotate_op = (
        xformable.AddRotateXYZOp()
    )

    translate_op.Set(
        Gf.Vec3d(
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )
    )

    rotate_op.Set(
        Gf.Vec3f(
            float(rotation[0]),
            float(rotation[1]),
            float(rotation[2]),
        )
    )

    return True


def set_world_matrix(
    path,
    world_matrix,
):

    prim = stage.GetPrimAtPath(path)

    if not prim.IsValid():
        return False

    parent = prim.GetParent()

    if parent and parent.IsValid():

        parent_world = (
            UsdGeom.Xformable(parent)
            .ComputeLocalToWorldTransform(
                UsdGeom.XformCache.GetTime()
            )
        )

        local_matrix = (
            world_matrix
            * parent_world.GetInverse()
        )

    else:

        local_matrix = world_matrix


    xformable = UsdGeom.Xformable(prim)

    xformable.ClearXformOpOrder()

    matrix_op = (
        xformable.AddTransformOp()
    )

    matrix_op.Set(
        Gf.Matrix4d(local_matrix)
    )

    return True


# ============================================================
# MOVE PRIM
# ============================================================

def move_prim(
    src,
    dst,
):

    if src == dst:
        return True

    src_prim = stage.GetPrimAtPath(src)

    if not src_prim.IsValid():
        return False

    try:

        omni.kit.commands.execute(
            "MovePrim",
            path_from=src,
            path_to=dst,
        )

        return (
            stage.GetPrimAtPath(dst)
            .IsValid()
        )

    except Exception as e:

        print(
            "[C05 CARGO] MovePrim error:",
            e
        )

        return False


# ============================================================
# INITIAL LOAD INTO C05
# ============================================================

def load_pack_into_c05():

    print("")
    print("========================================")
    print("C05 SURGICAL PACK INITIAL LOAD")
    print("========================================")


    c05 = stage.GetPrimAtPath(
        C05_ROOT_PATH
    )

    if not c05.IsValid():

        print(
            "[ERROR] C05 NOT FOUND:",
            C05_ROOT_PATH
        )

        return False


    # --------------------------------------------------------
    # Already loaded
    # --------------------------------------------------------

    loaded = stage.GetPrimAtPath(
        C05_PACK_PATH
    )

    if loaded.IsValid():

        print(
            "[INFO] SurgicalPack already under C05."
        )


    # --------------------------------------------------------
    # Move from Cargo_OR2 into C05
    # --------------------------------------------------------

    else:

        source = stage.GetPrimAtPath(
            SOURCE_PACK_PATH
        )

        if not source.IsValid():

            print(
                "[ERROR] SurgicalPack not found."
            )

            print(
                "Checked:",
                SOURCE_PACK_PATH
            )

            return False


        ok = move_prim(
            SOURCE_PACK_PATH,
            C05_PACK_PATH,
        )

        if not ok:

            print(
                "[ERROR] Failed to move SurgicalPack."
            )

            return False


        print(
            "[OK] SurgicalPack moved under C05."
        )


    # --------------------------------------------------------
    # Fixed local pose inside C05
    # --------------------------------------------------------

    ok = set_local_pose(
        C05_PACK_PATH,
        PACK_LOCAL_POSITION,
        PACK_LOCAL_ROTATION,
    )

    if not ok:
        return False


    OWNER["state"] = "CAPSULE"
    OWNER["released"] = False


    print("")
    print("OWNER = CAPSULE")
    print("PACK =", C05_PACK_PATH)
    print(
        "LOCAL POS =",
        PACK_LOCAL_POSITION
    )
    print(
        "LOCAL ROT =",
        PACK_LOCAL_ROTATION
    )
    print("")
    print(
        "DOOR_OPEN will NOT release cargo."
    )
    print(
        "Waiting for PACK_GRIPPED..."
    )
    print("========================================")

    return True


# ============================================================
# RELEASE AFTER PACK_GRIPPED
# ============================================================

def release_pack_from_c05():

    if OWNER["released"]:

        print(
            "[C05 CARGO] Already released."
        )

        return


    pack = stage.GetPrimAtPath(
        C05_PACK_PATH
    )

    if not pack.IsValid():

        print(
            "[C05 CARGO] Pack is not under C05."
        )

        return


    # --------------------------------------------------------
    # Save current WORLD pose
    # --------------------------------------------------------

    cache = UsdGeom.XformCache()

    world_matrix = (
        cache.GetLocalToWorldTransform(
            pack
        )
    )


    print("")
    print("========================================")
    print("PACK_GRIPPED RECEIVED")
    print("========================================")
    print(
        "Releasing SurgicalPack from C05..."
    )


    # --------------------------------------------------------
    # C05 → /World/Cargo_OR2
    # --------------------------------------------------------

    ok = move_prim(
        C05_PACK_PATH,
        FREE_PACK_PATH,
    )

    if not ok:

        print(
            "[ERROR] Failed to release pack."
        )

        return


    # --------------------------------------------------------
    # Restore exact WORLD transform
    # Prevent visual jump on ownership transfer
    # --------------------------------------------------------

    set_world_matrix(
        FREE_PACK_PATH,
        world_matrix,
    )


    OWNER["state"] = "ROBOT"
    OWNER["released"] = True


    print(
        "[OK] C05 coordinate lock released."
    )
    print(
        "OWNER = ROBOT"
    )
    print(
        "PACK =",
        FREE_PACK_PATH
    )
    print(
        "World pose preserved."
    )
    print("========================================")


# ============================================================
# ROS EVENT CALLBACK
# ============================================================

def on_or_station_event(msg):

    try:

        data = json.loads(
            msg.data
        )

    except Exception:

        return


    capsule_id = str(
        data.get(
            "capsule_id",
            ""
        )
    )

    event = str(
        data.get(
            "event",
            ""
        )
    ).upper()


    if capsule_id != "C05":
        return


    # --------------------------------------------------------
    # DOOR_OPEN
    #
    # IMPORTANT:
    # Work permission only.
    # Cargo remains attached to C05.
    # --------------------------------------------------------

    if event == "DOOR_OPEN":

        print("")
        print(
            "[C05 CARGO] DOOR_OPEN"
        )

        print(
            "[C05 CARGO] Cargo remains locked."
        )

        return


    # --------------------------------------------------------
    # PACK_GRIPPED
    #
    # Robot physically owns cargo now.
    # This is the ONLY release point.
    # --------------------------------------------------------

    if event == "PACK_GRIPPED":

        release_pack_from_c05()

        return


    # --------------------------------------------------------
    # PACK_UNLOADED
    # --------------------------------------------------------

    if event == "PACK_UNLOADED":

        OWNER["state"] = "TRAY"

        print("")
        print(
            "[C05 CARGO] PACK_UNLOADED"
        )

        print(
            "[C05 CARGO] OWNER = TRAY"
        )

        return


    # --------------------------------------------------------
    # DOOR_CLOSED
    # --------------------------------------------------------

    if event == "DOOR_CLOSED":

        print("")
        print(
            "[C05 CARGO] DOOR_CLOSED"
        )

        return


# ============================================================
# CLEAN PREVIOUS INSTANCE
# ============================================================

OLD_SUB_KEY = (
    "_c05_pack_owner_update_sub"
)

OLD_NODE_KEY = (
    "_c05_pack_owner_ros_node"
)


old_update_sub = getattr(
    builtins,
    OLD_SUB_KEY,
    None,
)

if old_update_sub is not None:

    try:
        old_update_sub.unsubscribe()
    except Exception:
        pass

    setattr(
        builtins,
        OLD_SUB_KEY,
        None,
    )


old_node = getattr(
    builtins,
    OLD_NODE_KEY,
    None,
)

if old_node is not None:

    try:
        old_node.destroy_node()
    except Exception:
        pass

    setattr(
        builtins,
        OLD_NODE_KEY,
        None,
    )


# ============================================================
# ROS INIT
# ============================================================

if not rclpy.ok():

    rclpy.init(
        args=None
    )


node = rclpy.create_node(
    "c05_pack_owner_scene"
)


qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


ros_sub = node.create_subscription(
    String,
    "/or_station_event",
    on_or_station_event,
    qos,
)


setattr(
    builtins,
    OLD_NODE_KEY,
    node,
)


# ============================================================
# ISAAC UPDATE → ROS SPIN
# ============================================================

def on_update(_event):

    try:

        if rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.0,
            )

    except Exception as e:

        print(
            "[C05 CARGO] ROS spin error:",
            e
        )


update_sub = (
    omni.kit.app
    .get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(
        on_update,
        name="C05 SurgicalPack Ownership",
    )
)


setattr(
    builtins,
    OLD_SUB_KEY,
    update_sub,
)


# ============================================================
# INITIALIZE CARGO
# ============================================================

load_pack_into_c05()


print("")
print("========================================")
print("C05 CARGO OWNER LISTENER READY")
print("========================================")
print("Topic : /or_station_event")
print("C05 only")
print("")
print("DOOR_OPEN     -> KEEP LOCKED")
print("PACK_GRIPPED  -> RELEASE TO ROBOT")
print("PACK_UNLOADED -> OWNER TRAY")
print("DOOR_CLOSED   -> NO CARGO CHANGE")
print("========================================")