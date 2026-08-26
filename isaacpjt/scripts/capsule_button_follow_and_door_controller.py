# ============================================================
# COMBINED CONTROLLER
# 1) CapsuleButtonPhysics root follows Capsule_XX
# 2) Physical red button toggles capsule doors
#
# Run this ONE file inside Isaac Sim Script Editor / Isaac Python.
# ============================================================

import builtins

import omni.usd
import omni.kit.app

from pxr import UsdGeom, Gf


# ============================================================
# C01 ~ C10 EXTERNAL BUTTON ROOT
# PERMANENT FOLLOW CONTROLLER
#
# DOES NOT MODIFY:
# - capsule movement
# - capsule rotation
# - door logic
#
# Only:
# CapsuleButtonPhysics/Capsule_XX
# follows
# Capsules/Capsule_XX
# ============================================================


stage = omni.usd.get_context().get_stage()


STATE_KEY = "_capsule_button_follow_controller"


# ============================================================
# STOP OLD FOLLOW CONTROLLER
# ============================================================

if hasattr(builtins, STATE_KEY):

    old = getattr(
        builtins,
        STATE_KEY
    )

    try:
        old["running"] = False
    except:
        pass

    try:
        old["subscription"] = None
    except:
        pass


# ============================================================
# LOAD CAPSULES / BUTTON ROOTS
# ============================================================

items = []


for i in range(1, 11):

    num = f"{i:02d}"


    capsule_path = (
        f"/World/Capsules/Capsule_{num}"
    )


    button_root_path = (
        f"/World/CapsuleButtonPhysics/Capsule_{num}"
    )


    capsule = stage.GetPrimAtPath(
        capsule_path
    )


    button_root = stage.GetPrimAtPath(
        button_root_path
    )


    if not capsule.IsValid():

        print(
            "SKIP capsule:",
            capsule_path
        )

        continue


    if not button_root.IsValid():

        print(
            "SKIP button root:",
            button_root_path
        )

        continue


    items.append(
        {
            "num": num,
            "capsule": capsule,
            "button_root": button_root,
        }
    )


# ============================================================
# WORLD MATRIX
# ============================================================

def get_world_matrix_follow(prim):

    return (
        UsdGeom.Xformable(prim)
        .ComputeLocalToWorldTransform(0)
    )


# ============================================================
# SET WORLD MATRIX
#
# BUTTON ROOT ONLY.
# ============================================================

def set_world_matrix_follow(
    prim,
    target_world
):

    parent = prim.GetParent()


    if (
        parent
        and parent.IsValid()
        and str(parent.GetPath()) != "/"
    ):

        parent_world = (
            UsdGeom.Xformable(parent)
            .ComputeLocalToWorldTransform(0)
        )


        local = (
            Gf.Matrix4d(target_world)
            * parent_world.GetInverse()
        )


    else:

        local = Gf.Matrix4d(
            target_world
        )


    xf = UsdGeom.Xformable(
        prim
    )


    matrix_op = None


    for op in xf.GetOrderedXformOps():

        if (
            op.GetOpType()
            == UsdGeom.XformOp.TypeTransform
        ):

            matrix_op = op

            break


    if matrix_op is None:

        matrix_op = (
            xf.MakeMatrixXform()
        )


    matrix_op.Set(
        local
    )


# ============================================================
# SYNC
# ============================================================

def sync_all_follow():

    for item in items:

        capsule_world = (
            get_world_matrix_follow(
                item["capsule"]
            )
        )


        set_world_matrix_follow(
            item["button_root"],
            capsule_world
        )


# Initial sync immediately
sync_all_follow()


# ============================================================
# STATE
# ============================================================

follow_state = {
    "running": True,
    "subscription": None,
}


setattr(
    builtins,
    STATE_KEY,
    follow_state
)


# ============================================================
# UPDATE
# ============================================================

def on_update_follow(event):

    if not follow_state["running"]:

        return


    sync_all_follow()


# ============================================================
# START
# ============================================================

stream = (
    omni.kit.app
    .get_app()
    .get_update_event_stream()
)


follow_state["subscription"] = (
    stream.create_subscription_to_pop(
        on_update_follow,
        name="CapsuleButtonPermanentFollow"
    )
)


print("")
print("========================================")
print("CAPSULE BUTTON FOLLOW ACTIVE")
print("========================================")

print(
    "Connected =",
    len(items),
    "/ 10"
)

for item in items:

    print(
        f"Capsule_{item['num']} -> Button root FOLLOW"
    )

print("")
print("Movement code = UNCHANGED")
print("Rotation code = UNCHANGED")
print("Door code     = UNCHANGED")
print("========================================")



import builtins

import omni.usd
import omni.kit.app

from pxr import UsdGeom, Gf


# ============================================================
# C01 ~ C10
# PHYSICAL RED BUTTON -> DOOR TOGGLE CONTROLLER
#
# FIRST PRESS  -> OPEN
# RELEASE      -> RE-ARM
# SECOND PRESS -> CLOSE
#
# DOES NOT MODIFY:
# - capsule movement
# - capsule rotation
# - button-follow controller
#
# Button structure:
#
# /World/CapsuleButtonPhysics/Capsule_XX
#     /Button
#
# Door structure:
#
# /World/Capsules/Capsule_XX/Model/DoorSystem/
#     Door_Pos_L
#     Door_Pos_R
#     Door_Neg_L
#     Door_Neg_R
#
# ============================================================


stage = omni.usd.get_context().get_stage()


STATE_KEY = "_physical_red_button_door_controller"


# ============================================================
# FIXED DOOR POSITIONS
#
# IMPORTANT:
# Absolute Y positions.
#
# NEVER add/subtract repeatedly.
# ============================================================

LEFT_CLOSED_Y = 0.0605
LEFT_OPEN_Y   = 0.1755

RIGHT_CLOSED_Y = -0.0605
RIGHT_OPEN_Y   = -0.1755


DOOR_ANIM_TIME = 0.70


# ============================================================
# BUTTON PRESS
#
# Prismatic travel:
# 0 -> +0.008 m
#
# Trigger after ~4 mm press.
# Rearm below ~2 mm.
# ============================================================

PRESS_DISTANCE = 0.0040
RELEASE_DISTANCE = 0.0020


# ============================================================
# STOP OLD CONTROLLER
# ============================================================

if hasattr(builtins, STATE_KEY):

    old = getattr(
        builtins,
        STATE_KEY
    )

    try:
        old["running"] = False
    except:
        pass

    try:
        old["subscription"] = None
    except:
        pass


# ============================================================
# HELPERS
# ============================================================

def get_world_matrix(prim):

    return (
        UsdGeom.Xformable(prim)
        .ComputeLocalToWorldTransform(0)
    )


def get_relative_position(
    child,
    parent
):

    child_world = get_world_matrix(
        child
    )

    parent_world = get_world_matrix(
        parent
    )


    local = (
        Gf.Matrix4d(child_world)
        * parent_world.GetInverse()
    )


    p = local.ExtractTranslation()


    return (
        float(p[0]),
        float(p[1]),
        float(p[2]),
    )


# ============================================================
# DOOR TRANSLATE ACCESS
# ============================================================

def find_translate_op(prim):

    xf = UsdGeom.Xformable(
        prim
    )


    for op in xf.GetOrderedXformOps():

        if (
            op.GetOpType()
            == UsdGeom.XformOp.TypeTranslate
        ):

            return op


    return None


def get_translation(prim):

    op = find_translate_op(
        prim
    )


    if op is not None:

        value = op.Get()


        return (
            float(value[0]),
            float(value[1]),
            float(value[2]),
        )


    # --------------------------------------------------------
    # Matrix fallback
    # --------------------------------------------------------

    xf = UsdGeom.Xformable(
        prim
    )


    for xop in xf.GetOrderedXformOps():

        if (
            xop.GetOpType()
            == UsdGeom.XformOp.TypeTransform
        ):

            matrix = xop.Get()


            if matrix is None:

                break


            p = (
                Gf.Matrix4d(matrix)
                .ExtractTranslation()
            )


            return (
                float(p[0]),
                float(p[1]),
                float(p[2]),
            )


    raise RuntimeError(
        "Door translation op NOT FOUND: "
        + str(
            prim.GetPath()
        )
    )


def set_y_only(
    prim,
    new_y
):

    xf = UsdGeom.Xformable(
        prim
    )


    # --------------------------------------------------------
    # Prefer existing Translate op.
    # --------------------------------------------------------

    for op in xf.GetOrderedXformOps():

        if (
            op.GetOpType()
            == UsdGeom.XformOp.TypeTranslate
        ):

            old = op.Get()


            x = float(old[0])
            z = float(old[2])


            # Preserve original USD vector precision.
            if isinstance(
                old,
                Gf.Vec3f
            ):

                value = Gf.Vec3f(
                    x,
                    float(new_y),
                    z
                )

            else:

                value = Gf.Vec3d(
                    x,
                    float(new_y),
                    z
                )


            op.Set(
                value
            )

            return


    # --------------------------------------------------------
    # Matrix fallback.
    # Preserve rotation / scale.
    # Change translation Y only.
    # --------------------------------------------------------

    for op in xf.GetOrderedXformOps():

        if (
            op.GetOpType()
            == UsdGeom.XformOp.TypeTransform
        ):

            raw = op.Get()


            if raw is None:

                raw = Gf.Matrix4d(
                    1.0
                )


            matrix = Gf.Matrix4d(
                raw
            )


            p = (
                matrix.ExtractTranslation()
            )


            matrix.SetTranslateOnly(
                Gf.Vec3d(
                    float(p[0]),
                    float(new_y),
                    float(p[2])
                )
            )


            op.Set(
                matrix
            )

            return


    raise RuntimeError(
        "No usable door XformOp: "
        + str(
            prim.GetPath()
        )
    )


# ============================================================
# LOAD C01 ~ C10
# ============================================================

capsules = []


for i in range(
    1,
    11
):

    num = f"{i:02d}"


    capsule_path = (
        f"/World/Capsules/Capsule_{num}"
    )


    button_root_path = (
        f"/World/CapsuleButtonPhysics/Capsule_{num}"
    )


    button_path = (
        button_root_path
        + "/Button"
    )


    door_root = (
        capsule_path
        + "/Model/DoorSystem"
    )


    paths = {

        "left_pos":
            door_root
            + "/Door_Pos_L",

        "left_neg":
            door_root
            + "/Door_Neg_L",

        "right_pos":
            door_root
            + "/Door_Pos_R",

        "right_neg":
            door_root
            + "/Door_Neg_R",
    }


    capsule = stage.GetPrimAtPath(
        capsule_path
    )


    button_root = stage.GetPrimAtPath(
        button_root_path
    )


    button = stage.GetPrimAtPath(
        button_path
    )


    if not capsule.IsValid():

        print(
            "SKIP C"
            + num
            + ": capsule missing"
        )

        continue


    if not button_root.IsValid():

        print(
            "SKIP C"
            + num
            + ": button root missing"
        )

        continue


    if not button.IsValid():

        print(
            "SKIP C"
            + num
            + ": Button missing"
        )

        continue


    doors = {}


    missing = False


    for key, path in paths.items():

        prim = stage.GetPrimAtPath(
            path
        )


        if not prim.IsValid():

            print(
                "SKIP C"
                + num
                + ": missing "
                + path
            )

            missing = True

            break


        doors[key] = prim


    if missing:

        continue


    # ========================================================
    # CALIBRATE BUTTON REST POSITION
    #
    # Script should be run while button is NOT pressed.
    # ========================================================

    rel = get_relative_position(
        button,
        button_root
    )


    rest_z = rel[2]


    # ========================================================
    # Determine initial door state from ABSOLUTE position.
    # ========================================================

    left_y = get_translation(
        doors["left_pos"]
    )[1]


    right_y = get_translation(
        doors["right_pos"]
    )[1]


    left_open_error = abs(
        left_y
        - LEFT_OPEN_Y
    )


    left_closed_error = abs(
        left_y
        - LEFT_CLOSED_Y
    )


    right_open_error = abs(
        right_y
        - RIGHT_OPEN_Y
    )


    right_closed_error = abs(
        right_y
        - RIGHT_CLOSED_Y
    )


    is_open = (
        left_open_error
        + right_open_error

        <

        left_closed_error
        + right_closed_error
    )


    capsules.append({

        "num":
            num,

        "capsule":
            capsule,

        "button_root":
            button_root,

        "button":
            button,

        "doors":
            doors,

        "rest_z":
            rest_z,

        "armed":
            True,

        "is_open":
            is_open,

        "door_anim":
            None,
    })


# ============================================================
# DOOR TARGETS
# ============================================================

def door_targets(
    open_state
):

    if open_state:

        return {

            "left_pos":
                LEFT_OPEN_Y,

            "left_neg":
                LEFT_OPEN_Y,

            "right_pos":
                RIGHT_OPEN_Y,

            "right_neg":
                RIGHT_OPEN_Y,
        }


    return {

        "left_pos":
            LEFT_CLOSED_Y,

        "left_neg":
            LEFT_CLOSED_Y,

        "right_pos":
            RIGHT_CLOSED_Y,

        "right_neg":
            RIGHT_CLOSED_Y,
    }


# ============================================================
# START ANIMATION
# ============================================================

def start_door_animation(
    item,
    target_open
):

    if item["door_anim"] is not None:

        return


    starts = {}


    for key, prim in item[
        "doors"
    ].items():

        starts[key] = (
            get_translation(
                prim
            )[1]
        )


    targets = door_targets(
        target_open
    )


    item["door_anim"] = {

        "elapsed":
            0.0,

        "duration":
            DOOR_ANIM_TIME,

        "start":
            starts,

        "target":
            targets,

        "target_open":
            target_open,
    }


    print("")
    print(
        "========================================"
    )


    if target_open:

        print(
            f"C{item['num']} BUTTON PRESS"
        )

        print(
            "DOOR -> OPEN"
        )


    else:

        print(
            f"C{item['num']} BUTTON PRESS"
        )

        print(
            "DOOR -> CLOSE"
        )


    print(
        "========================================"
    )


# ============================================================
# UPDATE DOOR ANIMATION
# ============================================================

def update_door_animation(
    item,
    dt
):

    anim = item.get(
        "door_anim"
    )


    if anim is None:

        return


    anim["elapsed"] += (
        dt
    )


    t = (
        anim["elapsed"]
        / anim["duration"]
    )


    if t > 1.0:

        t = 1.0


    # --------------------------------------------------------
    # Smoothstep
    # --------------------------------------------------------

    smooth = (
        t * t
        * (
            3.0
            - 2.0 * t
        )
    )


    for key, prim in item[
        "doors"
    ].items():

        y0 = anim[
            "start"
        ][key]


        y1 = anim[
            "target"
        ][key]


        y = (
            y0
            + (
                y1 - y0
            ) * smooth
        )


        set_y_only(
            prim,
            y
        )


    # ========================================================
    # COMPLETE
    # ========================================================

    if t >= 1.0:

        target_open = anim[
            "target_open"
        ]


        # ----------------------------------------------------
        # Force exact final absolute coordinates.
        # ----------------------------------------------------

        targets = door_targets(
            target_open
        )


        for key, prim in item[
            "doors"
        ].items():

            set_y_only(
                prim,
                targets[key]
            )


        item["is_open"] = (
            target_open
        )


        item["door_anim"] = (
            None
        )


        print("")


        if target_open:

            print(
                f"C{item['num']} DOOR OPEN COMPLETE"
            )


        else:

            print(
                f"C{item['num']} DOOR CLOSED COMPLETE"
            )

            print(
                "EVENT = DOOR_CLOSED"
            )

            print(
                "CAPSULE = C"
                + item["num"]
            )

            print(
                "CONTROL CORE SIGNAL POINT"
            )


# ============================================================
# STATE
# ============================================================

state = {

    "running":
        True,

    "items":
        capsules,

    "subscription":
        None,
}


setattr(
    builtins,
    STATE_KEY,
    state
)


# ============================================================
# UPDATE
# ============================================================

def on_update(event):

    if not state[
        "running"
    ]:

        return


    try:

        dt = float(
            event.payload.get(
                "dt",
                1.0 / 60.0
            )
        )


    except:

        dt = (
            1.0 / 60.0
        )


    # ========================================================
    # ALL CAPSULES
    # ========================================================

    for item in state[
        "items"
    ]:

        # ----------------------------------------------------
        # Door animation
        # ----------------------------------------------------

        update_door_animation(
            item,
            dt
        )


        # ----------------------------------------------------
        # Read physical button position relative to its root.
        # ----------------------------------------------------

        try:

            rel = get_relative_position(
                item["button"],
                item["button_root"]
            )


        except:

            continue


        press_distance = (
            rel[2]
            - item["rest_z"]
        )


        # ====================================================
        # RELEASE -> REARM
        # ====================================================

        if (
            press_distance
            <= RELEASE_DISTANCE
        ):

            item["armed"] = (
                True
            )


        # ====================================================
        # PHYSICAL PRESS
        # ====================================================

        if (
            press_distance
            >= PRESS_DISTANCE
            and item["armed"]
            and item["door_anim"] is None
        ):

            item["armed"] = (
                False
            )


            # ------------------------------------------------
            # Toggle
            # ------------------------------------------------

            target_open = (
                not item["is_open"]
            )


            start_door_animation(
                item,
                target_open
            )


# ============================================================
# RUN
# ============================================================

stream = (
    omni.kit.app
    .get_app()
    .get_update_event_stream()
)


state["subscription"] = (
    stream.create_subscription_to_pop(
        on_update,
        name="PhysicalRedButtonDoorController"
    )
)


# ============================================================
# PRINT
# ============================================================

print("")
print("========================================")
print("PHYSICAL RED BUTTON DOOR CONTROL ACTIVE")
print("========================================")

print(
    "Connected =",
    len(capsules),
    "/ 10"
)

print("")
print(
    "PRESS 1 -> OPEN"
)

print(
    "RELEASE -> RE-ARM"
)

print(
    "PRESS 2 -> CLOSE"
)

print("")
print(
    "Press threshold =",
    PRESS_DISTANCE,
    "m"
)

print(
    "Release threshold =",
    RELEASE_DISTANCE,
    "m"
)

print(
    "Door animation =",
    DOOR_ANIM_TIME,
    "sec"
)

print("")


for item in capsules:

    print(
        f"C{item['num']}",
        "button rest Z =",
        round(
            item["rest_z"],
            5
        ),
        "| initial door =",
        (
            "OPEN"
            if item["is_open"]
            else "CLOSED"
        )
    )


print("")
print("Movement = UNCHANGED")
print("Rotation = UNCHANGED")
print("Button follow = UNCHANGED")
print("========================================")