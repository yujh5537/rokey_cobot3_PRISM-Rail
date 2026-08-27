import omni.usd
import omni.kit.app

from pxr import Usd, UsdGeom, UsdPhysics, Gf


stage = omni.usd.get_context().get_stage()

EE   = "/World/m0609/onrobot_rg2ft"
PACK = "/World/Cargo_OR2/SurgicalPack"


_FOLLOW = {
    "attached": False,
    "rel": None,
    "op": None,

    "rb_enabled": None,
    "kinematic": None,

    "collisions": {},
}


# ============================================================
# WORLD MATRIX
# ============================================================

def _world(path):
    prim = stage.GetPrimAtPath(path)

    if not prim.IsValid():
        raise RuntimeError("Prim missing: " + path)

    cache = UsdGeom.XformCache(
        Usd.TimeCode.Default()
    )

    return cache.GetLocalToWorldTransform(prim)


# ============================================================
# PACK XFORM OP
# ============================================================

def _ensure_op():

    if _FOLLOW["op"] is not None:
        return _FOLLOW["op"]

    prim = stage.GetPrimAtPath(PACK)

    xf = UsdGeom.Xformable(prim)

    # 현재 LOCAL transform 저장
    local = xf.GetLocalTransformation()

    xf.ClearXformOpOrder()

    op = xf.AddTransformOp()
    op.Set(Gf.Matrix4d(local))

    _FOLLOW["op"] = op

    return op


# ============================================================
# SET PACK WORLD
# ============================================================

def _set_world(desired_world):

    prim = stage.GetPrimAtPath(PACK)
    parent = prim.GetParent()

    cache = UsdGeom.XformCache(
        Usd.TimeCode.Default()
    )

    parent_world = cache.GetLocalToWorldTransform(
        parent
    )

    # USD 계층 기준 world -> local
    local = (
        desired_world
        * parent_world.GetInverse()
    )

    _ensure_op().Set(
        Gf.Matrix4d(local)
    )


# ============================================================
# COLLISION OFF
# ============================================================

def _disable_collisions():

    root = stage.GetPrimAtPath(PACK)

    _FOLLOW["collisions"] = {}

    for prim in Usd.PrimRange(root):

        if not prim.HasAPI(
            UsdPhysics.CollisionAPI
        ):
            continue

        api = UsdPhysics.CollisionAPI(prim)

        attr = api.GetCollisionEnabledAttr()

        old = attr.Get()

        if old is None:
            old = True

        path = str(prim.GetPath())

        _FOLLOW["collisions"][path] = bool(old)

        attr.Set(False)


def _restore_collisions():

    for path, value in _FOLLOW[
        "collisions"
    ].items():

        prim = stage.GetPrimAtPath(path)

        if not prim.IsValid():
            continue

        if not prim.HasAPI(
            UsdPhysics.CollisionAPI
        ):
            continue

        UsdPhysics.CollisionAPI(
            prim
        ).GetCollisionEnabledAttr().Set(
            value
        )

    _FOLLOW["collisions"] = {}


# ============================================================
# ATTACH
# ============================================================

def attach():

    pack = stage.GetPrimAtPath(PACK)
    ee   = stage.GetPrimAtPath(EE)

    if not pack.IsValid():
        print("[DIRECT ERROR] PACK missing")
        return

    if not ee.IsValid():
        print("[DIRECT ERROR] EE missing")
        return


    # --------------------------------------------------------
    # attach 순간 상대 위치 먼저 저장
    # --------------------------------------------------------

    pack_world = _world(PACK)
    ee_world   = _world(EE)

    _FOLLOW["rel"] = (
        pack_world
        * ee_world.GetInverse()
    )


    # --------------------------------------------------------
    # 물리 완전히 OFF
    #
    # 이제 PhysX가 PACK 위치를 덮어쓸 수 없음
    # --------------------------------------------------------

    if pack.HasAPI(
        UsdPhysics.RigidBodyAPI
    ):

        rb = UsdPhysics.RigidBodyAPI(pack)

        enabled_attr = rb.GetRigidBodyEnabledAttr()
        kin_attr = rb.GetKinematicEnabledAttr()

        _FOLLOW["rb_enabled"] = enabled_attr.Get()
        _FOLLOW["kinematic"] = kin_attr.Get()

        enabled_attr.Set(False)

        print("[DIRECT] RigidBody OFF")


    # 이동 중 충돌도 OFF
    _disable_collisions()

    _ensure_op()

    _FOLLOW["attached"] = True


    print("")
    print("========================================")
    print("[ATTACH DIRECT FORCE]")
    print("PACK =", PACK)
    print("RigidBody = OFF")
    print("Collision = OFF")
    print("Follow = ON")
    print("========================================")


# ============================================================
# RELEASE
# ============================================================

def release():

    if not _FOLLOW["attached"]:
        print("[DIRECT] already released")
        return


    # follow 먼저 종료
    _FOLLOW["attached"] = False


    # collision 복구
    _restore_collisions()


    pack = stage.GetPrimAtPath(PACK)

    # rigid body 원래대로 복구
    if pack.HasAPI(
        UsdPhysics.RigidBodyAPI
    ):

        rb = UsdPhysics.RigidBodyAPI(pack)

        if _FOLLOW["kinematic"] is not None:
            rb.GetKinematicEnabledAttr().Set(
                _FOLLOW["kinematic"]
            )

        if _FOLLOW["rb_enabled"] is not None:
            rb.GetRigidBodyEnabledAttr().Set(
                _FOLLOW["rb_enabled"]
            )


    print("")
    print("========================================")
    print("[RELEASE DIRECT FORCE]")
    print("Follow = OFF")
    print("Collision restored")
    print("RigidBody restored")
    print("========================================")


# ============================================================
# COMPATIBILITY
# ============================================================

def busy():
    return False


# ============================================================
# EVERY FRAME
# ============================================================

_counter = 0

def _tick(event):

    global _counter

    if not _FOLLOW["attached"]:
        return

    try:

        ee_world = _world(EE)

        target = (
            _FOLLOW["rel"]
            * ee_world
        )

        _set_world(target)

        _counter += 1

        # follow가 실제 실행되는지 너무 많이 찍히지 않게
        if _counter % 60 == 0:

            p = target.ExtractTranslation()

            print(
                "[FOLLOW]",
                round(float(p[0]), 3),
                round(float(p[1]), 3),
                round(float(p[2]), 3)
            )

    except Exception as e:

        print(
            "[FOLLOW ERROR]",
            repr(e)
        )


_sub = (
    omni.kit.app.get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(
        _tick,
        name="OR_SurgicalPack_DirectForceFollow"
    )
)


print("")
print("========================================")
print("DIRECT FORCE ATTACH READY")
print("EE   =", EE)
print("PACK =", PACK)
print("========================================")
