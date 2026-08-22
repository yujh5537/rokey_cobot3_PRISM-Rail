import sys
from pxr import Usd, UsdGeom

stage = Usd.Stage.Open('/home/rokey/rokey_cobot3/isaacpjt/hos_shaft.usd')
if not stage:
    print("Could not open stage")
    sys.exit(1)

prim = stage.GetPrimAtPath('/World/Capsules/Capsule_01')
if not prim.IsValid():
    print("Capsule_01 not found")
else:
    xform = UsdGeom.Xformable(prim)
    print("Xform Ops for Capsule_01:")
    for op in xform.GetOrderedXformOps():
        print(f"  {op.GetOpName()} (Type: {op.GetOpType()}): {op.Get()}")

