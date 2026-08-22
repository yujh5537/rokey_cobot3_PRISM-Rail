import sys
import os
sys.path.append(os.path.abspath('src/rail_control_core'))
from rail_control_core import geometry

print(geometry.pose_to_xyz("BB-01", 0.0, True))
print(geometry.pose_to_xyz("BB-01", 4.53, True))
print(geometry.pose_to_xyz("BB-01", 0.0, False))
print(geometry.pose_to_xyz("BB-01", 4.53, False))
