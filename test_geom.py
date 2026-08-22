import sys
import os
sys.path.append(os.path.abspath('src/rail_control_core'))
from rail_control_core import geometry
print(geometry.pose_to_xyz("BB-01", 2.0, True))
print(geometry.pose_to_xyz("BB-09", 2.0, True))
