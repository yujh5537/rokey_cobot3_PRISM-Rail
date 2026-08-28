from glob import glob

from setuptools import setup

package_name = "ros2_vg10_gripper"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        # Register package with the ament index
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # Install package.xml for tooling
        (f"share/{package_name}", ["package.xml"]),
        # launch files, params, etc.
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
        (f"share/{package_name}/meshes", glob("meshes/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Elvin Yang",
    maintainer_email="eyy@umich.edu",
    description="Basic ROS2 Support for the OnRobot VG10 Gripper with a UR Robot",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "vg10_node = ros2_vg10_gripper.vg10_node:main",
        ],
    },
)
