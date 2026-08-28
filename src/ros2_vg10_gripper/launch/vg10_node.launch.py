import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="ros2_vg10_gripper",
                executable="vg10_node",
                name="vg10",
            ),
        ]
    )
