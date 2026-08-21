"""control_core.launch.py — 관제 코어 실행.

사용 예:
    ros2 launch rail_control_core control_core.launch.py
    ros2 launch rail_control_core control_core.launch.py mode:=A speed_scale:=2.0
    ros2 launch rail_control_core control_core.launch.py autostart:=false   # 수동 시작
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

PKG = "rail_control_core"


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PKG)

    args = [
        DeclareLaunchArgument("mode", default_value="B",
                              description="A=FCFS(비교군) / B=P-EDD 계단식 선점"),
        DeclareLaunchArgument("speed_scale", default_value="1.0",
                              description="시뮬레이션 배속"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="기동과 동시에 시나리오 진행"),
        DeclareLaunchArgument("rate_hz", default_value="30.0",
                              description="틱·발행 주기(Hz) — 엔진 DT 와 맞춰 실시간"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    node = Node(
        package=PKG,
        executable="control_core",
        name="control_core_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "config_dir": PathJoinSubstitution([share, "config"]),
            "mode": LaunchConfiguration("mode"),
            "speed_scale": LaunchConfiguration("speed_scale"),
            "autostart": LaunchConfiguration("autostart"),
            "rate_hz": LaunchConfiguration("rate_hz"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription([*args, node])
