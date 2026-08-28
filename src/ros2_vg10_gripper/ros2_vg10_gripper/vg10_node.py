import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from ros2_vg10_gripper.vg10_driver import VG10


class VG10Node(Node):
    def __init__(self, node_name: str = "vg10") -> None:
        super().__init__(node_name)
        # TODO: IP and port should be params
        self.vg10 = VG10("192.168.100.102", 0)

        self.grip_service = self.create_service(
            Trigger, f"/{node_name}/grip", self.grip_callback
        )
        self.release_service = self.create_service(
            Trigger, f"/{node_name}/release", self.release_callback
        )

    def grip_callback(self, request, response):
        # TODO(elvout): strength should be a param
        self.vg10.vg10_grip(2, 60)

        return response

    def release_callback(self, request, response):
        self.vg10.vg10_release(1, 1)

        return response


def main() -> None:
    rclpy.init()

    node = VG10Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
