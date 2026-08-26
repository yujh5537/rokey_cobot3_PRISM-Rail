#!/usr/bin/env python3
# UDP(127.0.0.1:47136) -> ROS2 /or_station_event 릴레이
# 실행: source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=136 && python3 or_event_relay.py
import socket
import rclpy
from std_msgs.msg import String
rclpy.init()
node = rclpy.create_node("or_event_relay")
pub = node.create_publisher(String, "/or_station_event", 10)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 47136))
print("[relay] UDP 47136 -> /or_station_event (Ctrl+C to stop)")
while True:
    data, _ = sock.recvfrom(65536)
    m = String(); m.data = data.decode("utf-8")
    pub.publish(m)
    print("[relay] published:", m.data)
