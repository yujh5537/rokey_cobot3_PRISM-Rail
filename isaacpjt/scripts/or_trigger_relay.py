#!/usr/bin/env python3
# ============================================================
# 관제 /order_event -> UDP 47138 -> Isaac 오케스트레이터
#   Isaac 내부 rclpy.spin_once 는 Kit 이벤트 루프에서 콜백이 불리지 않는다(실측).
#   그래서 구독을 Isaac 밖으로 빼고 UDP 로 찔러 넣는다.
#   실행: source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=136 \
#         && python3 or_trigger_relay.py
# ============================================================
import json, socket
import rclpy
from std_msgs.msg import String

UDP = ("127.0.0.1", 47138)
WATCH = {"SERVICE_START", "SERVICE_DONE", "CODE_CRIMSON", "SIM_DONE"}

rclpy.init()
node = rclpy.create_node("or_trigger_relay")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def on_event(msg):
    try:
        d = json.loads(msg.data)
    except Exception:
        return
    ev = d.get("event")
    if ev in WATCH:
        payload = json.dumps({"event": ev, "subject": d.get("subject"),
                              "sim_t": d.get("sim_t")})
        sock.sendto(payload.encode(), UDP)
        print("[relay] %-14s %s  t=%s" % (ev, d.get("subject"), d.get("sim_t")))

node.create_subscription(String, "/order_event", on_event, 50)
print("[relay] /order_event -> UDP 47138  (Ctrl+C to stop)")
rclpy.spin(node)
