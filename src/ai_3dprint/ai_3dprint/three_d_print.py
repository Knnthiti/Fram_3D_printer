#!/usr/bin/env python3
# ai_3dprint/three_d_print.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import requests
import time

API_KEY = "gl3ye-kMFFW6eRuDp98Y2cyME19SXApyId_Y8QuL_b0"
OCTOPRINT_URL = "https://ender1.serveo.net"
HOST_URL = OCTOPRINT_URL
headers = {"Content-Type": "application/json", "X-Api-Key": API_KEY}

clear_gcode = [
    "G91",
    "G1 Z10",
    "G90",
    "G1 X110 Y218 F3000",
    "G1 Z1",
    "G90",
    "G1 X110 Y1 Z1 F2400",
]


def set_active(active: bool):
    return requests.post(
        f"{HOST_URL}/plugin/continuousprint/set_active",
        headers={"X-Api-Key": API_KEY},
        data={"active": str(active).lower()},
    )


def cancel_job():
    data = {"command": "cancel"}
    return requests.post(f"{OCTOPRINT_URL}/api/job", headers=headers, json=data)


def send_gcode(commands):
    data = {"commands": commands}
    return requests.post(
        f"{OCTOPRINT_URL}/api/printer/command", headers=headers, json=data
    )


class ThreeDPrint(Node):
    def __init__(self):
        super().__init__("three_d_print")
        self.create_subscription(Int32, "detection_result", self.cv_callback, 10)
        self.create_subscription(Int32, "test_input", self.test_callback, 10)
        self.create_subscription(Int32, "api_input", self.api_callback, 10)
        self.cv_value = 0
        self.test_value = 0
        self.api_value = 0

    def cv_callback(self, msg: Int32):
        self.cv_value = msg.data
        self.get_logger().info(f"[ThreeDPrint] Received CV={msg.data}")
        if self.cv_value == 1:
            cancel_job()
            time.sleep(10)  # รอ 1 นาที (20*60 วินาที)
            send_gcode(clear_gcode)
            set_active(False)

    def test_callback(self, msg: Int32):
        self.test_value = msg.data
        self.get_logger().info(f"[ThreeDPrint] Received Test={msg.data}")
        if self.test_value == 1:
            cancel_job()
            time.sleep(5)  # รอ 5 วินาที
            send_gcode(clear_gcode)
            set_active(False)

    def api_callback(self, msg: Int32):
        self.api_value = msg.data
        self.get_logger().info(f"[ThreeDPrint] Received API={msg.data}")
        if self.api_value == 1:
            cancel_job()
            time.sleep(5)  # รอ 5 วินาที
            send_gcode(clear_gcode)
            set_active(False)


def main(args=None):
    rclpy.init(args=args)
    node = ThreeDPrint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
