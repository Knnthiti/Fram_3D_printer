#!/usr/bin/env python3
# ai_3dprint/three_d_print.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import requests
import time

API_KEY = "gl3ye-kMFFW6eRuDp98Y2cyME19SXApyId_Y8QuL_b0"
OCTOPRINT_URL = "http://192.168.145.103:5000"
HOST_URL = OCTOPRINT_URL
headers = {"Content-Type": "application/json", "X-Api-Key": API_KEY}

clear_gcode = [
    "G91", "G1 Z10", "G90",
    "G1 X110 Y218 F3000", "G1 Z1",
    "G90", "G1 X110 Y1 Z1 F2400"
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
    return requests.post(f"{OCTOPRINT_URL}/api/printer/command", headers=headers, json=data)


class ThreeDPrint(Node):
    def __init__(self):
        super().__init__("three_d_print")
        self.create_subscription(Int32, "detection_result", self.cv_callback, 10)
        self.create_subscription(Int32, "test_input", self.test_callback, 10)
        self.cv_value = 0
        self.test_value = 0

    def cv_callback(self, msg: Int32):
        self.cv_value = msg.data
        self.get_logger().info(f"[ThreeDPrint] Received CV={msg.data}")
        self.take_action()

    def test_callback(self, msg: Int32):
        self.test_value = msg.data
        self.get_logger().info(f"[ThreeDPrint] Received Test={msg.data}")
        self.take_action()

    def take_action(self):
        if self.cv_value == 1:
           try:
              cancel_job()
              time.sleep(60)  # รอ 1 นาที (20*60 วินาที)
              send_gcode(clear_gcode)
              set_active(False)
              self.get_logger().info("Executed cancel + clear + restart sequence (cv_value)")
           except Exception as e:
              self.get_logger().error(f"Printer action failed (cv_value): {e}")

        elif self.test_value == 1:
            try:
               cancel_job()
               time.sleep(5)  # รอ 5 วินาที
               send_gcode(clear_gcode)
               set_active(False)
               self.get_logger().info("Executed cancel + clear + restart sequence (test_value)")
            except Exception as e:
               self.get_logger().error(f"Printer action failed (test_value): {e}")

        else:
            self.get_logger().info("No action needed (all good).")


def main(args=None):
    rclpy.init(args=args)
    node = ThreeDPrint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
