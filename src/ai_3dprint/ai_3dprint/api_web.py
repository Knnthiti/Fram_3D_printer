import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import requests
import time

API = "https://g1tuesm.consolutechcloud.com/cards/latest"

class GetAPINode(Node):
    def __init__(self):
        super().__init__('get_api_node')
        self.publisher_ = self.create_publisher(Int32, 'api_input', 10)
        self.timer = self.create_timer(10.0, self.timer_callback)  # ทุก 10 วินาที

    def timer_callback(self):
        try:
            response = requests.get(API)
            if response.status_code == 200:
                data = response.json()

                if "status" in data:
                    msg = Int32()

                    if data["status"].upper() == "FAIL":
                        msg.data = 1
                        self.get_logger().info("FAIL detected -> api = 1")

                    elif data["status"].upper() == "NORMAL":
                        msg.data = 0
                        self.get_logger().info("NORMAL detected -> api = 0")

                    else:
                        msg.data = -1
                        self.get_logger().warn(f"Unknown status: {data['status']}")

                    self.publisher_.publish(msg)
                else:
                    self.get_logger().warn("No 'status' field in response.")
            else:
                self.get_logger().warn(f"API Error: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Request failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GetAPINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
