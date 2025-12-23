#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from keyboard_msgs.msg import Key

class ResetCaller(Node):
    def __init__(self):
        super().__init__('reset_caller')
        self.reset_service = '/reset'
        self.subscription = self.create_subscription(
            Key,
            'keydown',
            self.keydown_callback,
            10
        )
        self.get_logger().info("reset_caller node started, listening to 'keydown' topic.")

    def keydown_callback(self, msg):
        if msg.code == Key.KEY_C:
            self.get_logger().info("Received 'c' on keydown. Calling /reset service...")
            client = self.create_client(SetBool, self.reset_service)
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Service /reset not available.")
                return
            req = SetBool.Request()
            future = client.call_async(req)
            # rclpy.spin_until_future_complete(self, future)
            # if future.result() is not None:
            #     self.get_logger().info("/reset service called successfully.")
            # else:
            #     self.get_logger().error(f"Service call failed: {future.exception()}")

def main(args=None):
    rclpy.init(args=args)
    node = ResetCaller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
