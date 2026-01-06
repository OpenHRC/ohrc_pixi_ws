import random
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from ohrc_msgs.msg import State

import numpy as np

#!/usr/bin/env python3
# /home/itadera/myros/src/humble/ohrc_ws/src/OpenHRC/ohrc_tools/ohrc_tools/reaching_game.py



class RandomMarkerPublisher(Node):
    def __init__(self):
        super().__init__('random_marker_publisher')
        self.pub = self.create_publisher(Marker, 'marker', 10)
        self.sub = self.create_subscription(State, '/state/current', self.state_callback, 10)
        self.next_id = 0
        
        self.update_marker()
        self.average_time = 0.0
        self.attempts = 0

    def state_callback(self, msg):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        m_pos = np.array([self.marker.pose.position.x, self.marker.pose.position.y, self.marker.pose.position.z])
        dist = np.linalg.norm(pos - m_pos)
        if dist < 0.05:
            t = self.get_clock().now() - self.timer_start
            self.attempts += 1
            self.average_time += (t.nanoseconds / 1e9 - self.average_time) / self.attempts
            self.get_logger().info(f'Average time: {self.average_time:.1f} seconds over {self.attempts} attempts')
            # self.get_logger().info('Marker reached! Generating new marker.')
            self.update_marker()

        self.pub.publish(self.marker)

    def update_marker(self):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'random_markers'
        m.id = 0

        m.type = Marker.SPHERE
        m.action = Marker.ADD

        # Random position within bounds (change as needed)
        m.pose.position.x = random.uniform(0.2, 0.6)
        m.pose.position.y = random.uniform(-0.2, 0.2)
        m.pose.position.z = random.uniform(0.0, 0.4)
        m.pose.orientation.w = 1.0

        # Size and color
        s = 0.08
        m.scale.x = s
        m.scale.y = s
        m.scale.z = s
        m.color.r = random.random()
        m.color.g = random.random()
        m.color.b = random.random()
        m.color.a = 1.0

        # lifetime 0 -> persistent until replaced/removed
        m.lifetime = Duration(sec=0, nanosec=0)
        self.marker = m

        self.timer_start = self.get_clock().now()

        


def main(args=None):
    rclpy.init(args=args)
    node = RandomMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()