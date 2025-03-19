#!/usr/bin/env python3

import rclpy
import math
import sys
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserListener(Node):
    def __init__(self, topic_name, output="overview"):
        super().__init__('laser_listener')
        self.output = output
        self.subscription = self.create_subscription(
            LaserScan,
            topic_name,
            self.callback_scan,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning

    def callback_scan(self, data):
        if self.output == "overview":
            self.get_logger().info(f"Topic {self.subscription.topic_name} has data from: {data.angle_min} rad or {math.degrees(data.angle_min)}°")
            self.get_logger().info(f"                       to: {data.angle_max} rad or {math.degrees(data.angle_max)}°")
            self.get_logger().info(f"       with increments of: {data.angle_increment} rad or {math.degrees(data.angle_increment)}°")
            self.get_logger().info(f"      and a range between: {data.range_min} and {data.range_max}")
            self.get_logger().info(f"                 in total: {len(data.ranges)} values\n")
        elif self.output == "data":
            index = 0
            object_found = False
            for value in data.ranges:
                if not math.isinf(value):
                    if not object_found:
                        object_found = True
                    current_angle = data.angle_min + (data.angle_increment * index)
                    self.get_logger().info(f"Found an object at {math.degrees(current_angle)}°, distance: {value}")
                index += 1

            if not object_found:
                self.get_logger().info("No object(s) found in range.")


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) == 3:
        topic_name = str(sys.argv[1])
        argument = str(sys.argv[2])
    else:
        print("Usage:\n    python3 script.py <topic_name> overview\n Or:\n    python3 script.py <topic_name> data")
        sys.exit(1)

    node = LaserListener(topic_name, argument)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

