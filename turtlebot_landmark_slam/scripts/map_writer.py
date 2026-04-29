#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


# Run this node when your map is complete.
# It will generate a text file with the landmark poses.
class MapWriter(Node):
    def __init__(self):
        super().__init__("map_writer")
        self.create_subscription(MarkerArray, "/ekf/map", self.callback, 1)

    def callback(self, data: MarkerArray):
        self.get_logger().info("Gotten MarkerArray callback. Writing map to map")
        with open("map_slam.txt", "w") as map_file:
            for marker in data.markers:
                line = "POINT2D %s %.3f %.3f \n" % (
                    marker.id,
                    marker.pose.position.x,
                    marker.pose.position.y,
                )
                map_file.write(line)

        self.get_logger().info("Map written in map_slam.txt")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MapWriter()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()