#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlebot_landmark_slam.pipeline import Pipeline
from turtlebot_landmark_slam.ekf import ExtendedKalmanFilter


class EkfPipelineNode(Node):
    def __init__(self) -> None:
        super().__init__("ekf_pipeline")
        self.pipeline = Pipeline(self, ExtendedKalmanFilter())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfPipelineNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()