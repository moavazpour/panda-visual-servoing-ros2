#!/usr/bin/env python3
"""Record ground-truth placement accuracy for every pick-and-place cycle.

The robot measures its own placement error with the camera. This node compares
the real cube and shelf poses from Gazebo, so the true accuracy can be reported.
"""
import csv
import json
import math
import os
import statistics
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

SHELF_HEIGHT = 0.10   # [m]
CUBE_SIZE = 0.05      # [m]
ON_SHELF_TOLERANCE = 0.01  # [m] cube height must match "standing on the shelf"


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quarter_turn_error(angle):
    """A cube looks the same every 90 degrees: map the error to [-45, 45] deg."""
    return math.remainder(angle, math.pi / 2.0)


class BenchmarkRecorder(Node):

    def __init__(self):
        super().__init__('benchmark_recorder')

        default_path = os.path.expanduser(
            f'~/arm_vs_benchmark_{datetime.now():%Y%m%d_%H%M%S}.csv')
        self.path = self.declare_parameter('output', default_path).value

        self.cube = None
        self.shelf = None
        self.true_errors_mm = []
        self.cycles = 0
        self.true_successes = 0

        self.create_subscription(TFMessage, '/model/aruco_cube/pose', self.on_poses, 10)
        self.create_subscription(TFMessage, '/model/place_shelf/pose', self.on_poses, 10)
        self.create_subscription(String, '/cycle_result', self.on_result, 10)

        self.file = open(self.path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'cycle', 'reported_success', 'true_success', 'estimated_error_mm',
            'true_error_mm', 'true_error_x_mm', 'true_error_y_mm', 'true_yaw_error_deg',
            'duration_s', 'swivel_deg', 'grasp_offset_mm'])

        self.get_logger().info(f'Writing benchmark results to {self.path}')

    def on_poses(self, msg):
        # Gazebo sends the pose of every entity; keep the two we need
        for t in msg.transforms:
            if t.child_frame_id == 'aruco_cube':
                self.cube = t.transform
            elif t.child_frame_id == 'place_shelf':
                self.shelf = t.transform

    def on_result(self, msg):
        result = json.loads(msg.data)
        if self.cube is None or self.shelf is None:
            self.get_logger().warn('No ground truth poses received yet, skipping this cycle')
            return

        c = self.cube.translation
        s = self.shelf.translation

        # Ground truth: is the cube really standing on the shelf?
        expected_z = s.z + SHELF_HEIGHT / 2.0 + CUBE_SIZE / 2.0
        on_shelf = abs(c.z - expected_z) < ON_SHELF_TOLERANCE

        dx = (c.x - s.x) * 1000.0
        dy = (c.y - s.y) * 1000.0
        error = math.hypot(dx, dy)
        yaw_error = math.degrees(quarter_turn_error(
            yaw_from_quaternion(self.cube.rotation) - yaw_from_quaternion(self.shelf.rotation)))

        self.cycles += 1
        if on_shelf:
            self.true_successes += 1
            self.true_errors_mm.append(error)

        self.writer.writerow([
            result['cycle'], result['success'], on_shelf, result['estimated_error_mm'],
            f'{error:.2f}', f'{dx:.2f}', f'{dy:.2f}', f'{yaw_error:.2f}', result['duration_s'],
            result.get('swivel_deg', ''), result.get('grasp_offset_mm', '')])
        self.file.flush()

        self.get_logger().info(
            f'Cycle {result["cycle"]}: on shelf={on_shelf}, true error {error:.1f} mm '
            f'(robot estimated {result["estimated_error_mm"]:.1f} mm), yaw {yaw_error:.1f} deg')

    def print_summary(self):
        if self.cycles == 0:
            return
        lines = [
            f'Cycles: {self.cycles}',
            f'True success rate: {self.true_successes}/{self.cycles} '
            f'({100.0 * self.true_successes / self.cycles:.0f}%)',
        ]
        if self.true_errors_mm:
            lines.append(
                f'True placement error: mean {statistics.mean(self.true_errors_mm):.1f} mm, '
                f'max {max(self.true_errors_mm):.1f} mm')
        self.get_logger().info('\n'.join(lines))

    def close(self):
        self.file.close()


def main():
    rclpy.init()
    node = BenchmarkRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()