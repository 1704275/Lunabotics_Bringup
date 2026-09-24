#!/usr/bin/env python3
"""Starter node for the Lunabotics ROS 2 case study.

Fill in TASKS 1-3 here. See README.md for the full description of each task.

Run it with:

    ros2 run move publisher

As shipped this node starts, spins, and does nothing -- that is intentional. Use it to
confirm your workspace is built and sourced before you write any logic.

Each task is marked with a TASK n.n comment matching the README. Commented-out lines are
deliberate: uncomment and complete them.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64


class RobotController(Node):
    """Drives the robot, tracks its position error, and flags obstacles."""

    def __init__(self):
        super().__init__('robot_controller')

        # Publish to /error when the position delta exceeds this (TASK 2.3).
        # This is a starting value -- justify whatever you settle on.
        self.error_thresh = 0.5

        # ---- TASK 1.2: publisher that drives the robot ---------------------
        # Which topic moves the robot? Find it first (TASK 1.1), then uncomment.
        #
        self.move_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #
        # Then drive it on a timer:
        self.move_timer = self.create_timer(0.1, self.send_move_cmd)

        # ---- TASK 1.3: the path you chose ----------------------------------
        # Pick a route that gets the robot around the wall, and represent it
        # however you think is best -- a list of waypoints, a sequence of
        # timed velocity commands, a parametric curve, something else.
        #
        # Document HERE why you chose this path and this representation.
        # That reasoning is a large part of what we are evaluating.
        # (linear_x m/s, angular_z rad/s, duration s)
        # this is manually set for task 1. I purposefully set a wide berth since there
        # will be drift
        self.path = [
            (0.0, 0.5, 3.14),  # turn ~90 deg left
            (0.5, 0.0, 10),   # drive forward
            (0.0, -0.5, 3.3),  # turn ~90 deg right
            (0.5, 0.0, 15),   # drive past the wall
            (0.0, -0.5, 3.3),  # turn ~90 deg right
            (0.5, 0.0, 10),   # drive behind wall
            (0.0, 0.5, 3.14),  # turn ~90 deg left
        ]
        # Keeps track of which segment we are on 
        self.segment_idx = 0
        # keeps track of how long we were on a certain segment
        self.segment_start = self.get_clock().now()

        # ---- TASK 2.2: subscriber for the robot's 6D pose ------------------
        # One of the two onboard sensors reports 6D data. Find it (TASK 2.1).
        #
        self.robot_pos_sub = self.create_subscription(
            Imu,
            '/imu',
            self.on_robot_pos,
            qos_profile_sensor_data,
        )

        # ---- TASK 2.3: where the measured-vs-actual error goes -------------
        self.error_pub = self.create_publisher(Float64, '/error', 10)
        #
        # Hint: ground truth for "actual" is published by the simulator on the
        # robot's odometry topic (nav_msgs/Odometry). Deciding what to compare,
        # and in which frame, is part of the task.

        # ground truth from the simulator
        self.odom_sub = self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry',
            self.on_odom, qos_profile_sensor_data)

        # latest ground-truth yaw, set by on_odom
        self.truth_yaw = None
        # IMU dead-reckoned yaw, integrated in on_robot_pos
        self.imu_yaw = 0.0
        self.last_stamp = None

        # ---- TASK 3: lidar in, filtered obstacles out ----------------------
        # The lidar has a single vertical sample, so this cloud is one flat
        # row of points at the sensor's height -- not a 3D volume.
        #
        # self.lidar_sub = self.create_subscription(
        #     PointCloud2,
        #     '/lidar/points',
        #     self.on_lidar,
        #     qos_profile_sensor_data,
        # )
        # self.obstacle_pub = self.create_publisher(
        #     PointCloud2, '/obstacle_cloud', 10)

        self.get_logger().info(
            'robot_controller started (scaffold -- nothing wired up yet)')

    # -----------------------------------------------------------------------
    # TASK 1.2 -- publish a velocity command
    # -----------------------------------------------------------------------
    def send_move_cmd(self):
        """Publish one Twist that moves the robot along self.path.

        TODO: build the Twist and publish it on self.move_pub.
        """
        cmd = Twist()
        if self.segment_idx < len(self.path):
            lin, ang, dur = self.path[self.segment_idx]
            elapsed = (self.get_clock().now() - self.segment_start).nanoseconds / 1e9
            if elapsed >= dur:
                self.segment_idx += 1
                self.segment_start = self.get_clock().now()
            else:
                cmd.linear.x = lin
                cmd.angular.z = ang
        # after the last segment, cmd stays all zeros, so the robot stops
        self.move_pub.publish(cmd)

    # -----------------------------------------------------------------------
    # TASK 2.3 -- compare reported position against ground truth
    # -----------------------------------------------------------------------

    def quat_to_yaw(self, q):
        """Extract yaw (rotation about z) from a quaternion."""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
    def on_odom(self, msg):
        # Ground truth from the simulator, in the odom frame.
        self.truth_yaw = self.quat_to_yaw(msg.pose.pose.orientation)

    def on_robot_pos(self, msg):
        """Compare the sensor's idea of where we are against the truth.

        Publish a Float64 on self.error_pub when the delta exceeds
        self.error_thresh.

        TODO: decide what "delta" means here and justify it in a comment.
        """
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            self.last_stamp = stamp
            return
        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 0.0:
            return

        self.imu_yaw += msg.angular_velocity.z * dt

        if self.truth_yaw is None:
            return  # no ground truth yet

        # Wrap into [-pi, pi] so 359 deg vs 1 deg reads as 2 deg, not 358.
        delta = abs(math.atan2(math.sin(self.imu_yaw - self.truth_yaw),
                               math.cos(self.imu_yaw - self.truth_yaw)))

        # if delta > self.error_thresh:
        self.error_pub.publish(Float64(data=delta))
        self.get_logger().info(f'delta={delta:.4f}')

    # -----------------------------------------------------------------------
    # TASK 3.3 -- classify a single lidar point
    # -----------------------------------------------------------------------
    def is_obstacle(self, point):
        """Return True if `point` is something we must avoid.

        The barrier is passable -- treat it like dust in the air. The poles are
        not. `point` is an (x, y, z) tuple in the lidar's frame.

        TODO: decide what separates a pole from the barrier and implement it.
        """
        raise NotImplementedError('TASK 3.3')

    # -----------------------------------------------------------------------
    # TASK 3.2 -- filter the scan and republish what matters
    # -----------------------------------------------------------------------
    def on_lidar(self, msg):
        """Filter incoming points through is_obstacle and republish.

        point_cloud2.read_points(msg, field_names=('x', 'y', 'z')) iterates the
        cloud; point_cloud2.create_cloud_xyz32(msg.header, pts) builds the
        outgoing one.

        TODO: keep only the obstacle points and publish on self.obstacle_pub.
        """
        raise NotImplementedError('TASK 3.2')


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
