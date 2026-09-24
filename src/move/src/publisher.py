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
        self.error_thresh = 0.7

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
        # Instead of timing each segment and giving a fixed acceleration, I am using waypoint navigation
        # this reduces the error caused by manually timing unalignment. I'm going right because
        # it is a wider curve that is easier to navigate. I pulled these coordinate points from 
        # the sdf file directly.
        self.path = [(3.0, -4.0), (8.0, -4.0), (8.0, 0.0)]
        self.wp_idx = 0
        self.pose = None            # (x, y, yaw), set by on_odom
        self.pos_tol = 0.15         # m, "close enough" to a waypoint

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
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.on_lidar,
            qos_profile_sensor_data,
        )
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/obstacle_cloud', 10)

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
        if self.pose is None or self.wp_idx >= len(self.path):
            self.move_pub.publish(cmd)   # no pose yet, or done: stop
            return

        x, y, yaw = self.pose
        gx, gy = self.path[self.wp_idx]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)

        if dist < self.pos_tol:
            self.wp_idx += 1
            self.move_pub.publish(cmd)
            return

        # Heading error, wrapped to [-pi, pi].
        heading = math.atan2(dy, dx)
        err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))

        # Turn in place until roughly pointed at the goal, then drive.
        # Proportional control: command scales with error
        if abs(err) > 0.2:
            cmd.angular.z = max(-0.5, min(0.5, 1.5 * err))
        else:
            cmd.linear.x = min(0.5, 0.8 * dist)
            cmd.angular.z = max(-0.5, min(0.5, 1.5 * err))
        self.move_pub.publish(cmd)

    # -----------------------------------------------------------------------
    # TASK 2.3 -- compare reported position against ground truth
    # -----------------------------------------------------------------------

    def quat_to_yaw(self, q):
        """Extract yaw (rotation about z) from a quaternion."""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
    
    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        self.pose = (p.x, p.y, yaw)
        self.truth_yaw = yaw

    def on_robot_pos(self, msg):
        """Compare the sensor's idea of where we are against the truth.

        Publish a Float64 on self.error_pub when the delta exceeds
        self.error_thresh.

        Since the sampling is done once every second, there is a base error that can be
        screened out by delta = 0.7
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

        if delta > self.error_thresh:
            self.error_pub.publish(Float64(data=delta))
        # self.get_logger().info(f'delta={delta:.4f}')

    # -----------------------------------------------------------------------
    # TASK 3.3 -- classify a single lidar point
    # -----------------------------------------------------------------------
    def is_obstacle(self, point):
        """Return True if `point` is something we must avoid.

        The barrier is passable -- treat it like dust in the air. The poles are
        not. `point` is an (x, y, z) tuple in the lidar's frame.

        TODO: decide what separates a pole from the barrier and implement it.
        """
        # The wall is retro 0 ("dust"), the pillar retro 2000. Both span the
        # lidar's height, so z cannot separate them. Intensity is the only
        # thing that does.
        if not all(math.isfinite(point[f]) for f in ('x', 'y', 'z')):
            return False
        return point['intensity'] > 100.0

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
        vals = {round(p['intensity']) for p in point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z', 'intensity'))}
        # self.get_logger().info(f'intensities: {sorted(vals)}', throttle_duration_sec=2.0)
        pts = [(p['x'], p['y'], p['z'])
               for p in point_cloud2.read_points(
                   msg, field_names=('x', 'y', 'z', 'intensity'))
               if self.is_obstacle(p)]
        self.obstacle_pub.publish(
            point_cloud2.create_cloud_xyz32(msg.header, pts))


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
