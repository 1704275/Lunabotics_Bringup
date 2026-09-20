# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bring up the Gazebo sim testbed: world + bridge + TF + RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_move = get_package_share_directory('move')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_move, 'worlds', 'building_robot.sdf')

    # robot_state_publisher needs a single <model>, not the <world>, so it reads the
    # model file directly. The world <include>s this same file -- one source of truth.
    sdf_file = os.path.join(pkg_move, 'models', 'vehicle_blue', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    controller_launch_arg = DeclareLaunchArgument(
        'controller', default_value='false',
        description='Run the starter node (move/src/publisher.py). Off by default so '
                    'the robot sits still on bringup; enable with controller:=true.'
    )

    # Lets the world resolve <uri>model://vehicle_blue</uri>
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_move, 'models'),
    )

    # -r starts the sim unpaused; without it the clock never advances and the
    # sensors never publish.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Bridge to forward tf, sensors and odometry to ros2
    gz_topic = '/model/vehicle_blue'
    joint_state_gz_topic = '/world/car_world' + gz_topic + '/joint_state'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2) -- robot_state_publisher needs these
            # to animate the wheel transforms
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            gz_topic + '/tf' + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # Lidar (Gazebo -> ROS2)
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Point cloud for TASK 3. The gpu_lidar has one vertical sample,
            # so this is a single flat row of points, not a 3D volume.
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # IMU (Gazebo -> ROS2) -- note the gz type is IMU, not Imu
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            (joint_state_gz_topic, '/joint_states'),
            (gz_topic + '/tf', '/tf'),
        ],
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # RViz conventionally uses `map` as the fixed frame, but nothing in this world
    # publishes one -- the gz odometry tree starts at `odom`. Anchor them together.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Starter node applicants fill in (see README TASKS 1-3).
    controller = Node(
        package='move',
        executable='publisher',
        name='route_pub',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('controller')),
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_move, 'rviz', 'vehicle.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        rviz_launch_arg,
        controller_launch_arg,
        gz_resource_path,
        gazebo,
        bridge,
        static_tf,
        robot_state_publisher,
        controller,
        rviz
    ])
