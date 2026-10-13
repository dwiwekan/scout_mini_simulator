#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Get the urdf file
    sdf_path = os.path.join(
        get_package_share_directory('scout_gazebo_sim'),
        'models',
        'scout_mini',
        'scout_mini.sdf'
    )

    # Launch configuration variables specific to simulation
    namespace = LaunchConfiguration('namespace', default='scout_mini')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.245')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf', default='false')

    # Declare the launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=namespace,
        description='Specify robot namespace')

    declare_x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value=x_pose,
        description='Specify robot x position')

    declare_y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value=y_pose,
        description='Specify robot y position')

    declare_z_pose_arg = DeclareLaunchArgument(
        'z_pose', default_value=z_pose,
        description='Specify robot z position (raise it for heightmap worlds)')

    declare_yaw_pose_arg = DeclareLaunchArgument(
        'yaw_pose', default_value=yaw_pose,
        description='Specify robot yaw angle')

    declare_publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf', default_value=publish_odom_tf,
        description='Publish odom -> base_link TF from Gazebo (keep false when the EKF is running)')

    # Add namespace to gazebo model file
    namespaced_sdf_file = ReplaceString(
        source_file=os.path.join(get_package_share_directory('scout_gazebo_sim'),
        'models',
        'scout_mini',
        'scout_mini.sdf'), 
        replacements={'/robot_namespace': ('/', namespace),
                      '<publish_odom_tf>false</publish_odom_tf>':
                          ('<publish_odom_tf>', publish_odom_tf, '</publish_odom_tf>')})

    # Nodes
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', (namespace, '_robot'),
            '-file', namespaced_sdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen',
    )

    # slam_toolbox needs a 2D LaserScan: flatten the Livox cloud to /<ns>/scan.
    # Heights are in livox_frame (0.34 m above the ground): keeps 0.2 - 1.0 m above the
    # ground, so the scan sees walls/boxes/pillars like the old 2D lidar but not the floor.
    pointcloud_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        namespace=namespace,
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.05,
            'min_height': -0.14,
            'max_height': 0.66,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.01745,  # 1 deg, the simulated Livox horizontal step
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 40.0,
            'use_inf': True,
        }],
        remappings=[('cloud_in', 'livox/lidar'), ('scan', 'scan'),
                    ('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_x_pose_arg)
    ld.add_action(declare_y_pose_arg)
    ld.add_action(declare_z_pose_arg)
    ld.add_action(declare_yaw_pose_arg)
    ld.add_action(declare_publish_odom_tf_arg)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(pointcloud_to_laserscan_cmd)

    return ld

