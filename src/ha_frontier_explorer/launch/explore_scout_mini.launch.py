"""SLAM + Nav2 + frontier explorer for the Scout Mini.

Start Gazebo first with publish_odom_tf:=false (the EKF in nav2.launch.py publishes odom->base_link):
  ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py publish_odom_tf:=false
then:
  ros2 launch ha_frontier_explorer explore_scout_mini.launch.py
Without the EKF: publish_odom_tf:=true in Gazebo, and use_ekf:=false here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_launch_dir = os.path.join(get_package_share_directory('scout_navigation'), 'launch')
    explorer_launch_dir = os.path.join(get_package_share_directory('ha_frontier_explorer'), 'launch')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    explorer_delay = LaunchConfiguration('explorer_delay')
    use_ekf = LaunchConfiguration('use_ekf')

    # slam_toolbox: /<ns>/map and map->odom
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'slam_online_async.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items(),
    )

    # Nav2 without map_server/amcl, EKF publishes odom->base_link
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'nav2.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
            'slam': 'true',
            'use_ekf': use_ekf,
        }.items(),
    )

    # controller_server publishes cmd_vel_nav, the Gazebo diff drive listens on cmd_vel
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        namespace=namespace,
        arguments=['cmd_vel_nav', 'cmd_vel'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # give slam_toolbox time to publish a first map and Nav2 time to activate
    explorer = TimerAction(
        period=explorer_delay,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(explorer_launch_dir, 'explorer.launch.py')),
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items(),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='scout_mini'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='false: use Gazebo odom TF (spawn with publish_odom_tf:=true)'),
        DeclareLaunchArgument('explorer_delay', default_value='10.0',
                              description='Seconds to wait before starting the explorer'),
        slam,
        nav2,
        cmd_vel_relay,
        explorer,
    ])
