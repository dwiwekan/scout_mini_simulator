from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_name = "scout_navigation"

    namespace = LaunchConfiguration("namespace", default="")
    scan = LaunchConfiguration("scan", default="scan")
    slam_params_file = LaunchConfiguration(
        "slam_params_file", default="mapper_params_online_async.yaml"
    )
    tf = LaunchConfiguration("tf", default="tf")
    tf_static = LaunchConfiguration("tf_static", default="tf_static")
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    use_rviz = LaunchConfiguration("use_rviz", default="false")
    rviz_params_file = LaunchConfiguration(
        "rviz_params_file", default="scout_mini_navigation.rviz"
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=namespace,
        description="Specify namespace of the robot",
    )

    declare_scan_arg = DeclareLaunchArgument(
        "scan",
        default_value=scan,
        description="Specify scan remapping of the robot",
    )

    slam_params_file_path = PathJoinSubstitution(
        [get_package_share_directory(pkg_name), "config", slam_params_file]
    )

    declare_slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_file_path,
        description="Name of the yaml parameters file of slam_toolbox node in mapping mode",
    )

    declare_tf_arg = DeclareLaunchArgument(
        "tf",
        default_value=tf,
        description="Specify tf remapping of the robot",
    )

    declare_tf_static_arg = DeclareLaunchArgument(
        "tf_static",
        default_value=tf_static,
        description="Specify tf static remapping of the robot",
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )

    declare_user_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value=use_rviz, description="Run rviz if true"
    )

    declare_rviz_params_file = DeclareLaunchArgument(
        "rviz_params_file",
        default_value=rviz_params_file,
        description="Specify the rviz 2 configuration file",
    )

    # Nest the parameters under the namespace so they match /<namespace>/slam_toolbox
    namespaced_slam_params = RewrittenYaml(
        source_file=slam_params_file_path,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    async_slam_toolbox_node = Node(
        namespace=namespace,
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[namespaced_slam_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/scan", scan),
            ("/tf", tf),
            ("/tf_static", tf_static),
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
        ],
    )

    rviz2_node = Node(
        namespace=namespace,
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [get_package_share_directory(pkg_name), "rviz", rviz_params_file]
            ),
        ],
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        remappings=[
            ("/tf", tf),
            ("/tf_static", tf_static),
            ("/goal_pose", "goal_pose"),
            ("/clicked_point", "clicked_point"),
            ("/initialpose", "initialpose"),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_arg)

    ld.add_action(declare_scan_arg)
    ld.add_action(declare_slam_params_file_arg)
    ld.add_action(declare_tf_arg)
    ld.add_action(declare_tf_static_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_user_rviz_arg)
    ld.add_action(declare_rviz_params_file)

    ld.add_action(async_slam_toolbox_node)
    ld.add_action(rviz2_node)

    return ld
