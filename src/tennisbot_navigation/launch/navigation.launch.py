import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "waypoint_follower"]
    tennisbot_navigation_pkg = get_package_share_directory("tennisbot_navigation")

    # Absolute path
    navigate_to_pose_bt = os.path.join(
        tennisbot_navigation_pkg,
        "behavior_tree",
        "smooth_navigation.xml"
    )
    navigate_through_poses_bt = os.path.join(
        tennisbot_navigation_pkg,
        "behavior_tree",
        "smooth_navigation.xml"
    )


    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            os.path.join(
                tennisbot_navigation_pkg,
                "config",
                "controller_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
        ("cmd_vel", "/cmd_vel_nav")
        ]
    )
    
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(
                tennisbot_navigation_pkg,
                "config",
                "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )
    
    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(
                tennisbot_navigation_pkg,
                "config",
                "bt_navigator.yaml"
            ),
            {
                "use_sim_time": use_sim_time,
                "default_nav_to_pose_bt_xml": navigate_to_pose_bt,
                "default_nav_through_poses_bt_xml": navigate_through_poses_bt,
            }
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(
                tennisbot_navigation_pkg,
                "config",
                "smoother_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("tennisbot_navigation"),
                "config",
                "waypoint_follower.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )
    

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    patrol_nodes = Node(
                package="tennisbot_navigation",
                executable="patrol_nodes",
                name="patrol_nodes",
                parameters=[
                            {"cancel_button": 2},
                            {"start_button": 3},
                            {"waypoints_x": [1.0, 1.0, -3.5, -3.5]},
                            {"waypoints_y": [0.0, 4.0, 4.0, 0.0]},
                        ],
            )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
        waypoint_follower,
        patrol_nodes,
    ])