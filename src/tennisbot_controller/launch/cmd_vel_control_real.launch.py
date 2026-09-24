import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use real time"
    )

    pkg_share = get_package_share_directory("tennisbot_navigation")

    collision_monitor_config = os.path.join(
        pkg_share,
        "config",
        "collision_monitor.yaml"
    )

    ## With Turbo
    joy_teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("tennisbot_controller"),
                "config",
                "teleop_twist_joy.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            # ("/cmd_vel", "/cmd_vel_joy")
            ("/cmd_vel", "/diffdrive_controller/cmd_vel") ##TEMP
        ]
    )

    ## Linux
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joystick",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("tennisbot_controller"),
                "config",
                "joy_config_real.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )

    twist_mux_config = os.path.join(
        get_package_share_directory("tennisbot_controller"),
        "config",
        "twist_mux.yaml"
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_config,
            {"use_stamped": True},
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("cmd_vel_out", "/cmd_vel_muxed")
        ]
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            collision_monitor_config,
            {"use_sim_time": use_sim_time}
        ]
    )

    collision_monitor_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[
            {"autostart": True},
            {"node_names": ["collision_monitor"]},
            {"use_sim_time": use_sim_time}
        ],
    )
    

    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
            # twist_mux,
            # collision_monitor,
            # collision_monitor_lifecycle_manager,
        ]
    )