import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # tennisbot_controller_pkg = get_package_share_directory('tennisbot_controller')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    # joy_teleop = Node(
    #     package="joy_teleop",
    #     executable="joy_teleop",
    #     parameters=[os.path.join(get_package_share_directory("tennisbot_controller"), "config", "joy_teleop.yaml"),
    #                 {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    # )

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
        {"use_sim_time": LaunchConfiguration("use_sim_time")}
    ],
    remappings=[
        ("/cmd_vel", "/diffdrive_controller/cmd_vel")
    ]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("tennisbot_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    

    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
        ]
    )