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
        ("/cmd_vel", "/cmd_vel_joy")
    ]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("tennisbot_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
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
            {"use_stamped": True}
        ],
        remappings=[
            ("cmd_vel_out", "/cmd_vel_muxed") # For collision monitor
        ]
    )

    # joystick_function = Node(
    #         package="tennisbot_controller",
    #         executable="joystick_function",
    #         name="joystick_function",
    #         parameters=[
    #                     {"cancel_button": 2}
    #                 ]
    #     )
    

    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
            twist_mux,
            # joystick_function,
        ]
    )