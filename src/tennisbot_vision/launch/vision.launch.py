from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_button = LaunchConfiguration("enable_button")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    enable_button_arg = DeclareLaunchArgument(
            "enable_button",
            default_value= "7"
        )
    

    ball_detection_function = Node(
        package="tennisbot_vision",
        executable="ball_detector",
        name="ball_detector",
        parameters=[
            {"enable_button": enable_button},
            {"use_sim_time": use_sim_time}
        ]
    )
    

    return LaunchDescription([
        use_sim_time_arg,
        enable_button_arg,
        ball_detection_function,
    ])