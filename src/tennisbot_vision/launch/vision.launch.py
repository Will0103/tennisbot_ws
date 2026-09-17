from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    ball_detection_function = Node(
        package="tennisbot_vision",
        executable="ball_detector",
        name="ball_detector",
        parameters=[
            {"enable_button": 7},
            {"use_sim_time": use_sim_time}
        ]
    )
    

    return LaunchDescription([
        use_sim_time_arg,
        ball_detection_function,
    ])