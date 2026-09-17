
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    

    ball_detection_function = Node(
            package="tennisbot_vision",
            executable="ball_detector",
            name="ball_detector",
            parameters=[
                        {"enable_button": 7}
                    ]
        )
    

    return LaunchDescription(
        [
            ball_detection_function,
        ]
    )