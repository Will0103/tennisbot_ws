from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # DiffDriveController = simple_controller + simple_velocity_controller
    wheel_controller_spawner =  Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
                "diffdrive_controller", 
                "--controller-manager", 
                "/controller_manager"
        ]
    )


    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            wheel_controller_spawner
        ]
    )