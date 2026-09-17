import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )
    
    cmd_vel_control = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_controller"),
            "launch",
            "cmd_vel_control.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    delayed_navigation = TimerAction(
        period=2.0,
        actions=[navigation]
    )

    vision_function = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_vision"),
            "launch",
            "vision.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )


    rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(
                get_package_share_directory("tennisbot_navigation"),
                "rviz", "Path_nav2_camera.rviz")],
            condition=UnlessCondition(use_slam)
        )


    ### SLAM ###
    slam = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("tennisbot_mapping"),
                "launch",
                "slam.launch.py"
            ),
            condition=IfCondition(use_slam)
        )
    rviz_slam = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(
                    get_package_share_directory("tennisbot_mapping"),
                    "rviz", "slam_camera.rviz")],
                condition=IfCondition(use_slam)
            )



    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        cmd_vel_control,
        delayed_navigation,
        rviz,
        localization,
        slam,
        rviz_slam,
        vision_function
    ])