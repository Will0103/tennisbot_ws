import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")


    # =========================
    # Launch Arguments
    # =========================

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    # Real robot MUST use system time
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )


    # =========================
    # Hardware Interface
    # =========================

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )


    # =========================
    # LiDAR
    # =========================

    laser_driver = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory("tennisbot_bringup"),
                "config",
                "rplidar_a1.yaml"
            ),
            {
                "use_sim_time": use_sim_time
            }
        ],
    )


    # =========================
    # ros2_control Controllers
    # =========================

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )


    # =========================
    # Joystick
    # =========================

    cmd_vel_control = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_controller"),
            "launch",
            "cmd_vel_control_real.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )


    # =========================
    # Localization
    # =========================

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
        condition=UnlessCondition(use_slam)
    )


    # =========================
    # SLAM
    # =========================

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
        condition=IfCondition(use_slam)
    )


    # =========================
    # Navigation
    # =========================

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )


    # =========================
    # Launch
    # =========================

    return LaunchDescription([
        use_slam_arg,
        use_sim_time_arg,

        hardware_interface,
        controller,
        cmd_vel_control,

        laser_driver,
        # localization,
        slam,
        # navigation,
    ])