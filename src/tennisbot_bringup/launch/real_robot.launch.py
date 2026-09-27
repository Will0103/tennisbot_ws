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

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )


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

    ekf = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_localization"),
            "launch",
            "ekf.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )


    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_name": "will_home"
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

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tennisbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    patrol_nodes = Node(
        package="tennisbot_navigation",
        executable="patrol_nodes",
        name="patrol_nodes",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"cancel_button": 1},
            {"start_button": 3},
            {"waypoints_x": [-1.0, -3.1, -4.0, -5.0, -5.0, -4.2, -2.6, -1.4, 0.0]},
            {"waypoints_y": [1.3, 1.5, 1.5, 1.0, 0.0, -0.9, -0.9, -4.4, 0.0]},
        ],
        condition=UnlessCondition(use_slam)
    )

    return LaunchDescription([
        use_slam_arg,
        use_sim_time_arg,

        hardware_interface,
        controller,
        cmd_vel_control,

        ekf,

        laser_driver,
        localization,
        slam,
        navigation,

        patrol_nodes,
    ])