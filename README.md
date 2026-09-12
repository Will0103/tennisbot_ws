# tennisbot_ws

A ROS 2 differential-drive autonomous mobile robot (AMR), built from scratch as a personal project to gain hands-on experience with the full robotics systems stack — from robot description and control, through sensing, mapping, localization, and navigation.

**Status: in progress.** Everything below is implemented and working in simulation. The next milestone is adding a camera and simple object detection so the robot can autonomously find a tennis ball.

## Overview

Tennisbot is a Gazebo-simulated diff-drive robot equipped with LiDAR and IMU. It can either build a map of an environment from scratch (SLAM) or localize itself within a saved map (AMCL) and navigate autonomously to a goal using the Nav2 stack, with a joystick available at any time to take over manual control.

## Features implemented

- **Robot description**: URDF/Xacro model (`tennisbot_description`), Gazebo (`gz-sim`) plugins for the robot, world, and sensors.
- **Control**: `ros2_control`-based differential drive controller.
- **Teleop**: joystick control via `teleop_twist_joy`, with a deadman switch; a custom `joystick_function` node lets a joystick button cancel an active Nav2 goal.
- **Arbitration**: `twist_mux` arbitrates between joystick commands and Nav2's autonomous commands, so manual input always has priority.
- **Sensors**: LiDAR (`gpu_lidar`) and IMU, bridged from Gazebo into ROS 2 via `ros_gz_bridge`.
- **Mapping**: SLAM via `slam_toolbox` (Ceres-based scan matching with loop closure), with a map saver.
- **Localization**: AMCL, using a saved map.
- **Navigation**: full Nav2 stack — `planner_server`, `smoother_server`, `controller_server` (Regulated Pure Pursuit), `bt_navigator` with a custom behavior tree, and `collision_monitor` for safety.
- **Environments**: includes `small_warehouse` (with a saved map) and `small_house` simulation worlds.

## Not yet implemented

- Camera integration and object detection (the goal: detecting a tennis ball) — next planned step, to close the loop from sensing → localizing → planning → acting on a target object.
- Testing on physical hardware (currently simulation-only, in Gazebo).

## Package structure

```
src/
├── tennisbot_bringup       # Top-level launch files that bring everything up together
├── tennisbot_controller    # ros2_control diff-drive controller, joystick teleop, twist_mux
├── tennisbot_description   # URDF/Xacro robot model, Gazebo worlds/plugins, RViz configs
├── tennisbot_localization  # AMCL localization
├── tennisbot_mapping       # SLAM (slam_toolbox), saved maps
└── tennisbot_navigation    # Nav2 stack: planner, smoother, controller, bt_navigator, collision monitor
```

## Requirements

- ROS 2 Jazzy
- Gazebo (`gz-sim`) + `ros_gz_bridge` / `ros_gz_sim`
- Nav2 (`nav2_bringup` and related packages)
- `slam_toolbox`
- `teleop_twist_joy`, `twist_mux`
- A joystick/gamepad (for teleop and Nav2 goal cancellation)

## Build

```bash
cd tennisbot_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

**Autonomous navigation** (loads the saved `small_warehouse` map, runs AMCL + full Nav2 stack):

```bash
ros2 launch tennisbot_bringup robot_simulation.launch.py
```

**SLAM mapping** (build a new map from scratch instead of localizing on a saved one):

```bash
ros2 launch tennisbot_bringup robot_simulation.launch.py use_slam:=true
```

Both bring up Gazebo, the diff-drive controller, and joystick teleop automatically. RViz opens with a view matched to the mode (Nav2 path/costmap view, or SLAM view).

To switch worlds or use a different saved map, pass `world_name` (e.g. `small_house`) to `tennisbot_description`'s `gazebo.launch.py`, or `map_name` to `tennisbot_localization`'s `global_localization.launch.py`.

## Roadmap

- [ ] Add a camera and integrate simple object detection to find a tennis ball
- [ ] Move from simulation to a physical robot platform
- [ ] Expand test coverage in `small_house` (the target environment for home navigation)

## Author

Will Hsu — [github.com/Will0103](https://github.com/Will0103)
