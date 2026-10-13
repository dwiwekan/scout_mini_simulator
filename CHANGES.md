# Changes from upstream

This repo builds on two upstream projects. This file lists what we changed in each, so you can port the changes to another version yourself.

## ugv_gazebo_sim (overlay/)

Base: [ROSETEA-lab/ugv_gazebo_sim](https://github.com/ROSETEA-lab/ugv_gazebo_sim) at commit `3b9de23` (2024-07-30).

`setup.sh` clones that commit and copies `overlay/ugv_gazebo_sim/` over it. The overlay keeps the upstream file paths. To see one change, diff the overlay file against upstream:

```bash
./setup.sh
cd src/ugv_gazebo_sim && git diff          # after a normal (copy) setup
```

### Changed files

| File | Change |
| -- | -- |
| `scout/scout_gazebo_sim/models/scout_mini/scout_mini.sdf` | RPLIDAR replaced by a Livox Mid-360 (`gazebo_ros_ray_sensor`, PointCloud2, 360 × 16 rays, −7° to +52°) and a ZED 2i (`gazebo_ros_camera` depth camera, zed-ros2-wrapper topic names). Body collision is now a box, and the wheels are cylinders with mu 0.4 along the axle and mu2 1, so skid steering still works. The mesh collisions ran at 0.2× real time. GPS collisions removed, because the Livox rays hit them. |
| `scout/scout_description/urdf/scout_mini_extras.urdf` | `livox_frame` at (0.15, 0, 0.16). ZED frames `zed_camera_link`, `zed_left_camera_optical_frame`, `zed_imu_link` and the others, at (0.27, 0, 0.084). The `base_scan` frame is gone. |
| `scout/scout_gazebo_sim/launch/spawn_scout_mini.launch.py` | `z_pose` and `publish_odom_tf` arguments. Adds a `pointcloud_to_laserscan` node: `livox/lidar` → `scan`. |
| `scout/scout_gazebo_sim/launch/scout_mini_empty_world.launch.py` | Passes `z_pose` and `publish_odom_tf`. The RViz fixed frame follows `publish_odom_tf`. Fixes `GAZEBO_MODEL_PATH`. Sets RViz `use_sim_time`. |
| `scout/scout_gazebo_sim/launch/scout_v2_empty_world.launch.py`, `multi_scout_*_empty_world.launch.py`, `bunker/.../bunker_empty_world.launch.py` | Fix for `GAZEBO_MODEL_PATH`: when the variable was already set, upstream appended a relative path (`':' + models`). Also RViz `use_sim_time`. |
| `scout/scout_gazebo_sim/package.xml` | `exec_depend` on `pointcloud_to_laserscan`. |
| `scout/scout_gazebo_sim/worlds/empty.world` | Replaced by a generated world: a 20 × 20 m obstacle arena, then the park through a gate in the east wall. |

### New files

| File | What it is |
| -- | -- |
| `scout/scout_gazebo_sim/tools/gen_empty_world.py` | Generates `empty.world` and `park_terrain.stl` |
| `scout/scout_gazebo_sim/models/park_terrain/` | Park terrain mesh: sunken plaza and hills |
| `scout/scout_gazebo_sim/worlds/rough_terrain_40m.world`, `models/rough_terrain/` | 40 × 40 m rough terrain. The collision is an STL, because ODE lidar rays against a heightmap are too slow. |

`setup.sh` also adds `COLCON_IGNORE` to these ROS 1 only folders: `autokit hunter hunter_se limo limo_s2 ranger_mini_V2 tracer tracer_mini scout/scout_control`.

## scout_navigation (src/scout_navigation)

Base: [mattiadutto/scout_navigation](https://github.com/mattiadutto/scout_navigation) at commit `d1e6c4a`, Apache 2.0. The package is copied in here with our edits applied.

| File | Change |
| -- | -- |
| `src/cloud_height_filter.cpp`, `CMakeLists.txt`, `package.xml` | New node. Converts the 3D clouds to `base_link` and keeps only points at obstacle height. Outputs `livox/obstacles` and `zed/obstacles`. |
| `config/nav2_params.yaml` | The local and global costmaps use a `VoxelLayer` fed by the two obstacle clouds. |
| `launch/nav2.launch.py` | Starts the two filter nodes. New arguments: `slam:=true` skips map_server and AMCL, and `use_ekf:=false` skips the EKF. |
| `launch/slam_online_async.launch.py` | New arguments `namespace` and `use_rviz`. Params go through `RewrittenYaml(root_key=namespace)`. Without that, slam_toolbox under `/scout_mini` ignored its yaml and used the default frames. |
| `launch/rtabmap.launch.py` | `frame_id: base_link` |
| `rviz/scout_mini_navigation.rviz` | Livox and ZED point cloud displays |

## ha_frontier_explorer (src/ha_frontier_explorer)

Frontier-based exploration node, adapted from a TurtleBot3 + Cartographer version for Scout Mini + slam_toolbox + Nav2:
- `yolo_msgs` is optional.
- Goal timeout and blacklist.
- Each goal is the frontier cell nearest to the frontier's centroid.
- `explore_scout_mini.launch.py` starts slam_toolbox, Nav2, a `cmd_vel_nav` → `cmd_vel` relay and the explorer.
