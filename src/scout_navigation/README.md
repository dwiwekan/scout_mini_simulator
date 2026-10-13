This package is a package for ROS2 Humble with the target of implementing Navigation stack on the Agilex Robots, in particular to Scout Mini, Bunker and Bunker Pro.
Currently the navigation stack in the base configuration is working on the Agilex Scout Mini. The simulation is expected to be run on the Agilex Scout Mini.

The navigation stack is publish command velocity on **cmd_vel_nav** so you will need to use a *twist_mux* for remapping this to cmd_vel topic. 

## Usage

### 1. Download
```bash
mkdir -p <ros2_workspace>/src
cd <ros2_workspace>/src

git clone https://github.com/mattiadutto/scout_navigation.git
```

For using the **simulation** you also need the following packages:
```bash
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone https://github.com/ROSETEA-lab/ugv_gazebo_sim
git clone -b humble https://github.com/ROSETEA-lab/scout_ros2 
```
For using the **real robot** you need the following packages instead:
```bash
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone -b humble https://github.com/ROSETEA-lab/scout_ros2 
```

### 2. Build
```bash
cd ..
colcon build

. install/setup.bash
```

### 3. NAVIGATION: nav2.launch.py
This are usage example, the used file name or parameter values can be different in your case.

#### Usage
```bash
# Simulation case of nav2.launch.py
ros2 launch scout_navigation nav2.launch.py \
      namespace:=scout_mini \
      map_name:=workshop_big_empty_slam.yaml \
      rviz_params_file:=scout_mini_navigation.rviz

# Real robot of nav2.lauch.py
ros2 launch scout_navigation nav2.launch.py use_sim_time:=False map_name:=velodyne_andata_5_destra.yaml nav2_params_file:=nav2_params_scout_mini.yaml rviz_params_file:=scout_mini_robot.rviz 
```

#### Parameters 
| Parameter name | Default value | Description |
| -- | -- | -- |
| `use_rviz` | True | Usage of RViz2 for telemetry operation | 
| `use_sim_tim` | True | Usage of simulation time or not |

### 4. NAVIGATION: gps.launch.py
TODO: still to test in an extensive way the fusion of data between: GPS / IMU / Robot Odometry.

#### Parameters
| Parameter name | Default value | Description |
| -- | -- | -- |
| `ekf_params_file` | ekf_localization_with_gps.yaml | Configuration file for the Extended Kalman Filter, it must be present in the config folder of this package |
| `map_name` | slam_farm.yaml | Name of the map that will be loaded into Nav2 stack, it must be present into the maps folder |
| `namespace` | "" | Namespace of the robot |
| `nav2_params_file` | nav2_params.yaml | Configuration file for Navigation 2 stack, it must be present in the config folder of this package | 
| `rviz_params_file` | scout_mini_navigation.yaml | Configuration file for RViz2, it must be present in the configuration folder of this package | 
| `use_rviz` | True | Usage of RViz2 for telemetry operation | 
| `use_sim_tim` | True | Usage of simulation time or not |

### 5. MAPPING: slam_offline.launch.py
This launch file can allow you to map an environment that can be later utilized for the navigation stack. This allow us to create a map starting from **pre-recorded** data.

#### Usage
```bash
# Real robot
ros2 launch scout_navigation slam_offline.launch.py
```

#### Parameters
| Parameter name | Default value | Description |
| -- | -- | -- |
| `autostart` | True | Automatically startup the SLAM Toolbox, ignored when use_lifecycle_manager is set to True |
| `namespace` | "" | Namespace of the robot |
| `slam_params_file` | mapper_params_offline.yaml | Configuration file for the SLAM Toolbox, it must be present in the config folder of this package |
| `use_lifecycle_manager` | False | Enable bond connection during ROS2 node activation |

### 6. MAPPING: slam_online_async.launch.py
This launch file can allow you to map an environment that can be later utilized for the navigation stack. This allow us to create a map from **live** data.

#### Usage
```bash
# Real robot
ros2 launch scout_navigation slam_online_async.launch.py use_sim_time:=False
```

#### Parameters
| Parameter name | Default value | Description |
| -- | -- | -- |
| `namespace` | "" | Namespace of the robot |
| `scan` | scan | 2D laser scan topic remapped on the robot (without robot namespace) |
| `slam_params_file` | mapper_params_online_async.yaml | Configuration file for the SLAM Toolbox, it must be present in the config folder of this package |
| `tf` | tf | Tf remapping of the robot (without robot namespace) | 
| `tf_static` | tf_static | Tf static remapping of the robot (without robot namespace) |
| `use_sim_tim` | True | Usage of simulation time or not |