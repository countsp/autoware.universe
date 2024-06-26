# autoware.universe

For Autoware's general documentation, see [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/).

For detailed documents of Autoware Universe components, see [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware.universe/).

---
### Source installation

### Prerequisites

Ubuntu 20.04

ROS 2 Galactic
   

CUDA

cuDNN

TensorRT

```
sudo apt-get -y update
sudo apt-get -y install git
```

### How to set up a development environment

Clone autowarefoundation/autoware and move to the directory.

```
git clone https://github.com/autowarefoundation/autoware.git -b galactic
cd autoware
```
or download zip file using galactic branch 



---

### installation
reference: [https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/)
-> Installing dependencies manually

##### Install ROS 2
using fishros

##### Install the RMW Implementation
```wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# For details: https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html
sudo apt update
rmw_implementation_dashed=$(eval sed -e "s/_/-/g" <<< "${rmw_implementation}")
sudo apt install ros-${rosdistro}-${rmw_implementation_dashed}

# (Optional) You set the default RMW implementation in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc
```
##### install pacmod
```
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env
# Taken from https://github.com/astuff/pacmod3#installation
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-galactic-pacmod3
```
##### Install Autoware Core dependencies
```
pip3 install gdown
```
##### Install Autoware Universe dependencies
```
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm2008-1
```
##### Install pre-commit dependencies
```
pre_commit_clang_format_version=17.0.6
pip3 install pre-commit clang-format==${pre_commit_clang_format_version}
sudo apt install golang
```
##### Install Nvidia CUDA cuDNN TensorRT
refer to [https://github.com/countsp/ubuntu_settings/blob/main/nvidia.md](https://github.com/countsp/ubuntu_settings/blob/main/nvidia.md)

##### 报错
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO 失败

则 sudo apt-get install ros-galactic-${missing-package}

例如：

sudo apt-get install ros-galactic-osrf-testing-tools-cpp

（osrf_testing-tools-cpp报错不影响使用）

```
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
lanelet2_map_preprocessor: Cannot locate rosdep definition for [pcl_ros]
radar_tracks_msgs_converter: Cannot locate rosdep definition for [radar_msgs]
tier4_localization_launch: Cannot locate rosdep definition for [topic_tools]
vehicle_cmd_gate: Cannot locate rosdep definition for [diagnostic_updater]
radar_threshold_filter: Cannot locate rosdep definition for [radar_msgs]
autoware_testing: Cannot locate rosdep definition for [ros_testing]
behavior_path_planner: Cannot locate rosdep definition for [behaviortree_cpp_v3]
awsim_sensor_kit_launch: Cannot locate rosdep definition for [usb_cam]
grid_map_ros: Cannot locate rosdep definition for [nav2_msgs]
occupancy_grid_map_outlier_filter: Cannot locate rosdep definition for [pcl_ros]
obstacle_velocity_limiter: Cannot locate rosdep definition for [pcl_ros]
lidar_apollo_segmentation_tvm: Cannot locate rosdep definition for [tvm_vendor]
tier4_vehicle_launch: Cannot locate rosdep definition for [xacro]
autoware_point_types: Cannot locate rosdep definition for [point_cloud_msg_wrapper]
dummy_diag_publisher: Cannot locate rosdep definition for [diagnostic_updater]
lanelet2_extension: Cannot locate rosdep definition for [lanelet2_validation]
tier4_autoware_api_launch: Cannot locate rosdep definition for [topic_tools]
scenario_selector: Cannot locate rosdep definition for [topic_tools]
fault_injection: Cannot locate rosdep definition for [diagnostic_updater]
grid_map_octomap: Cannot locate rosdep definition for [octomap]
costmap_generator: Cannot locate rosdep definition for [pcl_ros]
image_diagnostics: Cannot locate rosdep definition for [diagnostic_updater]
obstacle_stop_planner: Cannot locate rosdep definition for [pcl_ros]
grid_map_costmap_2d: Cannot locate rosdep definition for [nav2_costmap_2d]
image_projection_based_fusion: Cannot locate rosdep definition for [pcl_ros]
lidar_centerpoint_tvm: Cannot locate rosdep definition for [tvm_vendor]
grid_map_filters: Cannot locate rosdep definition for [filters]
tensorrt_yolox: Cannot locate rosdep definition for [tensorrt_cmake_module]
tensorrt_common: Cannot locate rosdep definition for [tensorrt_cmake_module]
map_loader: Cannot locate rosdep definition for [pcl_ros]
external_velocity_limit_selector: Cannot locate rosdep definition for [topic_tools]
grid_map_cv: Cannot locate rosdep definition for [filters]
external_cmd_selector: Cannot locate rosdep definition for [diagnostic_updater]
osqp_interface: Cannot locate rosdep definition for [osqp_vendor]
autoware_auto_geometry: Cannot locate rosdep definition for [osrf_testing_tools_cpp]
trtexec_vendor: Cannot locate rosdep definition for [tensorrt_cmake_module]
accel_brake_map_calibrator: Cannot locate rosdep definition for [diagnostic_updater]
velodyne_pointcloud: Cannot locate rosdep definition for [pcl_ros]
steer_offset_estimator: Cannot locate rosdep definition for [diagnostic_updater]
grid_map_demos: Cannot locate rosdep definition for [octomap_msgs]
front_vehicle_velocity_estimator: Cannot locate rosdep definition for [pcl_ros]
awapi_awiv_adapter: Cannot locate rosdep definition for [topic_tools]
system_monitor: Cannot locate rosdep definition for [diagnostic_updater]
velodyne_description: Cannot locate rosdep definition for [xacro]
sample_vehicle_description: Cannot locate rosdep definition for [xacro]
lane_departure_checker: Cannot locate rosdep definition for [diagnostic_updater]
localization_error_monitor: Cannot locate rosdep definition for [diagnostic_updater]
tvm_utility: Cannot locate rosdep definition for [tvm_vendor]
ground_segmentation: Cannot locate rosdep definition for [pcl_ros]
velodyne_monitor: Cannot locate rosdep definition for [diagnostic_updater]
radar_static_pointcloud_filter: Cannot locate rosdep definition for [radar_msgs]
obstacle_collision_checker: Cannot locate rosdep definition for [pcl_ros]
web_controller: Cannot locate rosdep definition for [rosbridge_server]
pacmod_interface: Cannot locate rosdep definition for [diagnostic_updater]
tier4_api_msgs: Cannot locate rosdep definition for [geographic_msgs]
external_cmd_converter: Cannot locate rosdep definition for [diagnostic_updater]
sample_sensor_kit_launch: Cannot locate rosdep definition for [usb_cam]
lidar_apollo_segmentation_tvm_nodes: Cannot locate rosdep definition for [ros_testing]
bluetooth_monitor: Cannot locate rosdep definition for [diagnostic_updater]
pointcloud_preprocessor: Cannot locate rosdep definition for [point_cloud_msg_wrapper]
trajectory_follower: Cannot locate rosdep definition for [diagnostic_updater]
lidar_centerpoint: Cannot locate rosdep definition for [pcl_ros]
system_error_monitor: Cannot locate rosdep definition for [rqt_robot_monitor]
velodyne_driver: Cannot locate rosdep definition for [diagnostic_updater]
topic_state_monitor: Cannot locate rosdep definition for [diagnostic_updater]
probabilistic_occupancy_grid_map: Cannot locate rosdep definition for [pcl_ros]
planning_error_monitor: Cannot locate rosdep definition for [diagnostic_updater]

```

##### How to set up a workspace

Create the src directory and clone repositories into it.

Autoware uses vcstool to construct workspaces.

```
cd autoware
mkdir src
vcs import src < autoware.repos
```


Install vcs command
```
sudo apt install python3-vcstool
```

Install dependent ROS packages.

Autoware requires some ROS 2 packages in addition to the core components. The tool rosdep allows an automatic search and installation of such dependencies. You might need to run rosdep update before rosdep install.
```
source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
##### Build the workspace.

Autoware uses colcon to build workspaces. For more advanced options, refer to the documentation.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
    
