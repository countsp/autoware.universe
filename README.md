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
```
wget http://fishros.com/install -O fishros && . fishros
```
##### Install the RMW Implementation
```
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

sudo apt update
sudo apt install ros-galactic-rmw-cyclonedds-cpp


# (Optional) You set the default RMW implementation in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```
##### install pacmod
ubuntu 20.04
```
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env
# Taken from https://github.com/astuff/pacmod3#installation
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-galactic-pacmod3
```
ubuntu 22.04
``` 
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# Taken from https://github.com/astuff/pacmod3#installation
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-${rosdistro}-pacmod3
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
pip3 install pre-commit clang-format==17.0.6
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
```
sudo apt-get install -y ros-galactic-pcl-ros \
ros-galactic-radar-msgs \
ros-galactic-topic-tools \
ros-galactic-diagnostic-updater \
ros-galactic-ros-testing \
ros-galactic-behaviortree-cpp-v3 \
ros-galactic-usb-cam \
ros-galactic-nav2-msgs \
ros-galactic-lanelet2-validation \
ros-galactic-octomap \
ros-galactic-tvm-vendor \
ros-galactic-xacro \
ros-galactic-point-cloud-msg-wrapper \
ros-galactic-nav2-costmap-2d \
ros-galactic-filters \
ros-galactic-tensorrt-cmake-module \
ros-galactic-osqp-vendor \
ros-galactic-osrf-testing-tools-cpp \
ros-galactic-octomap-msgs \
ros-galactic-rosbridge-server \
ros-galactic-geographic-msgs \
ros-galactic-rqt-robot-monitor \
ros-galactic-lanelet2-maps \
ros-galactic-diagnostic-aggregator \
ros-galactic-rqt-runtime-monitor \
ros-galactic-cudnn-cmake-module \
ros-galactic-ament-clang-format \
librange-v3-dev \
libpcl-dev \
libfmt-dev \
libcpprest-dev \
libpcap-dev \
libcgal-dev \
nlohmann-json3-dev \
ros-galactic-ublox-gps \
ros-galactic-can-msgs \
ros-galactic-pacmod3-msgs \
libopenni-dev \
libopenni2-dev \
libpcap-dev \
libpng-dev \
libusb-1.0-0-dev \
libyaml-cpp-dev \
pkg-config \
libnl-genl-3-dev



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

sudo rosdep init
rosdep update

source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
##### Build the workspace.

Autoware uses colcon to build workspaces. For more advanced options, refer to the documentation.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```





---

## 报错

**1.tensorrt-common 报错**  
-Werror=unused-parameter

error: ‘IRNNv2Layer’ is deprecated [-Werror=deprecated-declarations]


**解决方案**

将以下内容添加到 tensorrt_common 包的 CMakeLists.txt 文件中。
```
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-unused-parameter")
```


**2.velodyne_pointcloud报错**

**解决方案**

在CMakeList 中加入

```
cmake_policy(SET CMP0074 NEW)
```
**3.opencv报错**

![Screenshot from 2024-07-19 08-48-27](https://github.com/user-attachments/assets/8438b6a6-e45a-467f-be96-689599f0b16a)

看看哪个项目报错，搜索src中项目，并修改对应cmakelist，添加OpenCV 与 cv_bridge 

```
cmake_minimum_required(VERSION 3.14)
project(traffic_light_map_based_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)

ament_auto_add_library(traffic_light_map_based_detector SHARED
  src/node.cpp
)

target_link_libraries(traffic_light_map_based_detector
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

rclcpp_components_register_node(traffic_light_map_based_detector
  PLUGIN "traffic_light::MapBasedDetector"
  EXECUTABLE traffic_light_map_based_detector_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
```
