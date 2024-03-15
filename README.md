# autoware.universe

For Autoware's general documentation, see [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/).

For detailed documents of Autoware Universe components, see [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware.universe/).

---
### Source installation

#### Prerequisites
Ubuntu 20.04

ROS 2 Galactic
   
```
sudo apt-get -y update
sudo apt-get -y install git
```

How to set up a development environment

Clone autowarefoundation/autoware and move to the directory.

```
git clone https://github.com/autowarefoundation/autoware.git -b galactic
cd autoware
```
or download zip file using galactic branch 

You can install the dependencies either manually or using the provided Ansible script.

Note: Before installing NVIDIA libraries, confirm and agree with the licenses.

**CUDA**

**cuDNN**
    
**TensorRT**

##### Installing dependencies manually

    Install ROS 2
    Install ROS 2 Dev Tools
    Install the RMW Implementation
    Install pacmod
    Install Autoware Core dependencies
    Install Autoware Universe dependencies
    Install pre-commit dependencies
    Install Nvidia CUDA
    Install Nvidia cuDNN and TensorRT

##### Installing dependencies using Ansible(optional)

Be very careful with this method. Make sure you read and confirmed all the steps in the Ansible configuration before using it.

If you've manually installed the dependencies, you can skip this section.
```
./setup-dev-env.sh
```
##### How to set up a workspace

    Create the src directory and clone repositories into it.

    Autoware uses vcstool to construct workspaces.
```
cd autoware
mkdir src
vcs import src < autoware.repos
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
    

##### How to update a workspace(optional)

    Update the .repos file.
```
cd autoware
git pull
```
Update the repositories.
```
vcs import src < autoware.repos
vcs pull src
```
For Git users:

    vcs import is similar to git checkout.
        Note that it doesn't pull from the remote.
    vcs pull is similar to git pull.
        Note that it doesn't switch branches.

For more information, refer to the official documentation.

Install dependent ROS packages.

```
source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

Build the workspace.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

### installation
reference: [https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/)
-> Installing dependencies manually

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

