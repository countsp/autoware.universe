# autoware.universe

For Autoware's general documentation, see [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/).

For detailed documents of Autoware Universe components, see [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware.universe/).

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
