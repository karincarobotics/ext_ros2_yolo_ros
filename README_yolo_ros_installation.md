# RKNN Toolkit and YOLO Installation Guide

This documentation provides a comprehensive guide for setting up RKNN Toolkit and YOLO on both embedded boards (Orange Pi/Radxa with RK3588) and desktop PCs for model training and conversion.

## Table of Contents
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Board Setup (RK3588 - ARM64)](#board-setup-rk3588---arm64)
- [Desktop Setup (AMD64)](#desktop-setup-amd64)
- [Model Conversion](#model-conversion)
- [ROS2 Integration](#ros2-integration)
- [Troubleshooting](#troubleshooting)
- [References](#references)

## Overview

This setup involves two main environments:
- **Board Environment**: ARM64 device with NPU (Neural Processing Unit) for inference
- **Desktop Environment**: AMD64 PC with CUDA support for model training and conversion

**Important**: Model conversion must be performed on AMD64 architecture, then transferred to the board.

## Prerequisites

### System Requirements
- **Board**: Orange Pi 5 / Radxa Rock 5C with RK3588 chipset
- **Desktop**: AMD64 architecture with NVIDIA GPU and CUDA support
- **OS**: Linux-based system with ROS2 installed
- **Python**: Version 3.10 or higher

### [RK3588] Verify NPU Functionality
Before starting, check if your NPU is properly recognized:
```
sudo cat /sys/kernel/debug/rknpu/version
#Expected output: RKNPU driver: v0.9.7
```

### [CUDA] Verify GPU Functionality


## Board Setup (RK3588 - ARM64)

```
export YOLO_ROS_INSTALL_DIR=$HOME/Projects/karinca_ros2_ws/src/ext_ros2_yolo_ros
sudo apt install python3-venv
cd $YOLO_ROS_INSTALL_DIR
```
Add --system-site-packages to enable access to system packages as well so we can use colcon etc. See also: https://answers.ros.org/question/414012/
```
python3 -m venv venv --system-site-packages --symlinks
. venv/bin/activate
pip3 install -r requirements.txt
# Install rknn-toolkit-lite2
pip3 install rknn-toolkit-lite2
```

Download and install latest rockchip librknnt.so This is the runtime library for the RKNN Toolkit and provided by rockchip.
```
wget https://github.com/airockchip/rknn-toolkit2/raw/refs/heads/master/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so
sudo mv librknnrt.so /usr/lib
```

Compile the project
```
colcon build --symlink-install
```

Add this section to `$HOME/ros2init.bash` to enable yolo_ros packages.

* **local_setup.bash:** This is needed for ros2 to find the packages.
* **PYTHONPATH:** This is needed for python scripts to find relevant libraries even if we're not using a virtual environment.
  See also this life saver document relevant with PYTHONPATH variable update
  https://robotics.stackexchange.com/questions/98214/how-to-use-python-virtual-environments-with-ros2

```
#Overlay yolo_ros packages 
source [YOLO_ROS_INSTALL_DIRECTORY]/install/local_setup.bash
export PYTHONPATH="[YOLO_ROS_INSTALL_DIRECTORY]/venv/lib/python3.12/site-packages":$PYTHONPATH 
```

### Test on the board
**Note** that the yolo_bringup/models directory is prebuilt for RKNN (Rockchip NPU models). See also [Model Conversion](#model-conversion) section to generate custom models.
#### Large model
```
cd $YOLO_ROS_INSTALL_DIR/yolo_bringup/models
yolo predict model='./yolo11n_rknn_model' source='https://ultralytics.com/images/bus.jpg'
```
#### Small model
yolo predict model='./yolo11s_rknn_model' source='https://ultralytics.com/images/bus.jpg'

## Desktop Setup (AMD64)
```
export YOLO_ROS_INSTALL_DIR=$HOME/Projects/karinca_ros2_ws/src/ext_ros2_yolo_ros
sudo apt install python3-venv
cd $YOLO_ROS_INSTALL_DIR
```
Add --system-site-packages to enable access to system packages as well so we can use colcon etc. See also: https://answers.ros.org/question/414012/
```
python3 -m venv venv --system-site-packages --symlinks
. venv/bin/activate
```

Install requirements with cuda 12.8 support. See also https://pytorch.org/get-started/locally/
```
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
```

Install remaining requirements:
```
pip3 install -r requirements.txt
```

Compile the project
```
colcon build --symlink-install
```

Add this section to `$HOME/ros2init.bash` to enable yolo_ros packages.

* **local_setup.bash:** This is needed for ros2 to find the packages.
* **PYTHONPATH:** This is needed for python scripts to find relevant libraries even if we're not using a virtual environment.
See also this life saver document relevant with PYTHONPATH variable update 
https://robotics.stackexchange.com/questions/98214/how-to-use-python-virtual-environments-with-ros2

```
#Overlay yolo_ros packages 
source [YOLO_ROS_INSTALL_DIRECTORY]/install/local_setup.bash
export PYTHONPATH="[YOLO_ROS_INSTALL_DIRECTORY]/venv/lib/python3.12/site-packages":$PYTHONPATH 
```

# Model Conversion
Convertion must be on a x64, AMD64 PC, ARM architeture is not supported!

Convert your model of choice from ultralytics on desktop PC we've converted the following. (See the documentation from Radxa for additional information https://docs.radxa.com/en/rock5/rock5c/app-development/rknn_ultralytics)

```
yolo export model=yolo11s.pt format=rknn name=rk3588
yolo export model=yolo11n.pt format=rknn name=rk3588
#Copy the folders orangepi/radxa
```

## ROS2 Integration

Needs to be cleaned up, draft documentation...
```
ros2 launch karinca_turret bringup_manipulator_only_launch.py
ros2 launch yolo_bringup bringup_camera_launch.py
ros2 launch yolo_bringup yolov11_opi.launch.py
/home/orangepi/Projects/testudo_docta_deploy/install/karinca_turret/bin/karinca_turret/karinca_turret_controller


#Debug stuff
ros2 run rqt_image_view rqt_image_view
ros2 run yolo_ros debug_node --ros-args   -p image_reliability:=1   --remap image_raw:=/camera_turret/color/image_raw   --remap detections:=/detections
ros2 topic pub -r 5 /commands std_msgs/msg/Float64MultiArray '{data: [0.01, 0.01]}'
```

## Troubleshooting
* If framerate is too low, make sure GPU/NPU is working properly. On Orange Pi, check if you can run `yolo predict` on the command line.
On orangepi you can check if the NPU is working properly by running, there should be some load while yolo is running.
```
sudo cat /sys/kernel/debug/rknpu/load
```
On desktop, make sure NVIDIA drivers are installed properly. Execute the following command to check if the GPU is working properly. It should display GPU, CUDA etc version (see the example output below)
```
nvidia-smi
```
Output:
```
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 575.57.08              Driver Version: 575.57.08      CUDA Version: 12.9     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 5080        On  |   00000000:0B:00.0  On |                  N/A |
|  0%   44C    P8             22W /  360W |     901MiB /  16303MiB |      7%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
                                                                                         
+-----------------------------------------------------------------------------------------+
| Processes:                                                                              |
|  GPU   GI   CI              PID   Type   Process name                        GPU Memory |
|        ID   ID                                                               Usage      |
|=========================================================================================|
|    0   N/A  N/A            3142      G   /usr/lib/xorg/Xorg                      436MiB |
|    0   N/A  N/A            3361      G   /usr/bin/gnome-shell                     44MiB |
|    0   N/A  N/A            4071      G   ...exec/xdg-desktop-portal-gnome          6MiB |
|    0   N/A  N/A            4382      G   ...slack/216/usr/lib/slack/slack         49MiB |
|    0   N/A  N/A           10393      G   ...ersion=20250928-180050.101000        108MiB |
|    0   N/A  N/A          147944      G   ...ess --variations-seed-version        166MiB |
+-----------------------------------------------------------------------------------------+
```

* If you get `ModuleNotFoundError: No module named 'torch'` Make sure PYTHONPATH is set properly.
