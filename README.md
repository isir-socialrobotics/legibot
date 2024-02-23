# Legibot: Legible Motions for Service Robots

## Introduction

## Setup Stack on Jetson

Don't forget to add the following line to the `.bashrc` of Jetson:
```sh
export MY_IP=$(ip -4 addr show eth0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
export ROS_MASTER_URI=http://$MY_IP:11311
export ROS_HOSTNAME=$MY_IP
echo "JETSON_IP: $MY_IP"
```

### General dependencies
```sh
sudo apt install sshpass
sudo apt install python-is-python3 # use python3 as default python version:
```

### ROS1
```
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-librealsense2* realsense2-viewer
cd ~/catkin_ws/src
git clone https://github.com/ros-naoqi/naoqi_driver.git
git clone https://github.com/pal-robotics/aruco_ros.git
```

### ROS2
Add the following lines to the `.bashrc` in Jetson:
```sh
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
```

```
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-librealsense2* realsense2-viewer
sudo apt-get install ros-$ROS_DISTRO-naoqi-libqi ros-$ROS_DISTRO-naoqi-libqicore ros-$ROS_DISTRO-naoqi-bridge-msgs
cd ~/ros2_ws/src
git clone https://github.com/ros-naoqi/naoqi_driver2.git
git clone https://github.com/mgonzs13/yolov8_ros.git
git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git
pip3 install -r yolov8_ros/requirements.txt
cd ~/ros2_ws
colcon build --continue-on-error
```

### Run the stack on Jetson (ROS2)
```sh   
# comment line 80 in yolov8_ros/yolov8_node.py (self.yolo.fuse())
ros2 run tf2_ros static_transform_publisher 1 0 0 0 0 0 map base_link
ros2 launch realsense2_camera rs_launch.py
ros2 launch yolov8_bringup yolov8_3d.launch.py input_image_topic:=/camera/color/image_raw input_depth_topic:=/camera/depth/image_rect_raw input_camera_info_topic:=/camera/color/camera_info input_depth_info_topic:=/camera/depth/camera_info model:=yolov8n.pt threshold:=0.2 target_frame:=map                                             
```


### Simulation (Gazebo)
* Install Pepper ROS library:
Add this library to your catkin workspace and compile it:
    ```
    cd $CATKIN_WS/src
    git clone https://github.com/michtesar/pepper_ros
    cd ..
    catkin_make
    ```

[//]: # (* Install turtlebot3 ros package:)
[//]: # (    ```sh)
[//]: # (    sudo apt install ros-$ROS_DISTRO-turtlebot3-description)
[//]: # (    ```)



## How to Cite
