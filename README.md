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
```

### ROS2
```
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-librealsense2* realsense2-viewer
cd ~/ros2_ws/src
git clone https://github.com/ros-naoqi/naoqi_driver2.git
git clone https://github.com/mgonzs13/yolov8_ros.git
```

source /home/jetson/catkin_ws/devel/setup.bash
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
