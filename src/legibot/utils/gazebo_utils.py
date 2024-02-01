import math
import os
import time
from threading import Thread

from typing import Tuple

import roslaunch
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Empty

import glob
import rospkg


def find_gazebo_sim_launch_files():
    rospack = rospkg.RosPack()
    all_packages = rospack.list()

    launch_files = {}
    world_files = {}
    gazebo_launch_files = {}
    for package in all_packages:
        package_path = rospack.get_path(package)
        launch_files_i = glob.glob(os.path.join(package_path, 'launch', '*.launch'))
        world_files_i = glob.glob(os.path.join(package_path, 'world*', '*.world'))
        if len(world_files_i) == 0:
            continue

        if len(launch_files_i) > 0:
            launch_files[package] = launch_files_i
            world_files[package] = world_files_i

            for launch_file in launch_files_i:
                with open(launch_file, 'r') as f:
                    launch_file_contents = f.readlines()
                    for l in launch_file_contents:
                        if l.strip().startswith("<!--"):
                            continue
                        if 'gazebo_ros)/launch/empty_world.launch' in l:
                            gazebo_launch_files[f"{package}: {os.path.basename(launch_file).split('.')[0]}"] = launch_file
                            break
    return gazebo_launch_files, world_files


def gazebo_world_properties(timeout_sec=5):
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=timeout_sec)
    get_world_prop = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    world_properties = get_world_prop()
    # print(world_properties)
    return world_properties


def list_of_existing_models():
    world_properties = gazebo_world_properties()
    return world_properties.model_names


def gazebo_delete_all_balls(timeout_sec=5):
    try:
        world_properties = gazebo_world_properties(timeout_sec)
        for name in world_properties.model_names:
            if name.startswith("ball"):
                rospy.wait_for_service("gazebo/delete_model", timeout=timeout_sec)
                delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
                delete_model(name)

    except Exception as e:  # fixme: be more specific on errors
        print("Error: Wait For Service (Gazebo/delete model) Failed!", e)


def gazebo_spawn_ball(xy: Tuple, index=1):
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    tmp_file_to_save_tennis_ball_model_address = "/var/tmp/find-tennis-ball.txt"
    os.system("locate tennis_ball/model.sdf > " + tmp_file_to_save_tennis_ball_model_address)
    if os.path.exists(tmp_file_to_save_tennis_ball_model_address):
        with open(tmp_file_to_save_tennis_ball_model_address) as f:
            roslaunch_file = f.readline()
            if len(roslaunch_file) == 0:
                raise FileNotFoundError("Could not find spawn_vivebot.launch in your computer!")
            else:
                tennis_ball_file = roslaunch_file[:-1]

    with open(tennis_ball_file, "r") as f:
        tennis_ball_sdf = f.read()

    ball_name = "ball{}".format(index)
    while ball_name in list_of_existing_models():
        index += 1
        ball_name = "ball{}".format(index)

    orient = Quaternion()
    item_pose = Pose(Point(x=xy[0], y=xy[1], z=0.1), orient)
    spawn_model(ball_name, tennis_ball_sdf, "", item_pose, "world")


def gazebo_delete_vivebot(timeout_sec=5):
    try:
        world_properties = gazebo_world_properties(timeout_sec)
        for name in world_properties.model_names:
            if "vivebot" in name:
                rospy.wait_for_service("gazebo/delete_model", timeout=timeout_sec)
                delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
                delete_model(name)
        else:
            pass

    except Exception as e:  # fixme: be more specific on errors
        print("Error: Wait For Service (Gazebo/delete model) Failed!", e)


def gazebo_find_launch_file(filename) -> str:
    tmp_file_to_save_launch_file_address = "/var/tmp/find-vivebot.launch.txt"
    os.system(f"locate launch/{filename} > " + tmp_file_to_save_launch_file_address)
    with open(tmp_file_to_save_launch_file_address) as f:
        roslaunch_file = f.readline()
        if len(roslaunch_file) == 0:
            raise FileNotFoundError(f"Could not find file {filename} in your computer! |\n"
                                    f"make sure you have the vive-simulation package on your computer "
                                    f"and that you have locate installed (sudo apt install mlocate)")
        else:
            roslaunch_file = roslaunch_file[:-1]
    return roslaunch_file


def gazebo_spawn_vivebot(pose=(0, 0, 0)):
    xyt0 = pose
    print("To Spawn Robot @ [{:.2f}, {:.2f}], {:.2f}º".format(xyt0[0], xyt0[1], xyt0[2]))
    try:
        def call_roslaunch():
            os.system(f"roslaunch vive-simulation spawn_vivebot.launch x:={xyt0[0]} y:={xyt0[1]} yaw:={xyt0[2]}")
        Thread(target=call_roslaunch).start()
        print("-----------------------")
        time.sleep(3)  # wait for vivebot to spawn

    except Exception as e:
        print("Error: Gazebo Spawn Robot Failed!", e)


def pause_gazebo():
    rospy.wait_for_service("gazebo/pause_physics")
    pause_physics = rospy.ServiceProxy("gazebo/pause_physics", Empty)
    pause_physics()


def unpause_gazebo():
    rospy.wait_for_service("gazebo/unpause_physics")
    unpause_physics = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
    unpause_physics()


if __name__ == "__main__":
    print(gazebo_find_launch_file("spawn_vivebot.launch"))
    gazebo_world_properties()
    # spawn_vivebot()

    gazebo_launch_files_, world_files_ = find_gazebo_sim_launch_files()
    print(gazebo_launch_files_)
    print(world_files_)
