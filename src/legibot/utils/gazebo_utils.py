import math
import os
import time
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Empty

import glob
import rospkg
from tf.transformations import quaternion_from_euler


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


def gazebo_delete_model(timeout_sec=3, robot_name=""):
    try:
        world_properties = gazebo_world_properties(timeout_sec)
        for name in world_properties.model_names:
            if robot_name in name:
                rospy.wait_for_service("gazebo/delete_model", timeout=timeout_sec)
                delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
                delete_model(name)
        else:
            pass

    except Exception as e:  # fixme: be more specific on errors
        print("Error: Wait For Service (Gazebo/delete model) Failed!", e)

def gazebo_spawn_static_model(model_name, model_filename, model_pose, reference_frame="world"):
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    # Args: model_name model_xml robot_namespace initial_pose reference_frame
    if isinstance(model_pose, str):
        model_pose = [float(p) for p in model_pose.split(',')]
        model_pose = Pose(Point(*model_pose[:3]), Quaternion(*quaternion_from_euler(*model_pose[3:])))
    model_xml = open(model_filename, "r").read()
    spawn_model(model_name, model_xml, "", model_pose, reference_frame)


def gazebo_find_launch_file(filename) -> str:
    tmp_file_to_save_launch_file_address = "/var/tmp/find_launch_file.txt"
    os.system(f"locate launch/{filename} > " + tmp_file_to_save_launch_file_address)
    with open(tmp_file_to_save_launch_file_address) as f:
        roslaunch_file = f.readline()
        if len(roslaunch_file) == 0:
            raise FileNotFoundError(f"Could not find file {filename} in your computer! |\n"
                                    f"Make sure that you have `locate` command installed (sudo apt install mlocate)")
        else:
            roslaunch_file = roslaunch_file[:-1]
    return roslaunch_file

def pause_gazebo():
    rospy.wait_for_service("gazebo/pause_physics")
    pause_physics = rospy.ServiceProxy("gazebo/pause_physics", Empty)
    pause_physics()


def unpause_gazebo():
    rospy.wait_for_service("gazebo/unpause_physics")
    unpause_physics = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
    unpause_physics()


if __name__ == "__main__":
    gazebo_world_properties()

    round_table_sdf = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, os.pardir, "models/round_table/model.sdf"))
    gazebo_spawn_static_model("table__01", round_table_sdf, "1, 0, 0, 0, 0, 0", "world")
    gazebo_launch_files_, world_files_ = find_gazebo_sim_launch_files()
    print(gazebo_launch_files_)
    print(world_files_)
