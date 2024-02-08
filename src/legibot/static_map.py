import os
import numpy as np
import xml.etree.ElementTree as ET
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from legibot.utils.singleton import Singleton
from legibot.utils.viz_utils import Visualizer


class StaticMap(metaclass=Singleton):
    def __init__(self):
        # self.tables_center_xy, self.observers_xy = self.parse_gazebo_world(world_filename)
        legibot_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        self.tables, self.persons = self.parse_gazebo_world(os.path.join(legibot_dir, "worlds/restaurant.world"))

        # extract walls
        walls_list = self.parse_walls(os.path.join(legibot_dir, "worlds/walls.sdf"))
        self.walls = self.conv_walls_to_obstacles(walls_list)

        self.human_radius = 0.3
        self.table_radius = 0.6
        self.update()

        # tables_center_xy, observers_xy = self.parse_gazebo_world(
        #     "/home/javad/workspace/catkin_ws/src/legibot/worlds/restaurant.world")

    def update(self):
        self.obstacles = (self.walls
                          + [[p[0], p[1], self.human_radius] for p in self.persons]
                          + [[p[0], p[1], self.table_radius] for p in self.tables]
                          )
        self.observers = [[p[0], p[1]] for p in self.persons]

    @staticmethod
    def parse_gazebo_world(world_filename):
        tree = ET.parse(world_filename)
        root = tree.getroot().find('world')

        tables_6d_pose = []
        observers_6d_pose = []

        # Find and list all elements of type "include"
        for include_element in root.findall('.//include'):
            pose = include_element.find('pose').text
            pose = [float(p) for p in pose.split(' ')]
            xyz, rpy = pose[:3], pose[3:]

            if not hasattr(include_element, "uri"):
                include_element.attrib["uri"] = include_element.find("uri").text

            if include_element.attrib["uri"] == "model://round_table":
                tables_6d_pose.append([*xyz, *rpy])
            elif include_element.attrib["uri"] == "model://person_standing":
                observers_6d_pose.append([*xyz, *rpy])

        print(f'Tables Center XY: {tables_6d_pose}')
        print(f'Observers XY: {observers_6d_pose}')
        return tables_6d_pose, observers_6d_pose

    @staticmethod
    def parse_walls(walls_filename):
        walls = []

        tree = ET.parse(walls_filename)
        root = tree.getroot().find('model')
        root_pose = root.find('pose').text
        root_pose = [float(p) for p in root_pose.split(' ')]
        xyz = root_pose[:3]
        # rpy = root_pose[3:]
        # quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        root_drift = Point(xyz[0], xyz[1], xyz[2])

        # Find and list all elements of type "link"
        for link_element in root.findall('.//link'):
            pose = link_element.find('pose').text
            pose = [float(p) for p in pose.split(' ')]
            xyz = pose[:3]
            rpy = pose[3:]
            quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            link_pose = Pose(Point(xyz[0] + root_drift.x, xyz[1] + root_drift.y, xyz[2] + root_drift.z),
                        Quaternion(quat[0], quat[1], quat[2], quat[3]))


            size = link_element.find('collision').find('geometry').find('box').find('size').text
            size = [float(s) for s in size.split(' ')]
            walls.append([link_pose, size])

            print(f'Link Element Text: {link_element.attrib["name"]} {link_element.find("pose").text}')
        return walls

    @staticmethod
    def conv_walls_to_obstacles(walls):
        obs_ = []
        for wall in walls:
            for ii in np.arange(-wall[1][0] / 2, wall[1][0] / 2 + 0.1, 0.2):
                yaw = euler_from_quaternion([wall[0].orientation.x,
                                                                wall[0].orientation.y,
                                                                wall[0].orientation.z,
                                                                wall[0].orientation.w])[2]

                obs_.append([wall[0].position.x + ii * np.cos(yaw),
                            wall[0].position.y + ii * np.sin(yaw),
                            0.1])
        return obs_

if __name__ == '__main__':
    static_map = StaticMap()
    vis = Visualizer()
    vis.draw_obstacles(static_map.obstacles)
    vis.show()