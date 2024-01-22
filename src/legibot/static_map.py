# <include>
#       <uri>model://round_table</uri>
#       <name>table_1</name>
#       <pose>-2.5 -8 0 0 0 1.57</pose>
#       <static>true</static>
#     </include>
#
#     <include>
#       <uri>model://round_table</uri>
#       <name>table_2</name>
#       <pose>-2.5 -4 0 0 0 1.57</pose>
#       <static>true</static>
#     </include>
#
#     <include>
#       <uri>model://person_standing</uri>
#       <name>person_2</name>
#       <pose>3.4 -8 0.16 0 0 -1.5707</pose>
#       <static>true</static>
#     </include>
#
#     <include>
#       <uri>model://round_table</uri>
#       <name>table_3</name>
#       <pose>2 -8 0 0 0 1.57</pose>
#       <static>true</static>
#     </include>
#
#     <include>
#       <uri>model://person_standing</uri>
#       <name>person_3</name>
#       <pose>-3.85 -4 0.16 0 0 1.5707</pose>
#       <static>true</static>
#     </include>
#
#     <include>
#       <uri>model://round_table</uri>
#       <name>round_table_1</name>
#       <pose>2 -4.0 0 0 0 1.57</pose>
#       <static>true</static>
#     </include>
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import tf

from legibot.utils.viz_utils import Visualizer


def parse_walls(walls_filename):
    import xml.etree.ElementTree as ET
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

def conv_walls_to_obstacles(walls):
    obs_ = []
    for wall in walls:
        for ii in np.arange(-wall[1][0] / 2, wall[1][0] / 2 + 0.1, 0.2):
            yaw = tf.transformations.euler_from_quaternion([wall[0].orientation.x,
                                                            wall[0].orientation.y,
                                                            wall[0].orientation.z,
                                                            wall[0].orientation.w])[2]

            obs_.append([wall[0].position.x + ii * np.cos(yaw),
                        wall[0].position.y + ii * np.sin(yaw),
                        0.1])
    return obs_

observers = [
    [3.4, -8],
    [-3.85, -4]
]

tables = [
    [-2.5, -8, 0.7],
    [-2.5, -4, 0.7],
    [2, -8, 0.7],
    [2, -4, 0.7],
]

walls_list = parse_walls("/home/javad/workspace/catkin_ws/src/legibot/worlds/walls.sdf")
walls_obs = conv_walls_to_obstacles(walls_list)
obstacles = tables + walls_obs + [[p[0], p[1], 0.3] for p in observers]


if __name__ == '__main__':
    vis = Visualizer()
    vis.draw_obstacles(obstacles)
    vis.show()