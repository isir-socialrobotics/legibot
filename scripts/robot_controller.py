#!/usr/bin/env python

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Point
from legibot.traj_controller import TrajectoryController

sys.path.append('/home/javad/workspace/catkin_ws/src/cvae-based-legible-motion-generation')
from bezier import Bezier

from vive_ai.utils.point2d import Point2
from legibot.colors import RGB_PURPLE, RGB_RED, RGB_ORANGE

# rospy.init_node('robot_controller')
from vive_ai.core.ros_manager import RosNodeManager
RosNodeManager().connect(node_name="PathPlanner")
from vive_ai.logger.logger_factory import logger

goal = rospy.get_param("/robot_controller/goal", "(-2.6,-7.1)")
goal = eval(goal)
if len(goal) != 2:
    print("Exiting Trajectory Controller: Invalid Goal")
    sys.exit(1)
trajectory = [Point(goal[0], goal[1], 0)]
robot_init_pos = [-3, 3]
sub_goals = [[-3, 1], [-1, 0], [1, -1.5]]  # legible
# sub_goals = [[-3, -1], [-0.7, -2], [-1.5, -4]]  # illegible
b = Bezier([robot_init_pos] + sub_goals + [goal], samples=7)
traj_curve = b.generate_curve()
traj_curve = np.stack(traj_curve, axis=0).T

len_traj = [np.linalg.norm(traj_curve[:, i] - traj_curve[:, i - 1]) for i in range(1, len(traj_curve.T))]
print("Total trajectory length: ", sum(len_traj))

tables_centers = [[-2.5, -8], [-2.5, -4], [2, -8]]
tables_4_corners = np.array([[x+0.2, y+0.5, x-0.2, y+0.5, x-0.2, y-0.5, x+0.2, y-0.5] for x, y in tables_centers]).reshape(-1, 4, 2)

controller = TrajectoryController([Point(p[0], p[1], 0) for p in traj_curve])
while not rospy.is_shutdown():
    # logger.info(f"Trajectory Controller Goal: {controller.get_current_waypoint()}")
    logger.points([Point2(p.x, p.y) for p in controller.trajectory], color=RGB_RED, namespace="Waypoints", radius=0.06)
    traj = controller.odom_history.get_trajectory() + controller.odom_history.get_trajectory()[::-1]
    logger.polygon(traj, color=RGB_PURPLE, namespace="Trajectory")
    controller.exec()

    for ii, corners in enumerate(tables_4_corners):
        logger.polygon(corners, color=RGB_ORANGE, namespace=f"Table_{ii}")

