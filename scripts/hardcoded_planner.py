#!/usr/bin/env python
import os
import sys
import time
import rospy
import numpy as np
from geometry_msgs.msg import Point, Pose

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir, 'src')))
from legibot.traj_controller import TrajectoryController
from legibot.utils.bezier import Bezier

try:
    rospy.init_node('hardcoded_planner')
except rospy.exceptions.ROSException:
    pass

# read goal value
goal = rospy.get_param("/robot_controller/goal", "(-2.6,-7.1)")
goal = eval(goal)
if len(goal) != 2:
    print("Exiting Trajectory Controller: Invalid Goal")
    sys.exit(1)

# hardcode the trajectory
trajectory = [Point(goal[0], goal[1], 0)]
robot_init_pos = [-3, 3]
sub_goals = [[-3, 1], [-1, 0], [1, -1.5]]  # legible
# sub_goals = [[-3, -1], [-0.7, -2], [-1.5, -4]]  # illegible
b = Bezier([robot_init_pos] + sub_goals + [goal], samples=7)
traj_curve = b.generate_curve()
traj_curve = np.stack(traj_curve, axis=0).T

len_traj = sum([np.linalg.norm(traj_curve[:, i] - traj_curve[:, i - 1]) for i in range(1, len(traj_curve.T))])
print("Total trajectory length: ", len_traj)

controller = TrajectoryController([Point(p[0], p[1], 0) for p in traj_curve])

# goal_publisher = rospy.Publisher('/pepper/goals', PoseArray, queue_size=10)
# goal_array = PoseArray()
# goal_array.header.frame_id = "odom"
# goal_array.poses = [Pose(Point(traj_curve[i, 0], traj_curve[i, 1])) for i in range(len(traj_curve))]

while True:
    rospy.loginfo(f"Trajectory Controller Goal: {controller.get_current_waypoint()}")
    traj = controller.odom_history.get_trajectory() + controller.odom_history.get_trajectory()[::-1]
    controller.exec()

    # goal_publisher.publish(goal_array)

    # logger.points([Point2(p.x, p.y) for p in controller.trajectory], color=RGB_RED, namespace="Waypoints", radius=0.06)
    # logger.polygon(traj, color=RGB_PURPLE, namespace="Trajectory")
    # for ii, corners in enumerate(tables_4_corners):
    #     logger.polygon(corners, color=RGB_ORANGE, namespace=f"Table_{ii}")

    time.sleep(0.2)

