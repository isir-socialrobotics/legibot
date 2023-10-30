#!/usr/bin/env python

import sys
import math
import rospy
import argparse
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

sys.path.append('/home/javad/workspace/catkin_ws/src/cvae-based-legible-motion-generation')
from bezier import Bezier

from vive_ai.utils.point2d import Point2
from vive_ai.consts.consts import *


def angle_to_unitary(angle_rad):
    if angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    elif angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad


def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    return 0


class OdomHistory:
    def __init__(self, max_size=100):
        self.max_size = max_size
        self.history = []

    def append(self, msg: Odometry):
        if len(self.history) > 0:
            last_msg = self.history[-1]
            if msg.header.stamp - last_msg.header.stamp < rospy.Duration(0.5):
                return
        # point = Point2(msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.history.append(msg)
        if len(self.history) > self.max_size:
            self.history.pop(0)

    def get_trajectory(self):
        return [Point2(msg.pose.pose.position.x, msg.pose.pose.position.y) for msg in self.history]


class TrajectoryController:
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.current_index = 0
        self.robot_xy = None
        self.robot_orien = -10
        self.odom_history = OdomHistory()

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(20)

    def odom_callback(self, msg: Odometry):
        self.robot_xy = msg.pose.pose.position
        self.robot_orien = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                  msg.pose.pose.orientation.y,
                                                  msg.pose.pose.orientation.z,
                                                  msg.pose.pose.orientation.w])[2]
        self.odom_history.append(msg)

    def get_current_waypoint(self):
        return self.trajectory[self.current_index]

    @staticmethod
    def is_reached(waypoint, pose: Point):
        dx = waypoint.x - pose.x
        dy = waypoint.y - pose.y
        dist = (dx**2 + dy**2)**0.5
        goal_threshold = 0.25
        return dist < goal_threshold

    def __calc_command__(self) -> Twist:
        command = Twist()
        if self.robot_xy is None:
            return command
        if self.current_index >= len(self.trajectory):
            print("Trajectory finished")
            return command
        if self.is_reached(self.get_current_waypoint(), self.robot_xy):
            self.current_index += 1
            return command

        next_subgoal = self.get_current_waypoint()
        subgoal_vec = Point(next_subgoal.x - self.robot_xy.x, next_subgoal.y - self.robot_xy.y, 0)
        angle_to_subgoal = math.atan2(subgoal_vec.y, subgoal_vec.x)
        d_angle = angle_to_unitary(angle_to_subgoal - self.robot_orien)

        # check for rotation
        if abs(math.degrees(angle_to_unitary(angle_to_subgoal - self.robot_orien))) > 10:
            command.linear.x = 0
            command.angular.z = 0.3 * sign(angle_to_unitary(angle_to_subgoal - self.robot_orien))

        else:
            command.linear.x = 0.5
            command.angular.z = angle_to_unitary(angle_to_subgoal - self.robot_orien) * 0.1  # small orien correction

        return command

    def exec(self):
        command = self.__calc_command__()
        self.cmd_vel_pub.publish(command)
        self.rate.sleep()


if __name__ == "__main__":
    # try:
    #     rospy.init_node('robot_controller')
    # except Exception as e:
    #     print("problem with ros init")
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
    sub_goal = [3, 0]
    b = Bezier([robot_init_pos, sub_goal, goal], samples=5)
    traj_curve = b.generate_curve()
    traj_curve = np.stack(traj_curve, axis=0).T

    # import matplotlib.pyplot as plt
    # plt.scatter([p[0] for p in traj_curve], [p[1] for p in traj_curve])
    # plt.show()

    controller = TrajectoryController([Point(p[0], p[1], 0) for p in traj_curve])
    while not rospy.is_shutdown():
        # logger.info(f"Trajectory Controller Goal: {controller.get_current_waypoint()}")
        logger.points([Point2(p.x, p.y) for p in controller.trajectory], color=RGB_RED, namespace="Waypoints", radius=0.07)
        logger.points(controller.odom_history.get_trajectory(), color=RGB_BLUE, namespace="Trajectory", radius=0.05)
        controller.exec()

