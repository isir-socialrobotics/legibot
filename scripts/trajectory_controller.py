#!/usr/bin/env python

import sys
import math
import rospy
import argparse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist


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


class TrajectoryController:
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.current_index = 0
        self.robot_xy = None
        self.robot_orien = -10

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(20)

    def odom_callback(self, msg: Odometry):
        self.robot_xy = msg.pose.pose.position
        self.robot_orien = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                  msg.pose.pose.orientation.y,
                                                  msg.pose.pose.orientation.z,
                                                  msg.pose.pose.orientation.w])[2]

    def get_current_waypoint(self):
        return self.trajectory[self.current_index]

    @staticmethod
    def is_reached(waypoint, pose: Point):
        dx = waypoint.x - pose.x
        dy = waypoint.y - pose.y
        dist = (dx**2 + dy**2)**0.5
        goal_threshold = 0.1
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
            command.angular.z = angle_to_unitary(angle_to_subgoal - self.robot_orien) * 0.1  # some small rotation

        return command

    def exec(self):
        command = self.__calc_command__()
        self.cmd_vel_pub.publish(command)
        self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node('trajectory_controller')
    args = argparse.ArgumentParser()
    goal = rospy.get_param("/trajectory_controller_node/goal", "()")
    goal = eval(goal)
    if len(goal) != 2:
        print("Exiting Trajectory Controller: Invalid Goal")
        sys.exit(1)
    trajectory = [Point(goal[0], goal[1], 0)]
    controller = TrajectoryController(trajectory)
    while not rospy.is_shutdown():
        controller.exec()
