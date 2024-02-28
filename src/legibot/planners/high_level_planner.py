#!/usr/bin/env python
import os
import sys
import time
import rospy
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from legibot.planners.smoother import smooth_trajectory

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir, 'src')))
from legibot.planners.local_planner import LocalPlanner
from legibot.static_map import StaticMap
from legibot.traj_controller import TrajectoryController
from legibot.utils.bezier import Bezier

def get_bezier_path(x0, g, subgoals):
    b = Bezier([x0] + subgoals + [g], samples=7)
    return np.stack(b.generate_curve(), axis=0).T

class MainPlanner:
    def __init__(self, goals, goal_idx=0, robot_xyt0=None, **kwargs):
        self.goals = goals
        self.goal_idx = goal_idx
        self.robot_xyt = robot_xyt0
        self._controller = TrajectoryController([])
        self.static_map = StaticMap()
        self._local_planner = LocalPlanner(np.array(self.goals),
                                           np.array(self.static_map.obstacles), goal_idx=self.goal_idx, **kwargs)
        # self.goal_publisher = rospy.Publisher('/pepper/goals', PoseArray, queue_size=10)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.plan_pub = rospy.Publisher('/la_planner/plan', PoseArray, queue_size=10)


    # hardcode the trajectory
    def generate_trajectory(self):
        # sub_goals = [[-3, 1], [-1, 0], [1, -1.5]]  # legible
        # sub_goals = [[-3, -1], [-0.7, -2], [-1.5, -4]]  # illegible
        # traj_curve = get_bezier_path(self.robot_xyt[:2], self.goal, sub_goals)
        # self.hardcode_trajectory = [Point(p[0], p[1], 0) for p in traj_curve]

        self._local_planner.optimal_speed_mps = 1
        plan = self._local_planner.full_plan(self.robot_xyt, dt=0.5, H=100)
        plan_smooth = smooth_trajectory(plan, num_points=len(plan) * 2)

        self._controller.trajectory = [Point(p[0], p[1], 0) for p in plan]
        pose_array = PoseArray()
        pose_array.header.frame_id = "odom"
        points = [Point(p[0], p[1], 0) for p in plan]
        vels = [(0, 0)] + [(points[i+1].x - points[i].x, points[i+1].y - points[i].y) for i in range(len(points)-1)]
        yaw = [np.arctan2(v[1], v[0]) for v in vels]
        pose_array.poses = [Pose(p, Quaternion(*quaternion_from_euler(0, 0, yaw[i]))) for i, p in enumerate(points)]
        self.plan_pub.publish(pose_array)
        self._controller.reset()

    def exec_loop(self):
        while True:
            if self.robot_xyt is None:
                print("Waiting for robot position")
                time.sleep(3)
                continue

            if len(self._controller.trajectory) == 0:
                self.generate_trajectory()

            # next_x, _ = self._local_planner.step(self.robot_xyt[:2], dt=0.2)
            # self._controller.trajectory = [Point(next_x[0], next_x[1], 0)]
            self._controller.exec()

            # goal_array = PoseArray()
            # goal_array.header.frame_id = "odom"
            # goal_array.poses = [Pose(Point(traj_curve[i, 0], traj_curve[i, 1])) for i in range(len(traj_curve))]
            # logger.points([Point2(p.x, p.y) for p in controller.trajectory], color=RGB_RED, namespace="Waypoints", radius=0.06)
            # traj = controller.odom_history.get_trajectory() + controller.odom_history.get_trajectory()[::-1]
            # logger.polygon(traj, color=RGB_PURPLE, namespace="Trajectory")
            # for ii, corners in enumerate(tables_4_corners):
            #     logger.polygon(corners, color=RGB_ORANGE, namespace=f"Table_{ii}")

            time.sleep(0.2)

    def get_trajectory_length(self):
        total_length = 0
        for i in range(1, len(self._controller.trajectory)):
            total_length += np.linalg.norm(np.array([self._controller.trajectory[i].x, self._controller.trajectory[i].y]) -
                                           np.array([self._controller.trajectory[i-1].x, self._controller.trajectory[i-1].y]))

    def odom_callback(self, msg: Odometry):
        robot_xyt = [msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     euler_from_quaternion([msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z,
                                            msg.pose.pose.orientation.w])[2]]
        self.robot_xyt = robot_xyt

if __name__ == '__main__':
    try:
        rospy.init_node('hardcoded_planner')
    except rospy.exceptions.ROSException:
        pass

    # read goal value
    goal = rospy.get_param("/robot_controller/goal", "(-2,-7.5)")
    goal = eval(goal)
    if len(goal) != 2:
        raise RuntimeError("Exiting Trajectory Controller: Invalid Goal")

    planner = MainPlanner([goal])
    planner.exec_loop()



