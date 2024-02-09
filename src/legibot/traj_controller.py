import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from legibot.utils.point2d import Point2


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
    def __init__(self, max_size=500):
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

    def get_instant_speed(self):
        if len(self.history) < 2:
            return 0
        last_msg = self.history[-1]
        prev_msg = self.history[-2]
        dt = (last_msg.header.stamp - prev_msg.header.stamp).to_sec()
        dx = last_msg.pose.pose.position.x - prev_msg.pose.pose.position.x
        dy = last_msg.pose.pose.position.y - prev_msg.pose.pose.position.y
        return ((dx**2 + dy**2)**0.5) / dt


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
        if self.current_index >= len(self.trajectory):
            return self.trajectory[-1]
        return self.trajectory[self.current_index]

    def reset(self):
        self.current_index = 0

    @staticmethod
    def is_reached(waypoint, pose: Point):
        dx = waypoint.x - pose.x
        dy = waypoint.y - pose.y
        dist = (dx**2 + dy**2)**0.5
        goal_threshold = 0.35
        return dist < goal_threshold

    def __calc_command__(self) -> Twist:
        command = Twist()
        if self.robot_xy is None:
            return command
        if self.current_index >= len(self.trajectory):
            # print("Trajectory finished")
            return command
        if self.is_reached(self.get_current_waypoint(), self.robot_xy):
            self.current_index += 1
            # return command

        next_subgoal = self.get_current_waypoint()
        subgoal_vec = Point(next_subgoal.x - self.robot_xy.x, next_subgoal.y - self.robot_xy.y, 0)
        angle_to_subgoal = math.atan2(subgoal_vec.y, subgoal_vec.x)
        d_angle = angle_to_unitary(angle_to_subgoal - self.robot_orien)

        # check for rotation
        # print("robot speed", self.odom_history.get_instant_speed())
        if abs(math.degrees(angle_to_unitary(angle_to_subgoal - self.robot_orien))) > 10:
            command.linear.x = 0.25
            command.angular.z = 0.8 * sign(angle_to_unitary(angle_to_subgoal - self.robot_orien))

        else:
            command.linear.x = 1.2
            command.angular.z = angle_to_unitary(angle_to_subgoal - self.robot_orien) * 0.3  # small orien correction

        return command

    def exec(self):
        try:
            command = self.__calc_command__()
            self.cmd_vel_pub.publish(command)
            self.rate.sleep()
        except KeyboardInterrupt:
            print("Shutting down")
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(1)
            rospy.signal_shutdown("KeyboardInterrupt")
            raise KeyboardInterrupt
