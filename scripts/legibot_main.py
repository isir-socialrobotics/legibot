import subprocess
import rospy

from legibot.planners.high_level_planner import MainPlanner
from legibot.utils.gazebo_utils import gazebo_delete_model


def main():
    robot_x0 = (-3.0, 3.0, -1.57)
    observer_x0 = (-3.5, -8.5, 2.3)
    robot_goal = (-2, -7.5)

    # gazebo_delete_model(robot_name="table")
    gazebo_delete_model(robot_name="pepper")
    gazebo_delete_model(robot_name="observer1")

    subprocess.Popen(["roslaunch", "legibot", "spawn_pepper.launch", f"x:={robot_x0[0]}", f"y:={robot_x0[1]}", f"yaw:={robot_x0[2]}"])
    subprocess.Popen(["roslaunch", "legibot", "spawn_observer.launch", f"x:={observer_x0[0]}", f"y:={observer_x0[1]}", f"yaw:={observer_x0[2]}"])

    try:
        rospy.init_node('legibot_node')
    except rospy.exceptions.ROSException:
        pass

    # read goal value
    goal = rospy.get_param("/robot_controller/goal", robot_goal)
    if len(goal) != 2:
        raise RuntimeError("Exiting Trajectory Controller: Invalid Goal")

    planner = MainPlanner(goal)
    planner.exec_loop()

if __name__ == '__main__':
    main()