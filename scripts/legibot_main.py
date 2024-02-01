import subprocess
import rospy

from legibot.planners.high_level_planner import MainPlanner


def main():
    # <!--   <include file="$(find legibot)/launch/spawn_pepper.launch" > -->
    # <!--     <arg name="x" value="-3.0"/> -->
    # <!--     <arg name="y" value="3.0"/> -->
    # <!--     <arg name="yaw" value="-1.57"/> -->
    # <!--   </include> -->
    #
    # <!--   <include file="$(find legibot)/launch/spawn_observer.launch"> -->
    # <!--     <arg name="x" value="-3.5"/> -->
    # <!--     <arg name="y" value="-8.5"/> -->
    # <!--     <arg name="yaw" value="2.3"/> -->
    # <!--     <arg name="obj_name" value="observer1" /> -->
    # <!--   </include> -->

    subprocess.Popen(["roslaunch", "legibot", "spawn_pepper.launch", "x:=-3.0", "y:=3.0", "yaw:=-1.57"])
    subprocess.Popen(["roslaunch", "legibot", "spawn_observer.launch", "x:=-3.5", "y:=-8.5", "yaw:=2.3", "obj_name:=observer1"])

    try:
        rospy.init_node('legibot_node')
    except rospy.exceptions.ROSException:
        pass

    # read goal value
    goal = rospy.get_param("/robot_controller/goal", "(-2,-7.5)")
    goal = eval(goal)
    if len(goal) != 2:
        raise RuntimeError("Exiting Trajectory Controller: Invalid Goal")

    planner = MainPlanner(goal)
    planner.exec_loop()

if __name__ == '__main__':
    main()