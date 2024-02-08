import math
import os
import subprocess
import time

import rospy

from legibot.planners.high_level_planner import MainPlanner
from legibot.static_map import StaticMap
from legibot.utils.gazebo_utils import gazebo_delete_model, gazebo_spawn_static_model


def main():
    robot_x0 = (-3.0, 3.0, -1.57)

    observers = [(3.5, -2.5, 170),
                 (3.5, -6.5, 160),
                 (-3.5, -8.5, 20),
                 ]
    robot_goal_idx = 0

    # for each observer, create a table in front of them
    tables_xy = []
    for observer in observers:
        observer_xy = observer[:2]
        observer_yaw = math.radians(observer[2])
        table_x = observer_xy[0] + 0.75 * math.cos(observer_yaw)
        table_y = observer_xy[1] + 0.75 * math.sin(observer_yaw)
        tables_xy.append((table_x, table_y))

    robot_goal = (tables_xy[robot_goal_idx][0], tables_xy[robot_goal_idx][1], math.radians(observers[robot_goal_idx][2]))
    other_goals = [(observers[i][0], observers[i][1], math.radians(observers[i][2]))
                     for i in range(len(observers)) if i != robot_goal_idx]

    # read goal value
    # goal = rospy.get_param("/robot_controller/goal", robot_goal)
    # if len(goal) != 2:
    #     raise RuntimeError("Exiting Trajectory Controller: Invalid Goal")
    try:
        rospy.init_node('legibot_node')
    except rospy.exceptions.ROSException:
        print("Node already initialized")

    # planner.static_map.persons
    # planner.static_map.tables

    legibot_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
    round_table_sdf = os.path.join(legibot_dir, "models/round_table/model.sdf")
    person_standing_sdf = os.path.join(legibot_dir, "models/person_standing/model.sdf")

    # gazebo_delete_model(robot_name="table")
    gazebo_delete_model(robot_name="pepper")
    gazebo_delete_model(robot_name="observer")
    gazebo_delete_model(robot_name="table")
    gazebo_delete_model(robot_name="person")

    subprocess.Popen(["roslaunch", "legibot", "spawn_pepper.launch", f"x:={robot_x0[0]}", f"y:={robot_x0[1]}", f"yaw:={robot_x0[2]}"])
    for i in range(len(observers)):
        # subprocess.Popen(["roslaunch", "legibot", "spawn_table.launch", f"x:={tables_xy[i][0]}", f"y:={tables_xy[i][1]}", f"yaw:={observers[i][2]}"])
        gazebo_spawn_static_model(f"table__{i}", round_table_sdf,
                                  f"{tables_xy[i][0]}, {tables_xy[i][1]}, 0, 0, 0, {observers[i][2]}", "world")
        time.sleep(0.2)

        if i == robot_goal_idx:
            subprocess.Popen(["roslaunch", "legibot", "spawn_observer.launch", f"x:={observers[i][0]}",
                              f"y:={observers[i][1]}", f"yaw:={math.radians(observers[i][2]) + math.pi/2}"])
        else:
            gazebo_spawn_static_model(f"person__{i}", person_standing_sdf,
                                        f"{observers[i][0]}, {observers[i][1]}, 0.16, 0, 0, {math.radians(observers[i][2])+ math.pi/2}", "world")
        time.sleep(0.2)

    static_map = StaticMap()
    static_map.tables = tables_xy
    static_map.persons = [obs[:2] for obs in observers]
    static_map.update()

    planner = MainPlanner([robot_goal] + other_goals, goal_idx=0)
    planner.exec_loop()

if __name__ == '__main__':
    main()
