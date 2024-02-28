import math
import os
import subprocess
import time
import rospy

from legibot.planners.high_level_planner import MainPlanner
from legibot.static_map import StaticMap
from legibot.utils.gazebo_utils import gazebo_delete_model, gazebo_spawn_static_model


def main():
    robot_x0 = (-0., 3.5, -1.57)

    # Scenario 1
    observers = [(0.7, -6, 90),
                 (-0.7, -6., 90),
                 # (-3, -5, 30),
                 ]

    clutter_tables = [
        (0.4, -0.5, 0.2),
        # (-2.5, -5.5, 0),
    ]
    static_map = StaticMap()
    static_map.tables = static_map.tables + clutter_tables
    StaticMap().update()

    # Scenario 2
    # observers = [(2.5, -2.5, 170),
    #              # (2, -4.5, 160),
    #              # (2.5, -6.5, 160),
    #              (-2.5, -8.5, -15),
    #              ]

    robot_goal_idx = 0  # 1

    # for each observer, create a table in front of them
    tables_xy = []
    for observer in observers:
        observer_xy = observer[:2]
        observer_yaw = math.radians(observer[2])
        table_x = observer_xy[0] + 0.8 * math.cos(observer_yaw)
        table_y = observer_xy[1] + 0.8 * math.sin(observer_yaw)
        tables_xy.append((table_x, table_y))

    robot_goal = (tables_xy[robot_goal_idx][0], tables_xy[robot_goal_idx][1], math.radians(observers[robot_goal_idx][2]))
    other_goals = [(tables_xy[i][0], tables_xy[i][1], math.radians(observers[i][2]))
                   for i in range(len(observers)) if i != robot_goal_idx]

    # read goal value
    # goal = rospy.get_param("/robot_controller/goal", robot_goal)
    # if len(goal) != 2:
    #     raise RuntimeError("Exiting Trajectory Controller: Invalid Goal")
    try:
        rospy.init_node('legibot_node')
    except rospy.exceptions.ROSException:
        print("Node already initialized")

    legibot_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
    round_table_sdf = os.path.join(legibot_dir, "models/round_table/model.sdf")
    person_standing_sdf = os.path.join(legibot_dir, "models/person_standing/model.sdf")
    clutter_cylinder_sdf = os.path.join(legibot_dir, "models/clutter_cylinder/model.sdf")

    # gazebo_delete_model(robot_name="table")
    gazebo_delete_model(robot_name="pepper")
    subprocess.Popen(["roslaunch", "legibot", "spawn_pepper.launch", f"x:={robot_x0[0]}", f"y:={robot_x0[1]}", f"yaw:={robot_x0[2]}"])

    gazebo_delete_model(robot_name="observer")
    gazebo_delete_model(robot_name="table")
    gazebo_delete_model(robot_name="person")
    gazebo_delete_model(robot_name="clutter_cyl")

    for i in range(len(observers)):
        # subprocess.Popen(["roslaunch", "legibot", "spawn_table.launch", f"x:={tables_xy[i][0]}", f"y:={tables_xy[i][1]}", f"yaw:={observers[i][2]}"])
        gazebo_spawn_static_model(f"table__{i}", round_table_sdf,
                                  f"{tables_xy[i][0]}, {tables_xy[i][1]}, 0, 0, 0, {observers[i][2]}", "world")

        if i != robot_goal_idx:
            subprocess.Popen(["roslaunch", "legibot", "spawn_observer.launch", f"x:={observers[i][0]}",
                              f"y:={observers[i][1]}", f"yaw:={math.radians(observers[i][2]) + math.pi/2}"])
        else:
            gazebo_spawn_static_model(f"person__{i}", person_standing_sdf,
                                      f"{observers[i][0]}, {observers[i][1]}, 0.16, 0, 0, {math.radians(observers[i][2])+ math.pi/2}", "world")
        time.sleep(0.2)

    for ii, table in enumerate(clutter_tables):
        gazebo_spawn_static_model(f"clutter_cyl__{ii}", clutter_cylinder_sdf,
                                  f"{table[0]}, {table[1]}, 0, 0, 0, 0", "world")

    # restart observer node
    record = True
    subprocess.Popen(["rosnode", "kill", "/record_observer"])
    if record:
        time.sleep(0.2)
        subprocess.Popen(["rosrun", "legibot", "record_observer.py"])

    static_map = StaticMap()
    static_map.tables = tables_xy
    static_map.persons = [obs[:2] for obs in observers]
    static_map.update()

    verbose = True
    legibile = False
    planner = MainPlanner([robot_goal] + other_goals, goal_idx=0, robot_xyt0=robot_x0, enable_legibility=legibile, verbose=verbose,
                          w_smoothness=0.12, w_speed=0.8, w_obstacle=0.16, w_fov=1, w_legibility=0.9)

    exp_name = f"exp_{'legible' * legibile}_{'illegible' * (not legibile)}_{time.strftime('%Y-%m-%d_%H-%M-%S')}"
    rospy.set_param("/legibot/experiment_name", exp_name)

    planner.generate_trajectory()
    planner.exec_loop()

if __name__ == '__main__':
    main()
