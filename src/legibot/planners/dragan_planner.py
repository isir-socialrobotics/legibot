import math

import scipy

from dragan_model import legibility
from legibot.static_map import StaticMap
from legibot.utils.viz_utils import Visualizer
from tests.test_dragan import test_optimize_dragan
import numpy as np
import pandas as pd

class DragPlanner:
    def __init__(self, goals, obstacles, goal_idx, **kwargs):
        self.all_goals_xyt = goals
        self.goal_idx = goal_idx
        self.obstacles = obstacles
        self.verbose = kwargs.get("verbose", 0)

        if self.verbose:
            Visualizer().draw_obstacles(obstacles)
            Visualizer().draw_goals(goals)
            Visualizer().draw_goals(StaticMap().observers, color=(0, 255, 240))

        self.robot_radius = 0.3  # pepper?
        self.goal_radius = 0.5  # m
        self.obstacle_radius = kwargs.get("obstacle_radius", 0.5)  # m (robot should keep this distance from obstacles)
        self.obstacle_radius_big = 3.0  # used to predict obstacles in more distant future, and adapt its plan

        self.optimal_speed_mps = kwargs.get("optimal_speed", 1.0)  # m/s (robot should keep this speed)

    def full_plan(self, robot_xyt0, dt, H):
        start = np.array(robot_xyt0[:2])
        goals = pd.DataFrame(
            [[f"goal_{i}", self.all_goals_xyt[i][0], self.all_goals_xyt[i][1]] for i in range(len(self.all_goals_xyt))],
            columns=["name", "x", "y"]
        ).set_index("name")

        num_steps = 10
        # init_traj, dragan_traj = test_optimize_dragan(start, goals, 0)

        initial_trajectory = pd.DataFrame({
                "time":np.linspace(0, 1, num_steps),
                "x":np.linspace(start[0], goals.loc["goal_0", "x"], num_steps),
                "y":np.linspace(start[1], goals.loc["goal_0", "y"], num_steps),
            }).set_index("time")

    def objective(self, x):
        x = x.reshape((num_steps, 2))
        x[0, :] = (0, 0)
        x[-1, :] = (0, 5)

        df = pd.DataFrame({
            "time":np.linspace(0, 1, num_steps),
            "x":x[:, 0],
            "y":x[:, 1],
        }).set_index("time")

        score = legibility(df, goals)
        # TODO: add smoothing term
        dx = np.diff(x, axis=0)
        # calc angle between each pair of points
        angles = np.arctan2(dx[:, 1], dx[:, 0])
        # penalize large changes in angle
        angle_diff = np.diff(angles)
        angle_diff = np.abs((angle_diff + np.pi) % (2 * np.pi) - np.pi)
        score -= 0.003 * np.sum(angle_diff)

        return -score.goal_0.iloc[-1]

if __name__ == "__main__":

    result = scipy.optimize.minimize(objective, initial_trajectory[["x", "y"]].to_numpy(), method="L-BFGS-B")
    print(result.fun, result.message)

    legible_trajectory = result.x.reshape((num_steps, 2))

    import matplotlib.pyplot as plt
    plt.plot(initial_trajectory["x"], initial_trajectory["y"], label="initial")
    plt.plot(legible_trajectory[:, 0], legible_trajectory[:, 1], label="legible")
    plt.scatter(goals["x"], goals["y"], label="goals")
    plt.legend()
    plt.show()
