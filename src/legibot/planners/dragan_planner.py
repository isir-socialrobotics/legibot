import math
import scipy
import numpy as np
import pandas as pd

from dragan_model import legibility
from legibot.static_map import StaticMap
from legibot.utils.viz_utils import Visualizer

class DragPlanner:
    def __init__(self, goals, obstacles, goal_idx, **kwargs):
        self.goal_idx = goal_idx

        self.goals = pd.DataFrame(
            [[f"goal_{i}", goals[i][0], goals[i][1]] for i in range(len(goals))],
            columns=["name", "x", "y"]
        ).set_index("name")

        self.verbose = kwargs.get("verbose", 0)
        if self.verbose:
            Visualizer().draw_obstacles(obstacles)
            Visualizer().draw_goals(goals)
            Visualizer().draw_goals(StaticMap().observers, color=(0, 255, 240))

        self.obstacles = obstacles

        self.robot_radius = 0.3  # pepper?
        self.goal_radius = 0.5  # m
        self.obstacle_radius = kwargs.get("obstacle_radius", 0.5)  # m (robot should keep this distance from obstacles)
        self.obstacle_radius_big = 3.0  # used to predict obstacles in more distant future, and adapt its plan
        self.num_steps = 10
        self.enable_legibility = True

        self.optimal_speed_mps = kwargs.get("optimal_speed", 1.0)  # m/s (robot should keep this speed)

    def full_plan(self, robot_xyt0):
        start = np.array(robot_xyt0[:2])
        # init_traj, dragan_traj = test_optimize_dragan(start, goals, 0)

        initial_trajectory = pd.DataFrame({
                "time":np.linspace(0, 1, self.num_steps),
                "x":np.linspace(start[0], self.goals.loc[f"goal_{self.goal_idx}", "x"], self.num_steps),
                "y":np.linspace(start[1], self.goals.loc[f"goal_{self.goal_idx}", "y"], self.num_steps),
            }).set_index("time")

        result = scipy.optimize.minimize(self.objective, initial_trajectory[["x", "y"]].to_numpy(), method="L-BFGS-B")
        print(result.fun, result.message)

        legible_trajectory = result.x.reshape((self.num_steps, 2))
        return legible_trajectory

    def objective(self, x):
        x = x.reshape((self.num_steps, 2))
        # x[0, :] = (0, 0)
        # x[-1, :] = (0, 5)

        traj_df = pd.DataFrame({
            "time":np.linspace(0, 1, self.num_steps),
            "x":x[:, 0],
            "y":x[:, 1],
        }).set_index("time")

        leg_score = legibility(traj_df, self.goals)
        leg_score = leg_score.iloc[-1, self.goal_idx]

        # TODO: add smoothing term
        dx = np.diff(x, axis=0)
        # calc angle between each pair of points
        angles = np.arctan2(dx[:, 1], dx[:, 0])
        # penalize large changes in angle
        angle_diff = np.diff(angles)
        angle_diff = np.abs((angle_diff + np.pi) % (2 * np.pi) - np.pi)
        score = 0.003 * np.sum(angle_diff) / self.num_steps + leg_score

        return -score

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # plt.plot(initial_trajectory["x"], initial_trajectory["y"], label="initial")
    # plt.plot(legible_trajectory[:, 0], legible_trajectory[:, 1], label="legible")
    # plt.scatter(goals["x"], goals["y"], label="goals")
    # plt.legend()
    # plt.show()

    pass

