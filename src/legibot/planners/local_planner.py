import math
import os
from datetime import datetime
import numpy as np
from typing import Tuple
#from concurrent.futures import ThreadPoolExecutor
from sklearn.metrics import pairwise_distances

from legibot.static_map import StaticMap
from legibot.utils.basic_math import cos_2_vecs, norm
from legibot.utils.viz_utils import Visualizer


class LocalPlanner:
    def __init__(self, goals, obstacles, goal_idx, **kwargs):
        self.all_goals_xyt = goals
        self.goal_idx = goal_idx
        self.goal_radius = 0.7  # m
        self.obstacles = obstacles
        self.enable_vis = kwargs.get("verbose", False)
        self.observer_fov = kwargs.get("observer_fov", math.radians(80))  # radians

        self.enable_legibility = kwargs.get("enable_legibility", True)
        self.legibility_cost_type = kwargs.get("legibility_cost_type", "cosine")  # ["cosine", "euclidean"]

        if self.enable_vis:
            Visualizer().draw_obstacles(obstacles)
            Visualizer().draw_goals(goals)
            Visualizer().draw_goals(StaticMap().observers, color=(0, 255, 240))

        self.robot_radius = 0.3  # pepper?

        # Planner parameters
        self.optimal_speed_mps = kwargs.get("optimal_speed", 1.0)  # m/s (robot should keep this speed)
        self.obstacle_radius = kwargs.get("obstacle_radius", 0.8)  # m (robot should keep this distance from obstacles)
        self.W = {"goal": kwargs.get("w_goal", 0.9),
                  "obstacle": kwargs.get("w_obstacle", 0.3),
                  "speed": kwargs.get("w_speed", 0.6),
                  "smoothness": kwargs.get("w_smoothness", 0.2),
                  "legibility": kwargs.get("w_legibility", 0.8),
                  "fov": kwargs.get("w_fov", 4),
                  }
        self.n_steps = kwargs.get("n_steps", 3)

        self.out_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../out"))

    def _cost_obstacle(self, xy, g=None):  # xy can be a point or a batch of points
        xy = xy.reshape(-1, 2)
        Dx = pairwise_distances(xy[:, 0].reshape(-1, 1), self.obstacles[:, 0].reshape(-1, 1))
        Dy = pairwise_distances(xy[:, 1].reshape(-1, 1), self.obstacles[:, 1].reshape(-1, 1))
        D = np.sqrt(np.square(Dx) + np.square(Dy)) - self.obstacles[:, 2] - self.robot_radius
        D = np.min(D, axis=1)  # min distance from each point to any obstacle
        if g is not None:  # as we get closer to the goal, obstacles are less important (GNRON problem: Goal Nonreachable with Obstacles Nearby)
            D = D / norm(xy - g[:2], axis=1)
        D = np.clip(D, 1e-3, 1000)
        cost_matrix = 0.2 / D
        cost_matrix[cost_matrix < 0.2/self.obstacle_radius] = 0
        return cost_matrix

    def _cost_task(self, cur_xyt, vel_twist, dt, goal_xy) -> Tuple[float]:
        goal_vec = goal_xy - cur_xyt[:2]
        # next_xy = cur_xyt + displacement_vec * dt

        cur_theta = cur_xyt[2]
        new_theta = cur_theta + vel_twist[1] * dt
        next_xy = cur_xyt[:2] + np.array([np.cos(new_theta), np.sin(new_theta)]) * vel_twist[0] * dt

        displacement_vec = next_xy - cur_xyt[:2]

        # cost of deviating from goal direction
        cost_goal = 1 - cos_2_vecs(displacement_vec, goal_vec)
        cost_goal += norm(goal_xy - next_xy) / norm(goal_xy - cur_xyt[:2])  # this is not a good idea

        cost_obs = self._cost_obstacle(next_xy, goal_xy)

        cost_turn = np.abs(vel_twist[1])

        # cost of deviating from optimal speed
        cost_speed = (norm(displacement_vec) - self.optimal_speed_mps) ** 2

        return cost_goal * self.W["goal"], cost_obs * self.W["obstacle"], cost_speed * self.W["speed"], cost_turn * self.W["smoothness"]

    def _cost_task_batch(self, cur_xyt, vel_twist_batch, dt, goal_xy):
        goal_vec = goal_xy[:2] - cur_xyt[:2]

        cur_theta = cur_xyt[2]
        new_theta_batch = cur_theta + vel_twist_batch[:, 1] * dt
        next_x_batch = cur_xyt[0] + np.cos(new_theta_batch) * vel_twist_batch[:, 0] * dt
        next_y_batch = cur_xyt[1] + np.sin(new_theta_batch) * vel_twist_batch[:, 0] * dt
        next_xy_batch = np.stack([next_x_batch, next_y_batch], axis=-1)

        displacement_vec_batch = next_xy_batch - cur_xyt[:2]

        # cost of deviating from goal direction
        cost_goal_batch = 1 - displacement_vec_batch @ goal_vec / (norm(displacement_vec_batch, axis=1) * norm(goal_vec) + 1e-6)

        cost_obs_batch = self._cost_obstacle(next_xy_batch, goal_xy)
        cost_turn_batch = np.abs(vel_twist_batch[:, 1])

        # cost of deviating from optimal speed
        cost_speed_batch = (norm(displacement_vec_batch, axis=1) - self.optimal_speed_mps) ** 2

        return cost_goal_batch * self.W["goal"], cost_obs_batch * self.W["obstacle"], cost_speed_batch * self.W["speed"], cost_turn_batch * self.W["smoothness"]

    def _cost_legibility(self, cur_xyt, vel_twist_batch, dt, goal_xyt, dxy_other_goals):
        vel_twist_batch = np.reshape(vel_twist_batch, (-1, 2))
        new_theta_batch = cur_xyt[2] + vel_twist_batch[:, 1] * dt
        dxy = (np.array([np.cos(new_theta_batch), np.sin(new_theta_batch)]) * vel_twist_batch[:, 0]).T * dt

        next_x_batch = cur_xyt[0] + np.cos(new_theta_batch) * vel_twist_batch[:, 0] * dt
        next_y_batch = cur_xyt[1] + np.sin(new_theta_batch) * vel_twist_batch[:, 0] * dt
        next_xy_batch = np.stack([next_x_batch, next_y_batch], axis=-1)

        dist_to_main_goal = norm(goal_xyt[:2] - cur_xyt[:2])
        other_goals = [self.all_goals_xyt[i] for i in range(len(self.all_goals_xyt)) if i != self.goal_idx]
        ratio_goal_dist = [0] * len(other_goals)
        for ii in range(len(other_goals)):
            dist_to_other_goal_i = norm(other_goals[ii][:2] - cur_xyt[:2])
            ratio_goal_dist[ii] = dist_to_main_goal / dist_to_other_goal_i
        # print(f"max (ratio_goal_dist): {max(ratio_goal_dist):.2f}")

        if self.legibility_cost_type.lower() == "cosine":
            costs = dxy @ dxy_other_goals.T / (norm(dxy, axis=1).reshape(-1, 1) * norm(dxy_other_goals, axis=1).reshape(1, -1) + 1e-6)

        elif self.legibility_cost_type.lower() == "euclidean":
            next_xy_other_goals = cur_xyt[:2] + dxy_other_goals
            D = pairwise_distances(next_xy_batch, next_xy_other_goals)
            costs = 1 / (D + 1e-6)
        else:
            raise ValueError(f"Unknown legibility cost type: {self.legibility_cost_type}")

        # apply weights for each non-main goal
        legib_costs_weighted = costs @ np.array(ratio_goal_dist).reshape(-1, 1) / len(dxy_other_goals)

        # cost of being out of the main observer's field of view
        observer_unit_vec = np.array([np.cos(goal_xyt[2]), np.sin(goal_xyt[2])])
        dxy_goal_normal_for_next_xy_batch = (next_xy_batch - goal_xyt[:2]) / (
                    norm(next_xy_batch - goal_xyt[:2], axis=1).reshape(-1, 1) + 1e-6)
        deviation_from_observer_center_of_view = np.abs(
            np.arccos(dxy_goal_normal_for_next_xy_batch @ observer_unit_vec))
        fov_cost = np.clip((2 * deviation_from_observer_center_of_view / self.observer_fov) - 1, 0, 10)

        return (legib_costs_weighted * self.W["legibility"] + fov_cost.reshape(-1, 1) * self.W["fov"]).reshape(-1)

    def __search_optimal_velocity__(self, xyt, dt, goal, illegible_v_stars=[]):
        ang_speed_range = np.linspace(-np.pi/2, np.pi/2, 36)  # 10 deg
        lin_speed_range = np.linspace(0, self.optimal_speed_mps, 11)
        speed_table = np.meshgrid(lin_speed_range, ang_speed_range)
        cost_map = np.zeros((len(ang_speed_range), len(lin_speed_range)))

        ## Parallel version
        # def calculate_cost(ij_tuple):
        #     i, j = ij_tuple
        #     lin_speed = speed_table[0][i, j]
        #     ang_speed = speed_table[1][i, j]
        #
        #     costs = self._cost_task(xyt, (lin_speed, ang_speed), dt, goal)
        #     cost_i = np.sum(costs)
        #     if len(illegible_v_stars) > 0 and self.enable_legibility:
        #         cost_i += self._cost_legibility(xyt, (lin_speed, ang_speed), dt, goal, illegible_v_stars)
        #     cost_i = np.clip(cost_i, 0, 1000)
        #     cost_map[i, j] = cost_i
        # with ThreadPoolExecutor() as executor:
        #     executor.map(calculate_cost, np.ndindex(speed_table[0].shape))

        ## Serial version
        # for i, j in np.ndindex(speed_table[0].shape):
        #     calculate_cost((i, j))

        ## Batch version
        costs_batch = self._cost_task_batch(xyt, np.stack(speed_table, axis=-1).reshape(-1, 2), dt, goal)
        cost_batch = np.sum(costs_batch, axis=0)
        if len(illegible_v_stars) > 0 and self.enable_legibility:
            cost_batch += self._cost_legibility(xyt, np.stack(speed_table, axis=-1).reshape(-1, 2), dt, goal, illegible_v_stars)
        min_cost_idx = np.argmin(cost_batch)
        twist_star_vw = np.array([speed_table[0].flatten()[min_cost_idx], speed_table[1].flatten()[min_cost_idx]])

        return twist_star_vw, 0

    def step(self, xyt, dt) -> (np.ndarray, np.ndarray, bool):
        if norm(xyt[:2] - self.all_goals_xyt[self.goal_idx][:2]) < (self.goal_radius + self.robot_radius + dt * self.optimal_speed_mps):
            return [self.all_goals_xyt[self.goal_idx][0], self.all_goals_xyt[self.goal_idx][1], xyt[2]], True

        optimal_plan_other_goals = []
        other_goals = [self.all_goals_xyt[i] for i in range(len(self.all_goals_xyt)) if i != self.goal_idx]
        if self.enable_legibility:
            # find optimal local-plan for each potential goal
            for g_idx, goal in enumerate(other_goals):
                last_xyt = xyt
                optimal_plan_goal_i = []
                for step in range(self.n_steps):
                    vw_star_other, _ = self.__search_optimal_velocity__(last_xyt, dt, goal)
                    new_theta = last_xyt[2] + vw_star_other[1] * dt
                    new_xy = last_xyt[:2] + np.array([np.cos(new_theta), np.sin(new_theta)]) * vw_star_other[0] * dt
                    if self.enable_vis:
                        Visualizer().add_arrow(last_xyt[:2], new_xy, color=(0, 0, 255))
                        Visualizer().show(20)

                    last_xyt = np.array([new_xy[0], new_xy[1], new_theta])
                    optimal_plan_goal_i.append(last_xyt[:2] - xyt[:2])
                optimal_plan_other_goals.append(optimal_plan_goal_i)

        if len(optimal_plan_other_goals) > 0:
            optimal_plan_other_goals = np.array(optimal_plan_other_goals)
        else:
            optimal_plan_other_goals = np.empty((len(other_goals), self.n_steps, 0))

        sub_plan = [xyt]
        for step in range(self.n_steps):
            vw_star, cost_map = self.__search_optimal_velocity__(xyt, dt, self.all_goals_xyt[self.goal_idx],
                                                                 optimal_plan_other_goals[:, step])
            cur_theta = xyt[2]
            new_theta = cur_theta + vw_star[1] * dt
            next_xy = xyt[:2] + np.array([np.cos(new_theta), np.sin(new_theta)]) * vw_star[0] * dt

            sub_plan.append((next_xy[0], next_xy[1], new_theta))

        return sub_plan[1], False

    def full_plan(self, xyt0, dt, H=100):
        assert len(xyt0) == 3, "local_planner: x0 must be a 3D vector: [x, y, theta(rad)]"
        xyt = xyt0
        plan = [xyt0]
        illegible_vectors = []

        now = datetime.now()
        for t in np.arange(0, H * dt, dt):
            new_xyt, reached = self.step(xyt, dt)
            plan.append(new_xyt)

            if self.enable_vis:
                Visualizer().add_arrow(xyt, new_xyt, color=(255, 0, 0))
                Visualizer().save(os.path.join(self.out_dir, f"{now.strftime('%Y%m%d-%H%M%S')}-{round(t, 4):.4f}.png"))

            xyt = new_xyt
            if reached:
                break
            # print(f"time passed: {datetime.now() - now}")

        # plan.append([self.all_goals[self.goal_idx][0], self.all_goals[self.goal_idx][1], xyt[2]])
        if self.enable_vis:
            Visualizer().draw_path(plan)
            Visualizer().show(delay=100)
        return plan

