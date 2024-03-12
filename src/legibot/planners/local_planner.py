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
        self.obstacles = obstacles
        self.verbose = kwargs.get("verbose", 0)
        self.observer_fov = kwargs.get("observer_fov", math.radians(120))  # radians

        self.enable_legibility = kwargs.get("enable_legibility", True)
        self.legibility_cost_type = kwargs.get("legibility_cost_type", "cosine")  # ["cosine", "euclidean"]

        if self.verbose:
            Visualizer().reset()
            Visualizer().draw_obstacles(obstacles)
            Visualizer().draw_goals(goals)
            # Visualizer().draw_goals(StaticMap().observers, color=(0, 255, 240))

        self.robot_radius = 0.3  # pepper?
        self.goal_radius = 0.5  # m
        self.obstacle_radius = kwargs.get("obstacle_radius", 0.5)  # m (robot should keep this distance from obstacles)
        self.obstacle_radius_big = 3.0  # used to predict obstacles in more distant future, and adapt its plan

        self.optimal_speed_mps = kwargs.get("optimal_speed", 1.0)  # m/s (robot should keep this speed)

        # Planner parameters
        self.W = {"goal": kwargs.get("w_goal", 0.9),
                  "obstacle": kwargs.get("w_obstacle", 0.4),
                  "obstacle_grad": kwargs.get("w_obstacle_grad", 0.4),
                  "speed": kwargs.get("w_speed", 0.6),
                  "smoothness": kwargs.get("w_smoothness", 0.1),
                  "legibility": kwargs.get("w_legibility", 1.),
                  "fov": kwargs.get("w_fov", 1), # applies only when legibility is enabled
                  }
        self.n_steps = kwargs.get("n_steps", 4)

        self.out_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../out_March"))

    def _cost_obstacle(self, cur_xy, next_xy, goal_xy=None):  # xy can be a point or a batch of points
        next_xy = next_xy.reshape(-1, 2)
        Dx = pairwise_distances(next_xy[:, 0].reshape(-1, 1), self.obstacles[:, 0].reshape(-1, 1))
        Dy = pairwise_distances(next_xy[:, 1].reshape(-1, 1), self.obstacles[:, 1].reshape(-1, 1))
        D_center2center = np.sqrt(np.square(Dx) + np.square(Dy))
        D = D_center2center - self.obstacles[:, 2] - self.robot_radius

        # Handle GNRON problem (Goal Nonreachable with Obstacles Nearby)
        if goal_xy is not None:
            w_dist_to_goal = 1 / norm(next_xy - goal_xy[:2], axis=1).reshape(-1, 1)
            # OCP (Obstacle Closest Point)
            OCP_x = self.obstacles[:, 0] + Dx * (self.obstacles[:, 2] + self.robot_radius )/ D_center2center
            OCP_y = self.obstacles[:, 1] + Dy * (self.obstacles[:, 2] + self.robot_radius )/ D_center2center
            # ignore obstacles within the goal radius
            D[np.sqrt(np.square(OCP_x - goal_xy[0]) + np.square(OCP_y - goal_xy[1])) < self.goal_radius] = 1000
        else:
            w_dist_to_goal = 1

        d_cur = cur_xy[:2] - self.obstacles[:, :2]
        grad_dist_to_obs = D_center2center - np.sqrt(np.square(d_cur[:, 0]) + np.square(d_cur[:, 1]))
        grad_dist_to_obs[grad_dist_to_obs > 0] = 0
        grad_dist_to_obs[D > self.obstacle_radius_big] = 0

        # min distance from each point to any obstacle
        D = np.min(D, axis=1)
        grad_dist_to_obs = np.min(grad_dist_to_obs, axis=1)

        cost_too_close = 1 - np.exp(D)
        cost_too_close[D > self.obstacle_radius] = 0
        cost_too_close[D < self.obstacle_radius / 5] = 10

        cost_grad = 1 - np.exp(-np.abs(grad_dist_to_obs))

        cost_matrix = cost_too_close * self.W["obstacle"] + cost_grad * self.W["obstacle_grad"]
        return cost_matrix

    def _cost_task(self, cur_xyt, vel_twist_batch, dt, goal_xy):
        goal_vec = goal_xy[:2] - cur_xyt[:2]

        cur_theta = cur_xyt[2]
        new_theta_batch = cur_theta + vel_twist_batch[:, 1] * dt
        next_x_batch = cur_xyt[0] + np.cos(new_theta_batch) * vel_twist_batch[:, 0] * dt
        next_y_batch = cur_xyt[1] + np.sin(new_theta_batch) * vel_twist_batch[:, 0] * dt
        next_xy_batch = np.stack([next_x_batch, next_y_batch], axis=-1)

        displacement_vec_batch = next_xy_batch - cur_xyt[:2]

        # cost of deviating from goal direction
        cost_goal_batch = 1 - displacement_vec_batch @ goal_vec / (norm(displacement_vec_batch, axis=1) * norm(goal_vec) + 1e-9)

        cost_obs_batch = self._cost_obstacle(cur_xyt, next_xy_batch, goal_xy)
        cost_turn_batch = np.abs(vel_twist_batch[:, 1])

        # cost of deviating from optimal speed
        cost_speed_batch = np.abs(norm(displacement_vec_batch, axis=1) - self.optimal_speed_mps) # ** 2

        return (cost_goal_batch * self.W["goal"], cost_obs_batch,
                cost_speed_batch * self.W["speed"], cost_turn_batch * self.W["smoothness"])

    def _cost_legibility(self, cur_xyt, vel_twist_batch, dt, goal_xyt, dxy_all_goals):
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
            dxy_actual_goal = dxy_all_goals[self.goal_idx]
            dxy_other_goals = np.array([dxy_all_goals[i] for i in range(len(dxy_all_goals)) if i != self.goal_idx])
            deviate_actual_goal = (dxy @ dxy_actual_goal / (norm(dxy, axis=1) * norm(dxy_actual_goal) + 1e-9)).reshape(-1, 1)
            deviate_other_goals = dxy @ dxy_other_goals.T / (norm(dxy, axis=1).reshape(-1, 1) * norm(dxy_other_goals, axis=1).reshape(1, -1) + 1e-9)
            costs = np.pi - np.clip(np.arccos(deviate_other_goals), 0, np.pi) \
                    + np.clip(np.arccos(deviate_actual_goal), 0, np.pi) / 2

        elif self.legibility_cost_type.lower() == "euclidean":
            next_xy_other_goals = cur_xyt[:2] + dxy_all_goals
            D = pairwise_distances(next_xy_batch, next_xy_other_goals)
            costs = 1 / (D + 1e-9)
        else:
            raise ValueError(f"Unknown legibility cost type: {self.legibility_cost_type}")

        # cost of being out of the main observer's field of view
        observer_unit_vec = np.array([np.cos(goal_xyt[2]), np.sin(goal_xyt[2])])
        dxy_goal_normal_for_next_xy_batch = (next_xy_batch - goal_xyt[:2]) / (
                    norm(next_xy_batch - goal_xyt[:2], axis=1).reshape(-1, 1) + 1e-9)
        deviation_from_observer_center_of_view = np.abs(
            np.arccos(dxy_goal_normal_for_next_xy_batch @ observer_unit_vec))  # radians
        fov_cost = 1/(1+np.exp(-5*(deviation_from_observer_center_of_view / (self.observer_fov/2) - 1)))
        observability = np.clip(1 - deviation_from_observer_center_of_view / (self.observer_fov/2), 0, 1)
        observability[observability > 0] = 1
        # print(f"min (observability): {min(observability):.2f} max (observability): {max(observability):.2f}")
        # print(f"min (dev): {min(deviation_from_observer_center_of_view):.2f} max (dev): {max(deviation_from_observer_center_of_view):.2f}")
        # print(f"min (fov_cost): {min(fov_cost):.2f} max (fov_cost): {max(fov_cost):.2f}")

        # apply weights for each non-main goal
        legib_costs_weighted = costs @ np.array(ratio_goal_dist).reshape(-1, 1) * observability.reshape(-1, 1) / len(dxy_all_goals)

        return (legib_costs_weighted * self.W["legibility"] + fov_cost.reshape(-1, 1) * self.W["fov"]).reshape(-1)

    def _search_optimal_velocity(self, xyt, dt, goal, suboptimal_v_stars=[]):
        ang_speed_range = np.linspace(-np.pi/2, np.pi/2, 72) / dt # 10 deg
        lin_speed_range = np.linspace(0.05, self.optimal_speed_mps, 10)
        speed_table = np.meshgrid(lin_speed_range, ang_speed_range)

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
        costs_batch = self._cost_task(xyt, np.stack(speed_table, axis=-1).reshape(-1, 2), dt, goal)
        cost_batch = np.sum(costs_batch, axis=0)
        if len(suboptimal_v_stars) > 0 and self.enable_legibility:
            cost_legib_batch = self._cost_legibility(xyt, np.stack(speed_table, axis=-1).reshape(-1, 2), dt, goal, suboptimal_v_stars)
            cost_batch += cost_legib_batch
        min_cost_idx = np.argmin(cost_batch)
        twist_star_vw = np.array([speed_table[0].flatten()[min_cost_idx], speed_table[1].flatten()[min_cost_idx]])

        angle_range = [xyt[2] + ang_speed_range[0] * dt, xyt[2] + ang_speed_range[-1] * dt]
        radius_range = [lin_speed_range[0] * dt, lin_speed_range[-1] * dt]
        if self.verbose > 1:
            if len(suboptimal_v_stars) > 0 and self.enable_legibility:
                # Visualizer().draw_heatmap(xyt[:2], cost_legib_batch.reshape(len(ang_speed_range), len(lin_speed_range)), radius_range, angle_range)
                pass
            # Visualizer().draw_heatmap(xyt[:2], costs_batch[1].reshape(len(ang_speed_range), len(lin_speed_range)), radius_range, angle_range, title="cost_obstacle")
            Visualizer().draw_heatmap(xyt[:2], cost_batch.reshape(len(ang_speed_range), len(lin_speed_range)), radius_range, angle_range, title="overall_cost")


        # if len(illegible_v_stars) > 0 and self.enable_legibility:
        #     print("legib cost: ", cost_legib_batch[min_cost_idx])

        return twist_star_vw, cost_batch

    def step(self, xyt, dt) -> (np.ndarray, np.ndarray, bool):
        if norm(xyt[:2] - self.all_goals_xyt[self.goal_idx][:2]) < (self.goal_radius + self.robot_radius + dt * self.optimal_speed_mps):
            return [xyt, [self.all_goals_xyt[self.goal_idx][0],
                          self.all_goals_xyt[self.goal_idx][1], xyt[2]]
                    ], True

        suboptimal_plan_all_goals = []
        other_goals = [self.all_goals_xyt[i] for i in range(len(self.all_goals_xyt)) if i != self.goal_idx]
        if self.enable_legibility:
            # find optimal local-plan for each potential goal
            for g_idx, goal in enumerate(self.all_goals_xyt):
                last_xyt = xyt
                optimal_plan_goal_i = []
                for step in range(self.n_steps):
                    vw_star_other, _ = self._search_optimal_velocity(last_xyt, dt, goal)
                    new_theta = last_xyt[2] + vw_star_other[1] * dt
                    new_xy = last_xyt[:2] + np.array([np.cos(new_theta), np.sin(new_theta)]) * vw_star_other[0] * dt
                    local_path_color = (0, 155, 0) if g_idx == self.goal_idx else (255, 0, 0)
                    if self.verbose:
                        Visualizer().add_arrow(last_xyt[:2], new_xy, color_rgb=local_path_color)
                        # Visualizer().show(20)

                    last_xyt = np.array([new_xy[0], new_xy[1], new_theta])
                    optimal_plan_goal_i.append(last_xyt[:2] - xyt[:2])
                suboptimal_plan_all_goals.append(optimal_plan_goal_i)

        if len(suboptimal_plan_all_goals) > 0:
            suboptimal_plan_all_goals = np.array(suboptimal_plan_all_goals)
        else:
            suboptimal_plan_all_goals = np.empty((len(other_goals), self.n_steps, 0))

        sub_plan = [xyt]
        for step in range(self.n_steps):
            vw_star, cost_map = self._search_optimal_velocity(sub_plan[-1], dt, self.all_goals_xyt[self.goal_idx],
                                                                 suboptimal_plan_all_goals[:, step])
            cur_theta = sub_plan[-1][2]
            new_theta = cur_theta + vw_star[1] * dt
            next_xy = sub_plan[-1][:2] + np.array([np.cos(new_theta), np.sin(new_theta)]) * vw_star[0] * dt

            sub_plan.append((next_xy[0], next_xy[1], new_theta))

        return sub_plan, False

    def full_plan(self, xyt0, dt, H=200):
        assert len(xyt0) == 3, "local_planner: x0 must be a 3D vector: [x, y, theta(rad)]"
        xyt = xyt0
        plan = [xyt0]
        illegible_vectors = []

        now = datetime.now()
        for t in np.arange(0, H * dt, dt):
            Visualizer().reset()
            # print("step: ", t, end=" ")
            new_xyts, reached = self.step(xyt, dt)
            new_xyt = new_xyts[1]
            plan.append(new_xyt)

            if self.verbose > 1:
                Visualizer().draw_obstacles(self.obstacles)
                Visualizer().draw_goals(self.all_goals_xyt, color=(200, 0, 200), with_text=True)
                Visualizer().draw_goals([self.all_goals_xyt[self.goal_idx]], color=(200, 0, 200), edge_color=(0.1, 0, 0), with_text=False)
                # Visualizer().draw_goals(StaticMap().observers, color=(0, 255, 240))
                Visualizer().draw_path(plan, color_rgb=(0, 0, 255))


                # if t == 0:
                Visualizer().draw_initial_point(xyt0)
                for ii in range(len(new_xyts) - 1):
                    Visualizer().add_arrow(new_xyts[ii], new_xyts[ii + 1], color_rgb=(0, 0, 255))

                Visualizer().save(os.path.join(self.out_dir, f"{now.strftime('%Y%m%d-%H%M%S')}-{round(t, 4):.4f}.png"))
                # print(f"fig saved to ", {os.path.join(self.out_dir, f'{now.strftime("%Y%m%d-%H%M%S")}-{round(t, 4):.4f}.png')})

            xyt = new_xyt
            if reached:
                break
            # print(f"time passed: {datetime.now() - now}")

        # plan.append([self.all_goals[self.goal_idx][0], self.all_goals[self.goal_idx][1], xyt[2]])
        if self.verbose:
            Visualizer().draw_path(plan)
            Visualizer().show(delay=100)
        return plan

