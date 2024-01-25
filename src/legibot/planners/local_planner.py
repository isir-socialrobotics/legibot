import os
from datetime import datetime
import numpy as np

from legibot.utils.basic_math import cos_2_vecs
from legibot.utils.viz_utils import Visualizer


class LocalPlanner:
    def __init__(self, goals, obstacles, goal_idx, **kwargs):
        self.all_goals = goals
        self.goal_idx = goal_idx
        self.obstacles = obstacles
        self.enable_vis = kwargs.get("verbose", False)
        self.enable_legibility = kwargs.get("enable_legibility", False)

        if self.enable_vis:
            Visualizer().draw_obstacles(obstacles)
            Visualizer().draw_goals(goals)

        self.robot_radius = 0.3  # pepper?

        # Planner parameters
        self.optimal_speed_mps = kwargs.get("optimal_speed", 2.0)  # m/s (robot should keep this speed)
        self.obstacle_radius = kwargs.get("obstacle_radius", 3.0)  # m (robot should keep this distance from obstacles)
        self.W = {"goal": kwargs.get("w_goal", 0.9),
                  "obstacle": kwargs.get("w_obstacle", 0.5),
                  "speed": kwargs.get("w_speed", 0.4),
                  "legibility": kwargs.get("w_legibility", 0.5)}
        self.n_steps = kwargs.get("n_steps", 3)

        self.out_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../out"))

    def _cost_obstacle(self, xy, g=None):
        d = np.sqrt(np.square(self.obstacles[:, 0] - xy[0]) + np.square(self.obstacles[:, 1] - xy[1]))
        d = d - self.obstacles[:, 2] - self.robot_radius
        if g is not None:
            d = d / np.linalg.norm(xy - g)
        d = max(d.min(), 0)
        if d > self.obstacle_radius:
            return 0
        return 0.5/ (d+ 1e-2) # np.exp(-d)

    def _cost_task(self, cur_xyt, vel_twist, dt, goal_xy):
        goal_vec = goal_xy - cur_xyt[:2]
        # next_xy = cur_xyt + displacement_vec * dt

        cur_theta = cur_xyt[2]
        new_theta = cur_theta + vel_twist[1] * dt
        next_xy = cur_xyt[:2] + np.array([np.cos(new_theta), np.sin(new_theta)]) * vel_twist[0] * dt

        displacement_vec = next_xy - cur_xyt[:2]

        # cost of deviating from goal direction
        cost_goal = 1 - cos_2_vecs(displacement_vec, goal_vec)
        cost_goal += np.linalg.norm(goal_xy - next_xy) / np.linalg.norm(goal_xy - cur_xyt[:2])  # this is not a good idea

        cost_obs = self._cost_obstacle(next_xy, goal_xy)

        cost_turn = np.abs(vel_twist[1]) ** 2

        # cost of deviating from optimal speed
        cost_speed = (np.linalg.norm(displacement_vec) - self.optimal_speed_mps) ** 2

        return cost_goal * self.W["goal"] + cost_obs * self.W["obstacle"] + self.W["speed"] * (cost_speed) #+ cost_turn)

    def _cost_legibility(self, cur_xyt, vel_twist, dt, goal_xy, dxy_other_goals):
        new_theta = cur_xyt[2] + vel_twist[1] * dt
        dxy = np.array([np.cos(new_theta), np.sin(new_theta)]) * vel_twist[0] * dt

        cost = 0
        for d_xy_illeg_i in dxy_other_goals:
            # cost += np.dot(vel, d_xy_illeg_i) / (np.linalg.norm(vel) * np.linalg.norm(d_xy_illeg_i) + 1e-6)
            cost += cos_2_vecs(dxy, d_xy_illeg_i)
        return cost / len(dxy_other_goals)

    def __search_optimal_velocity__(self, xyt, dt, goal, illegible_v_stars=[]):
        min_cost = np.inf
        twist_star_vw = np.zeros(2)
        omega_range = np.linspace(-np.pi, np.pi, 36)
        speed_range = np.linspace(0, self.optimal_speed_mps, 10)
        # cost_map = np.zeros((len(omega_range), len(speed_range)))
        cost_map = []
        for ang_speed in omega_range:
            for lin_speed in speed_range:
                # cur_theta = x[2]
                # new_theta = xyt[2] + ang_speed * dt
                # next_xyt = xyt[:2] + np.array([np.cos(new_theta), np.sin(new_theta)]) * lin_speed * dt

                # vel = np.array([np.cos(ang_speed), np.sin(ang_speed)]) * lin_speed
                cost = self._cost_task(xyt, (lin_speed, ang_speed), dt, goal)
                if len(illegible_v_stars) > 0 and self.enable_legibility:
                    cost += self._cost_legibility(xyt, (lin_speed, ang_speed), dt, goal, illegible_v_stars) * self.W["legibility"]
                cost = np.clip(cost, 0, 1000)
                if cost > min_cost:
                    continue
                min_cost = cost
                twist_star_vw = np.array([lin_speed, ang_speed])
        return twist_star_vw, cost_map

    def step(self, xyt, dt) -> (np.ndarray, np.ndarray, bool):
        if np.linalg.norm(xyt[:2] - self.all_goals[self.goal_idx]) < (self.robot_radius + dt * self.optimal_speed_mps):
            return self.all_goals[self.goal_idx], None, True

        optimal_plan_other_goals = []
        other_goals = [self.all_goals[i] for i in range(len(self.all_goals)) if i != self.goal_idx]
        if self.enable_legibility:
            # find optimal velocity for each potential goal
            for goal in other_goals:
                last_xyt = xyt
                optimal_plan_goal_i = []
                for step in range(self.n_steps):
                    vw_star_other, cost_map = self.__search_optimal_velocity__(last_xyt, dt, goal)
                    if self.enable_vis:
                        Visualizer().add_arrow(last_xyt[:2], last_xyt[:2] + vw_star_other * dt, color=(0, 0, 255))
                        # Visualizer().show(2)
                    last_xyt = (*(last_xyt[:2] + vw_star_other * dt), last_xyt[2] + vw_star_other[1] * dt)
                    optimal_plan_goal_i.append(vw_star_other)
                optimal_plan_other_goals.append(optimal_plan_goal_i)

        if len(optimal_plan_other_goals) > 0:
            optimal_plan_other_goals = np.array(optimal_plan_other_goals)
        else:
            optimal_plan_other_goals = np.empty((len(other_goals), self.n_steps, 0))

        sub_plan = [xyt]
        for step in range(3):
            vw_star, cost_map = self.__search_optimal_velocity__(xyt, dt, self.all_goals[self.goal_idx],
                                                                 optimal_plan_other_goals[:, step])
            cur_theta = xyt[2]
            new_theta = cur_theta + vw_star[1] * dt
            next_xy = xyt[:2] + np.array([np.cos(new_theta), np.sin(new_theta)]) * vw_star[0] * dt

            sub_plan.append((next_xy[0], next_xy[1], new_theta))

        return sub_plan[1], cost_map, False

    def full_plan(self, xyt0, dt=0.05, H=100):
        assert len(xyt0) == 3, "local_planner: x0 must be a 3D vector: [x, y, theta(rad)]"
        xyt = xyt0
        plan = [xyt0]
        illegible_vectors = []

        now = datetime.now()
        for t in np.arange(0, H * dt, dt):
            new_xyt, cost_map, reached = self.step(xyt, dt)
            plan.append(new_xyt)

            if self.enable_vis:
                Visualizer().add_arrow(xyt, new_xyt, color=(255, 0, 0))
                Visualizer().save(os.path.join(self.out_dir, f"{now.strftime('%Y%m%d-%H%M%S')}-{round(t, 4):.4f}.png"))

            xyt = new_xyt
            print(f"time passed: {datetime.now() - now}")

            if reached:
                break

        plan.append(self.all_goals[self.goal_idx])
        if self.enable_vis:
            Visualizer().draw_path(plan)
            Visualizer().show(delay=100)
        return plan

