import os
from datetime import datetime
import numpy as np

from legibot.utils.viz_utils import Visualizer


class LocalPlanner:
    def __init__(self, goals, obstacles, goal_idx, **kwargs):
        self.goals = goals
        self.goal_idx = goal_idx
        self.obstacles = obstacles

        # DWA parameters
        self.optimal_speed_mps = kwargs.get("optimal_speed", 2.0)  # m/s (robot should keep this speed)
        self.obstacle_radius = kwargs.get("obstacle_radius", 0.4)  # m (robot should keep this distance from obstacles)
        self.W = {"goal": kwargs.get("w_goal", 0.8),
                  "obstacle": kwargs.get("w_obstacle", 0.08),
                  "speed": kwargs.get("w_speed", 1),
                  "legibility": kwargs.get("w_legibility", 0.5)}
        self.n_steps = kwargs.get("n_steps", 3)

        self.out_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../out"))

    def cost_task(self, pos, vel, dt, goal_xy):
        goal_vec = goal_xy - pos
        next_xy = pos + vel * dt

        # cost of deviating from goal direction
        cost_goal = 1-np.dot(goal_vec, vel) /(np.linalg.norm(goal_vec) * np.linalg.norm(vel) + 1e-6)

        min_dist = np.inf
        for obstacle in self.obstacles:
            dist = np.linalg.norm(next_xy - obstacle[:2]) - obstacle[2]
            if dist < 0:
                min_dist = 0
                break
            if dist < min_dist and dist < self.obstacle_radius:
                min_dist = dist
        # cost of getting too close to obstacles
        cost_obstacle = 1/(min_dist + 1e-3)

        # cost of deviating from optimal speed
        cost_speed = (np.linalg.norm(vel) - self.optimal_speed_mps) ** 2

        return cost_goal * self.W["goal"] + cost_obstacle * self.W["obstacle"] + cost_speed * self.W["speed"]

    def cost_legibility(self, pos, vel, dt, goal_xy, illegible_v_stars):
        cost = 0
        for v_star in illegible_v_stars:
            cost += np.dot(vel, v_star) / (np.linalg.norm(vel) * np.linalg.norm(v_star) + 1e-6)
        return cost / len(illegible_v_stars)

    def __search_optimal_velocity__(self, x, dt, goal, illegible_v_stars=[]):
        cost_map = []
        min_cost = np.inf
        v_star = np.zeros(2)
        for theta in np.linspace(0, 2 * np.pi, 72):
            for speed in np.linspace(0, self.optimal_speed_mps, 10):
                vel = np.array([np.cos(theta), np.sin(theta)]) * speed
                cost = self.cost_task(x, vel, dt, goal)
                if len(illegible_v_stars) > 0:
                    cost += self.cost_legibility(x, vel, dt, goal, illegible_v_stars) * self.W["legibility"]
                cost = np.clip(cost, 0, 1000)
                if cost > min_cost:
                    continue
                min_cost = cost
                v_star = vel
        return v_star, cost_map

    def step(self, x, dt):
        # find optimal velocity for each potential goal
        other_goals = [self.goals[i] for i in range(len(self.goals)) if i != self.goal_idx]
        optimal_plan_other_goals = []
        for goal in other_goals:
            x_last = x
            optimal_plan_goal_i = [x]
            for step in range(self.n_steps):
                v_star_other, cost_map = self.__search_optimal_velocity__(x_last, dt, goal)
                Visualizer().add_arrow(x_last, x_last + v_star_other * dt, color=(0, 0, 255))
                # Visualizer().show(0)
                x_last = x_last + v_star_other * dt
                optimal_plan_goal_i.append(x_last)
            optimal_plan_other_goals.append(optimal_plan_goal_i)
        optimal_plan_other_goals = np.array(optimal_plan_other_goals)

        x_last = x
        for step in range(1):
            v_star, cost_map = self.__search_optimal_velocity__(x, dt, self.goals[self.goal_idx],
                                                                optimal_plan_other_goals[:, step+1]-optimal_plan_other_goals[:, step])
            x_last = x_last + v_star * dt

        return x_last, cost_map

    def get_plan(self, x0, dt=0.05, H=100):
        x = x0
        plan = [x0]
        illegible_vectors = []

        now = datetime.now()
        for t in np.arange(0, H * dt, dt):
            new_x, cost_map = self.step(x, dt)
            plan.append(new_x)

            Visualizer().add_arrow(x, new_x, color=(255, 0, 0))
            Visualizer().show(delay=100)
            Visualizer().save(os.path.join(self.out_dir, f"{now.strftime('%Y%m%d-%H%M%S')}-{round(t, 4):.4f}.png"))

            x = new_x
            if np.linalg.norm(x - self.goals[self.goal_idx]) < 0.1:
                break

        plan.append(self.goals[self.goal_idx])
        return plan

