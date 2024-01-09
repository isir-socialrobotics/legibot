# Dynamic Window Approach (DWA) for planning with legibility
import numpy as np


class DWA:
    def __init__(self, goals, obstacles, goal_idx, **kwargs):
        self.goals = goals
        self.goal_idx = goal_idx
        self.obstacles = obstacles
        self.optimal_speed_mps = 2
        self.obstacle_radius = kwargs.get("obstacle_radius", 0.4)
        self.W = {"goal": kwargs.get("w_goal", 1),
                  "obstacle": kwargs.get("w_obstacle", 0.2),
                  "speed": kwargs.get("w_speed", 1)}

    def sample_cost(self, xy0, xy1, dt):
        goal_vec = self.goals[self.goal_idx] - xy0
        new_vec = xy1 - xy0
        cost_goal = 1-np.dot(goal_vec, new_vec) /(np.linalg.norm(goal_vec) * np.linalg.norm(new_vec) + 1e-6)

        min_dist = np.inf
        for obstacle in self.obstacles:
            dist = np.linalg.norm(xy1 - obstacle[:2]) - obstacle[2]
            if dist < 0:
                min_dist = 0
                break
            if dist < min_dist and dist < self.obstacle_radius:
                min_dist = dist
        cost_obstacle = 1/(min_dist + 1e-6)

        cost_speed = np.abs(np.linalg.norm(xy1 - xy0) / dt - self.optimal_speed_mps)

        return cost_goal * self.W["goal"] + cost_obstacle * self.W["obstacle"] + cost_speed * self.W["speed"]

    def step(self, x, dt):
        min_cost = np.inf
        vw = np.array([0, 0])
        cost_map = []
        for theta in np.linspace(0, 2 * np.pi, 32):
            for speed in np.linspace(0, self.optimal_speed_mps, 10):
                xy1 = x + np.array([np.cos(theta), np.sin(theta)]) * speed * dt
                cost = self.sample_cost(x, xy1, dt)
                if np.isnan(cost) or cost > 1000:
                    cost = 1000
                cost_map.append((xy1[0], xy1[1], cost))
                if cost < min_cost:
                    min_cost = cost
                    vw = np.array([speed, theta])

        return x + np.array([np.cos(vw[1]), np.sin(vw[1])]) * vw[0] * dt, cost_map

    def get_plan(self, x0, dt=0.05, H=100):
        x = x0
        plan = [x0]
        for t in range(H):
            new_x, cost_map = self.step(x, dt)
            plan.append(new_x)
            x = new_x
            if np.linalg.norm(x - self.goals[self.goal_idx]) < 0.1:
                break
        return plan

