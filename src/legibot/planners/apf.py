# Artificial Potential Field (APF) legible field

import numpy as np


class APF:
    def __init__(self, goals, goal_idx=0, obstacles=[], **kwargs):
        self.goals = goals
        self.goal_idx = goal_idx
        self.other_goals_idx = [i for i in range(len(goals)) if i != goal_idx]
        self.obstacles = obstacles
        self.goal_radius = kwargs.get("goal_radius", 0.1)
        self.obstacle_radius = kwargs.get("obstacle_radius", 0.1)
        self.eta = kwargs.get("eta", 1.5)  # goal force weight
        self.rho = kwargs.get("rho", 0.5)  # obstacle force weight
        self.max_step = 0.04
        self.closest_points_visited = []
        self.enable_legibility_force = True

    def __get_obstacle_force__(self, x):
        # initialize the obstacle force
        obstacle_force = np.zeros(2)

        # iterate over the obstacles
        for obstacle in self.obstacles:
            # add the obstacle force
            closest_point = obstacle[:2] + (x - obstacle[:2]) / np.linalg.norm(x - obstacle[:2]) * obstacle[2]
            self.closest_points_visited.append(closest_point)

            if np.linalg.norm(x - closest_point) < self.obstacle_radius:
                obstacle_force += self.rho * (x - closest_point) / np.linalg.norm(x - closest_point) ** 2

        # return the obstacle force
        return obstacle_force

    def __get_goal_force__(self, x):
        # initialize the goal force
        goal_force = np.zeros(2)

        # add the goal force
        goal_force += -self.eta * (x - self.goals[self.goal_idx]) / np.linalg.norm(x - self.goals[self.goal_idx])

        # return the goal force
        return goal_force

    def __get_other_goals_force__(self, x):
        # initialize the other goals force
        other_goals_force = np.zeros(2)

        # iterate over the other goals
        for goal_idx in self.other_goals_idx:
            # add the other goal force
            other_goals_force += self.eta/2 * (x - self.goals[goal_idx]) / np.linalg.norm(x - self.goals[goal_idx])

        # return the other goals force
        return other_goals_force

    def __get_field_vec__(self, x):
        # initialize the legible field
        field_vector = np.zeros(2)

        # add the goal force
        field_vector += self.__get_goal_force__(x)

        # add the obstacle force
        field_vector += self.__get_obstacle_force__(x)

        # add the other goals force
        if self.enable_legibility_force:
            field_vector += self.__get_other_goals_force__(x)

        # add a very small random force
        field_vector += np.random.normal(0, 0.01, 2)

        # clip the legible field
        if np.linalg.norm(field_vector) > self.max_step:
            field_vector = self.max_step * field_vector / np.linalg.norm(field_vector)

        # return the legible field
        return field_vector

    def __is_collision__(self, x):
        # check if x is in collision with any of the obstacles
        for obstacle in self.obstacles:
            if (x[0] - obstacle[0]) ** 2 + (x[1] - obstacle[1]) ** 2 < obstacle[2] ** 2:
                return True
        return False

    def __post_process__(self, plan):
        # remove oscillations
        i = 0
        while i < len(plan) - 2:
            if np.linalg.norm(plan[i] - plan[i+2]) < 0.1:
                del plan[i+1]
            else:
                i += 1

        return plan

    def get_plan(self, initial_point, max_iter=10000):
        plan = [initial_point]

        # iterate until the goal is reached
        iter = 0
        while np.linalg.norm(plan[-1] - self.goals[self.goal_idx]) > self.goal_radius:
            # get the legible field at the current point
            legible_field = self.__get_field_vec__(plan[-1])

            # get the next point
            next_point = plan[-1] + legible_field

            if self.__is_collision__(next_point):
                return plan

            plan.append(next_point)

            iter += 1
            if iter > max_iter:
                print('Warning: max_iter reached')
                break
            # plot_path_cv(plan, self.goals, self.obstacles)

        else:
            plan.append(self.goals[self.goal_idx])
            plan = self.__post_process__(plan)
            print('Goal reached')


        return plan





