import random
import math
import matplotlib.pyplot as plt
import numpy as np

from src.legibot.legible_fields.apf import APF
from src.legibot.legible_fields.rrt import rrt
from src.legibot.legible_fields.utils import plot_path, plot_field

x0 = [0.1, 0.1]
goals = np.array([[0.9, 0.9], [0.9, 0.1]])
goal_idx = 0

obstacles = np.array([[0.5, 0.55, 0.1], [0.2, 0.8, 0.1]])
obstacle_radius = 0.1
goal_radius = 0.1

plan_rrt = rrt(x0, goals[goal_idx], obstacles)
plot_path(plan_rrt, goals, obstacles, title="RRT")

apf = APF(goals, goal_idx, obstacles, 0.1, obstacle_radius)
plan_legible = apf.get_plan(np.array([0.1, 0.1]))

apf.enable_legibility_force = False
plan_illegible = apf.get_plan(np.array([0.1, 0.1]))

field = np.zeros((25, 25, 2))
for i in range(field.shape[0]):
    for j in range(field.shape[1]):
        x = np.array([i / field.shape[0], j / field.shape[1]])
        if apf.is_collision(x):
            continue
        f = apf.get_field_vec(x)
        if not np.isnan(f[0]):
            field[j, i, 0] = f[0]
            field[j, i, 1] = -f[1]

fig, axs = plt.subplots(1, 3, figsize=(12, 4))

# Adjust layout
plt.tight_layout()

# plot_path_cv(plan, goals, obstacles)
plot_path(plan_legible, goals, obstacles, axs[0])
plot_path(plan_illegible, goals, obstacles, axs[1])
plot_field(field, axs[2])
plt.show()
