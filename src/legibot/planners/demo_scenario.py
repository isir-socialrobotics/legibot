import matplotlib.pyplot as plt
import numpy as np

from src.legibot.planners.local_planner import LocalPlanner
from legibot.utils.viz_utils import plot_path, Visualizer

x0 = [0.1, 0.1]
goals = np.array([
                  [0.9, 0.9],
                  # [0.3, 0.9],
                  [0.9, 0.1],
                  [0.95, 0.4]
                  ])
goal_idx = 0

obstacles = np.array([
                      [0.5, 0.55, 0.1],
                      [0.2, 0.8, 0.1]
                      ])
obstacle_radius = 0.1
goal_radius = 0.1

# Visualizer().draw_obstacles(obstacles)
# Visualizer().draw_goals(goals)

# plan_rrt = RRT(x0, goals[goal_idx], obstacles)
# plot_path(plan_rrt, goals, obstacles, title="RRT")
#
# apf_planner = APF(goals, goal_idx, obstacles, 0.1, obstacle_radius)
# plan_apf_legible = apf_planner.get_plan(np.array([0.1, 0.1]))
#
# apf_planner.enable_legibility_force = False
# plan_apf_illegible = apf_planner.get_plan(np.array([0.1, 0.1]))
#
# field = np.zeros((25, 25, 2))
# for i in range(field.shape[0]):
#     for j in range(field.shape[1]):
#         x = np.array([i / field.shape[0], j / field.shape[1]])
#         if apf_planner.__is_collision__(x):
#             continue
#         f = apf_planner.__get_field_vec__(x)
#         if not np.isnan(f[0]):
#             field[j, i, 0] = f[0]
#             field[j, i, 1] = -f[1]

dwa_planner = LocalPlanner(goals, obstacles, goal_idx)
plan_dwa = dwa_planner.get_plan(np.array([0.1, 0.1]))

fig, axs = plt.subplots(1, 1, figsize=(6, 4))

# plot_path(plan_apf_legible, goals, obstacles, axs[0])
# plot_path(plan_apf_illegible, goals, obstacles, axs[1])
# plot_field(field, axs[2])
plot_path(plan_dwa, goals, obstacles, axs, "Legibility-Aware Local Planner")
plt.show()

Visualizer().draw_path(plan_dwa)
Visualizer().show()
