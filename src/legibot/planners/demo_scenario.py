import math

import matplotlib.pyplot as plt
import numpy as np

from legibot.planners.smoother import smooth_trajectory
from src.legibot.planners.local_planner import LocalPlanner
# from src.legibot.planners.dragan_planner import DragPlanner as LocalPlanner
from legibot.utils.viz_utils import plot_path, Visualizer

Visualizer().mode = "matplotlib"
Visualizer().reset()
x0 = [0, 0, 0]
goals = np.array([
                  [10, 10, math.radians(-135)],
                  # [3, 9],
                  # [9, 1, math.radians(180)],
                  [9.55, 6, math.radians(180)]
                  ])
goal_idx = 0

obstacles = np.array([
                      [5, 5.5, 0.3],
                      [2, 8, 0.5],
                      [8, 8, 0.3],
                      # [9, 1, 0.5],
                      # [9.55, 6, 0.5],
                      # [10, 10, 0.5],
                      ])
Visualizer().world_x_range = (-1.5, 12)
Visualizer().world_y_range = (-1.5, 12)
Visualizer().reset()

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

local_planner = LocalPlanner(goals, obstacles, goal_idx, verbose=2, enable_legibility=True)
local_planner.legibility_cost_type = "cosine" # "cosine"
plan_legibot = local_planner.full_plan(np.array(x0), dt=0.6)

# fig, axs = plt.subplots(1, 1, figsize=(6, 4))

plot_title = "Local Planner"
if local_planner.enable_legibility:
    plot_title += " (Legible)"
else:
    plot_title += " (Illegible)"

plot_title += f" - cost =[{local_planner.legibility_cost_type}]"

# plan_smooth = smooth_trajectory(plan_legibot, num_points=len(plan_legibot) * 2)
# plot_path(plan_legibot, goals, obstacles, color='-ob', ax=axs, title=plot_title)
# plot_path(plan_smooth, goals, obstacles, color='-or', ax=axs, title=plot_title)
# plt.show()

# Visualizer().draw_path(plan_legibot)
# Visualizer().show()
