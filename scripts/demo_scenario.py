## demo_scenario.py

import math

import numpy as np

from legibot.planners.smoother import smooth_trajectory
from src.legibot.planners.local_planner import LocalPlanner
from src.legibot.planners.dragan_planner import DragPlanner
from legibot.utils.viz_utils import Visualizer

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

local_planner = DragPlanner(goals, obstacles, goal_idx, verbose=True, enable_legibility=False)
local_planner.legibility_cost_type = "cosine" # "cosine"
plan_legibot = local_planner.full_plan(np.array(x0))

# fig, axs = plt.subplots(1, 1, figsize=(6, 4))

plot_title = "Local Planner"
if local_planner.enable_legibility:
    plot_title += " (Legible)"
else:
    plot_title += " (Illegible)"

plot_title += f" - cost =[{local_planner.legibility_cost_type}]"

plan_smooth = smooth_trajectory(plan_legibot, num_points=len(plan_legibot) * 2)
# plt.show()

# Visualizer().draw_path(plan_legibot)
# Visualizer().show()
