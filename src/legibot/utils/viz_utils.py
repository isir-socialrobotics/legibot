import matplotlib.pyplot as plt
import numpy as np
import cv2
import matplotlib.patches as patches

from legibot.utils.singleton import Singleton


class Visualizer(metaclass=Singleton):
    def __init__(self, mode="opencv"):
        self.mode = mode  # "opencv" or "matplotlib"
        self.im_size = (1000, 1000)
        self.world_x_range = (-10, 10)
        self.world_y_range = (-10, 10)

        if self.mode == "opencv":
            self.img = None
            self.reset()

        else:  # matplotlib
            self.fig, self.ax = None, None

    def transform(self, x, y):
        t_xy = np.dot(self.transform_matrix, np.array([x, y, 1]))
        return t_xy[0], t_xy[1]

    def show(self, delay=0, title="image"):
        if self.mode == "opencv":
            cv2.imshow(title, cv2.flip(self.img, 0))
            k = cv2.waitKey(delay)
            if k == 27: # if ESC is pressed, exit the program
                print("Exiting...")
                exit()
        else:
            plt.show()

    def save(self, filename):
        if self.mode == "opencv":
            cv2.imwrite(filename, cv2.flip(self.img, 0))
        else:
            # plt.legend()
            plt.savefig(filename)

    def reset(self):
        if self.mode == "opencv":
            self.img = np.ones((self.im_size[0], self.im_size[1], 3), dtype=np.uint8) * 255
        else:
            self.fig, self.ax = plt.subplots(1, 1, figsize=(10, 7))
            self.ax.axis('equal')
            self.ax.set_xlim(self.world_x_range)
            self.ax.set_ylim(self.world_y_range)
            self.ax.set_xticklabels([])
            self.ax.set_yticklabels([])

        # transform matrix
        self.scale = self.img.shape[0] / (self.world_x_range[1] - self.world_x_range[0])
        offset_x = -self.world_x_range[0] * self.scale
        offset_y = -self.world_y_range[0] * self.scale
        self.transform_matrix = np.array([[self.scale, 0, offset_x],
                                          [0, self.scale, offset_y],
                                          ])

    def draw_obstacles(self, obstacles):
        if self.mode == "opencv":
            for obstacle in obstacles:
                obs_xy = self.transform(obstacle[0], obstacle[1])
                cv2.circle(self.img, (int(obs_xy[0]), int(obs_xy[1])),
                                        int(obstacle[2]*self.scale),
                           (0, 0, 0), -1)
        else:
            for obstacle in obstacles:
                circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], label='Obstacle' if 'Obstacle' not in self.ax.get_legend_handles_labels()[1] else "",
                                    color='k', hatch='///', fill=False)
                self.ax.add_artist(circle)

    def draw_triangle(self, xy_center, yaw, color_rgb=(0, 0, 255), edge_color=None):
        points = np.array([[xy_center[0] + np.cos(yaw) * 0.2, xy_center[1] + np.sin(yaw) * 0.2],
                           [xy_center[0] + np.cos(yaw + np.pi * 2/3) * 0.15, xy_center[1] + np.sin(yaw + np.pi * 2/3) * 0.15],
                           [xy_center[0] + np.cos(yaw + np.pi * 4/3) * 0.15, xy_center[1] + np.sin(yaw + np.pi * 4/3) * 0.15],
                           ])
        if self.mode == "opencv":
            points_tf = np.array([self.transform(p[0], p[1]) for p in points])
            cv2.polylines(self.img, [points_tf], True, color_rgb[::-1], 2)
        else:
            triangle = patches.Polygon(points, closed=True, fill=True,
                                       fc=(color_rgb[0]/255, color_rgb[1]/255, color_rgb[2]/255), ec=edge_color, linewidth=2)
            self.ax.add_patch(triangle)

    def draw_initial_point(self, x0):
        if self.mode == "opencv":
            x0_xy = self.transform(x0[0], x0[1])
            cv2.circle(self.img, (int(x0_xy[0]), int(x0_xy[1])),
                                    10, (0, 0, 255), -1)
        else:
            self.ax.plot(x0[0], x0[1], 'ob', label='Starting Point', markersize=10)

    def draw_goals(self, goals, color=(0, 255, 0), edge_color=None):
        for ii, goal in enumerate(goals):
            self.draw_triangle(goal[:2], goal[2], color, edge_color)
        # if self.mode == "opencv":
        #     for goal in goals:
        #         g_xy = self.transform(goal[0], goal[1])
        #         cv2.drawMarker(self.img, (int(g_xy[0]), int(g_xy[1])),
        #                        color, cv2.MARKER_CROSS, 20, 5)
        # else:
        #     for ii, goal in enumerate(goals):
        #         self.ax.plot(goal[0], goal[1], '+g', markersize=10,
        #                     label='Goal' if 'Goal' not in self.ax.get_legend_handles_labels()[1] else "")
            self.ax.text(goal[0]+0.5, goal[1], f"G{ii+1}", fontsize=12)

    def draw_path(self, path, color_rgb=(0, 0, 255)):
        if self.mode == "opencv":
            for i in range(len(path)-1):
                p_i_xy = self.transform(path[i][0], path[i][1])
                p_i1_xy = self.transform(path[i+1][0], path[i+1][1])
                cv2.line(self.img, (int(p_i_xy[0]), int(p_i_xy[1])), (int(p_i1_xy[0]), int(p_i1_xy[1])),
                                    color_rgb[::-1], 5)
        else:
            self.ax.plot([x[0] for x in path], [x[1] for x in path],
                         color=(color_rgb[0]/255, color_rgb[1]/255, color_rgb[2]/255), marker='o', linestyle='-', label='Path')

    def add_arrow(self, xy, uv, color_rgb=(0, 100, 255)):
        if self.mode == "opencv":
            xy_trans = self.transform(xy[0], xy[1])
            uv_trans = self.transform(uv[0], uv[1])
            cv2.arrowedLine(self.img, (int(xy_trans[0]), int(xy_trans[1])),
                                      (int(uv_trans[0]), int(uv_trans[1])),
                                      color_rgb[::-1], 2)
        else:
            self.ax.arrow(xy[0], xy[1], uv[0] - xy[0], uv[1] - xy[1],
                            head_width=0.2, head_length=0.3, fc='k', ec=(color_rgb[0]/255, color_rgb[1]/255, color_rgb[2]/255))

    def draw_heatmap(self, xy_center, polar_cost_map, radius_range, angle_range, title="Heatmap"):
        new_img = self.img  # np.ones((self.im_size[0], self.im_size[1], 3), dtype=np.uint8) * 255
        polar_cost_map_uint = (polar_cost_map - polar_cost_map.min()) / (polar_cost_map.max() - polar_cost_map.min()) * 255
        if self.mode == "opencv":
            for i in range(polar_cost_map.shape[0]):
                for j in range(polar_cost_map.shape[1]):
                    angle = angle_range[0] + i * (angle_range[1] - angle_range[0]) / polar_cost_map.shape[0]
                    radius = radius_range[0] + j * (radius_range[1] - radius_range[0]) / polar_cost_map.shape[1]
                    x = xy_center[0] + radius * np.cos(angle)
                    y = xy_center[1] + radius * np.sin(angle)
                    x, y = self.transform(x, y)
                    color = polar_cost_map_uint[i, j]
                    cv2.circle(new_img, (int(x), int(y)), 3, (255-color, 0, color), -1)

        # cv2.imshow(title, cv2.flip(new_img, 0))
        # cv2.waitKey(50)

def plot_path(path, goals, obstacles, color='-or', ax=None, title=""):
    """
    :param path: path from x0 to goal
    :param goals: goal points
    """
    if ax is None:
        ax = plt.gca()

    # plot the path
    ax.plot([x[0] for x in path], [x[1] for x in path], color, label='Path')

    # plot initial point
    ax.plot(path[0][0], path[0][1], 'ob', label='Initial Point', markersize=10)
    ax.set_title(title)

    for goal in goals:
        # show goals with x marks
        ax.plot(goal[0], goal[1], '+g', markersize=10,
                label='Goal' if 'Goal' not in ax.get_legend_handles_labels()[1] else "")

    for obstacle in obstacles:
        circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], label='Obstacle' if 'Obstacle' not in ax.get_legend_handles_labels()[1] else "",
                            color='k', hatch='///', fill=False)
        ax.add_artist(circle)

    ax.legend()
    ax.axis('equal')
    # plt.show()


def plot_field(field, ax=None):
    if ax is None:
        ax = plt.gca()

    ax.quiver(field[:, :, 0], field[:, :, 1], angles='xy')
