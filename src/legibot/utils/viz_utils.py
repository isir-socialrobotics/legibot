import matplotlib.pyplot as plt
import numpy as np
import cv2

from legibot.utils.singleton import Singleton


class Visualizer(metaclass=Singleton):
    def __init__(self):
        self.mode = "opencv"  # "matplotlib"
        self.im_size = (1000, 1000)
        self.world_x_range = (-10, 10)
        self.world_y_range = (-10, 10)

        self.img = None
        self.reset()

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

    def save(self, filename):
        if self.mode == "opencv":
            cv2.imwrite(filename, cv2.flip(self.img, 0))

    def reset(self):
        if self.mode == "opencv":
            self.img = np.ones((self.im_size[0], self.im_size[1], 3), dtype=np.uint8) * 255
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

    def draw_goals(self, goals, color=(0, 255, 0)):
        if self.mode == "opencv":
            for goal in goals:
                g_xy = self.transform(goal[0], goal[1])
                cv2.drawMarker(self.img, (int(g_xy[0]), int(g_xy[1])),
                               color, cv2.MARKER_CROSS, 20, 5)

    def draw_path(self, path):
        if self.mode == "opencv":
            for i in range(len(path)-1):
                p_i_xy = self.transform(path[i][0], path[i][1])
                p_i1_xy = self.transform(path[i+1][0], path[i+1][1])
                cv2.line(self.img, (int(p_i_xy[0]), int(p_i_xy[1])), (int(p_i1_xy[0]), int(p_i1_xy[1])),
                                    (255, 0, 0), 5)

    def add_arrow(self, xy, uv, color=(0, 100, 255)):
        if self.mode == "opencv":
            xy_trans = self.transform(xy[0], xy[1])
            uv_trans = self.transform(uv[0], uv[1])
            cv2.arrowedLine(self.img, (int(xy_trans[0]), int(xy_trans[1])),
                                      (int(uv_trans[0]), int(uv_trans[1])),
                                      color, 2)

    def draw_heatmap(self, xy_center, polar_cost_map, radius_range, angle_range):

        new_img = np.ones((self.im_size[0], self.im_size[1], 3), dtype=np.uint8) * 255
        polar_cost_map_uint = (polar_cost_map - polar_cost_map.min()) / (polar_cost_map.max() - polar_cost_map.min()) * 255
        if self.mode == "opencv":
            for i in range(polar_cost_map.shape[0]):
                for j in range(polar_cost_map.shape[1]):
                    angle = angle_range[0] + i * (angle_range[1] - angle_range[0]) / polar_cost_map.shape[0]
                    radius = radius_range[0] + j * (radius_range[1] - radius_range[0]) / polar_cost_map.shape[1]
                    x = xy_center[0] + radius * np.cos(angle) * 6
                    y = xy_center[1] + radius * np.sin(angle) * 6
                    x, y = self.transform(x, y)
                    color = polar_cost_map_uint[i, j]
                    cv2.circle(new_img, (int(x), int(y)), 5, (255-color, 0, color), -1)

        cv2.imshow("Heatmap", cv2.flip(new_img, 0))
        cv2.waitKey(50)

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
        ax.plot(goal[0], goal[1], 'xg', markersize=10,
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
