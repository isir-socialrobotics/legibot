import matplotlib.pyplot as plt
import numpy as np
import cv2

from legibot.utils.singleton import Singleton


class Visualizer(metaclass=Singleton):
    def __init__(self):
        self.mode = "opencv"  # "matplotlib"
        self.im_size = (1080, 1080)
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
            cv2.waitKey(delay)

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

    def draw_goals(self, goals):
        if self.mode == "opencv":
            for goal in goals:
                g_xy = self.transform(goal[0], goal[1])
                cv2.drawMarker(self.img, (int(g_xy[0]), int(g_xy[1])),
                               (0, 255, 0), cv2.MARKER_CROSS, 20, 5)

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


def plot_path(path, goals, obstacles, ax=None, title=""):
    """
    :param path: path from x0 to goal
    :param goals: goal points
    """
    if ax is None:
        ax = plt.gca()

    # plot the path
    ax.plot([x[0] for x in path], [x[1] for x in path], '-or', label='Path')

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
