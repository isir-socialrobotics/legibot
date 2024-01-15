import matplotlib.pyplot as plt
import numpy as np
import cv2

from legibot.utils.singleton import Singleton


class Visualizer(metaclass=Singleton):
    def __init__(self):
        self.mode = "opencv"  # "matplotlib"

        # OpenCV
        if self.mode == "opencv":
            self.scale = 512
            self.img = self.__blank_image__()

    def show(self, delay=0, title="image"):
        if self.mode == "opencv":
            cv2.imshow(title, cv2.flip(self.img, 0))
            cv2.waitKey(delay)

    def save(self, filename):
        if self.mode == "opencv":
            cv2.imwrite(filename, cv2.flip(self.img, 0))

    def __blank_image__(self):
        if self.mode == "opencv":
            return np.ones((self.scale, self.scale, 3), np.uint8) * 255

    def draw_obstacles(self, obstacles):
        if self.mode == "opencv":
            for obstacle in obstacles:
                cv2.circle(self.img, (int(obstacle[0]*self.scale),
                                        int(obstacle[1]*self.scale)),
                                        int(obstacle[2]*self.scale),
                           (0, 0, 0), -1)

    def draw_goals(self, goals):
        if self.mode == "opencv":
            for goal in goals:
                cv2.drawMarker(self.img, (int(goal[0]*self.scale),
                                          int(goal[1]*self.scale)),
                                          (0, 255, 0), cv2.MARKER_CROSS,
                                          20, 5)

    def draw_path(self, path):
        if self.mode == "opencv":
            for i in range(len(path)-1):
                cv2.line(self.img, (int(path[i][0]*self.scale), int(path[i][1]*self.scale)),
                                    (int(path[i+1][0]*self.scale), int(path[i+1][1]*self.scale)),
                                    (255, 0, 0), 5)

    def add_arrow(self, xy, uv, color=(0, 100, 255)):
        if self.mode == "opencv":
            cv2.arrowedLine(self.img, (int(xy[0]*self.scale), int(xy[1]*self.scale)),
                                      (int(uv[0]*self.scale), int(uv[1]*self.scale)),
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
