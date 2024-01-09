import matplotlib.pyplot as plt


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


def plot_path_cv(path, goals, obstacles):
    import cv2
    import numpy as np

    # create a black image
    img = np.ones((512, 512, 3), np.uint8) * 255

    # plot the path
    for i in range(len(path)-1):
        cv2.line(img, (int(path[i][0]*512), int(path[i][1]*512)), (int(path[i+1][0]*512), int(path[i+1][1]*512)), (255, 0, 0), 5)

    # plot initial point
    cv2.circle(img, (int(path[0][0]*512), int(path[0][1]*512)), 5, (0, 0, 255), 5)

    for goal in goals:
        # show goals with x marks
        cv2.drawMarker(img, (int(goal[0]*512), int(goal[1]*512)), (0, 255, 0), cv2.MARKER_CROSS, 20, 5)

    for obstacle in obstacles:
        # plt.plot(obstacle[0], obstacle[1], 'ok')
        cv2.circle(img, (int(obstacle[0]*512), int(obstacle[1]*512)), int(obstacle[2]*512), (0, 0, 0), -1)
        # add to legend

    # show the plot
    # plt.legend(['path', 'initial point', 'goal'])
    # plt.axis('equal')

    # cv2.imshow('image', cv2.flip(img, 0))
    # cv2.waitKey(0)
    return img


def plot_field(field, ax=None):
    if ax is None:
        ax = plt.gca()

    ax.quiver(field[:, :, 0], field[:, :, 1], angles='xy')
