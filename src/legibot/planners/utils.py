import matplotlib.pyplot as plt


def plot_path(path, goals, obstacles, ax=None, title=""):
    """
    :param path: path from x0 to goal
    :param goals: goal points
    """
    if ax is None:
        ax = plt.gca()

    # plot the path
    ax.plot([x[0] for x in path], [x[1] for x in path], '-or')

    # plot initial point
    ax.plot(path[0][0], path[0][1], 'or', label='initial point')
    ax.set_title(title)

    for goal in goals:
        # show goals with x marks
        ax.plot(goal[0], goal[1], 'xg')

    for obstacle in obstacles:
        # plt.plot(obstacle[0], obstacle[1], 'ok')
        circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='k')
        ax.add_artist(circle)
        # add to legend

    # show the plot
    ax.legend(['path', 'initial point', 'goal'])
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

    cv2.imshow('image', cv2.flip(img, 0))
    cv2.waitKey(0)


def plot_field(field, ax=None):
    if ax is None:
        ax = plt.gca()

    ax.quiver(field[:, :, 0], field[:, :, 1], angles='xy')
