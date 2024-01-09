import random
import math
import matplotlib.pyplot as plt

def sample_random():
    """
    :return: random point in the workspace
    """
    return [random.uniform(0, 1), random.uniform(0, 1)]

def find_nearest(tree, x_rand):
    """
    :param tree: list of points in the tree
    :param x_rand: random point
    :return: point in the tree nearest to x_rand
    """
    # compute the distance between x_rand and each point in the tree
    dist = [math.sqrt((x_rand[0] - x[0]) ** 2 + (x_rand[1] - x[1]) ** 2) for x in tree]
    # return the point in the tree that is closest to x_rand
    return tree[dist.index(min(dist))]

def extend(x_near, x_rand, delta):
    """
    :param x_near: nearest point in the tree
    :param x_rand: random point
    :param delta: step size
    :return: point in the tree that is delta away from x_near towards x_rand
    """
    # compute the angle between x_rand and x_near
    theta = math.atan2(x_rand[1] - x_near[1], x_rand[0] - x_near[0])
    # return the point in the tree that is delta away from x_near towards x_rand
    return [x_near[0] + delta * math.cos(theta), x_near[1] + delta * math.sin(theta)]

def is_collision(x, obstacles):
    """
    :param x: point to check for collision
    :param obstacles: list of obstacles
    :return: True if x is in collision with any of the obstacles, False otherwise
    """
    # check if x is in collision with any of the obstacles
    for obstacle in obstacles:
        if (x[0] - obstacle[0]) ** 2 + (x[1] - obstacle[1]) ** 2 < obstacle[2] ** 2:
            return True
    return False


def is_close(x, goal, epsilon):
    # check if x is close enough to the goal
    return (x[0] - goal[0]) ** 2 + (x[1] - goal[1]) ** 2 < epsilon ** 2

def path(x, tree):
    """
    :param x: point in the tree
    :param tree: list of points in the tree
    :return: path from the initial point to x
    """
    # initialize the path with the point x
    path = [x]
    # iterate until the initial point is reached
    while x != tree[0]:
        # find the parent of x
        for y in tree:
            if x == y:
                continue
            if (x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2 < 0.1:
                x = y
                break
        # add the parent of x to the path
        path.append(x)
    # reverse the path
    path.reverse()
    # return the path
    return path

def RRT(x0, goal, obstacles, max_iter=1000, delta=0.1, epsilon=0.1):
    """
    :param x0: initial point
    :param goal: goal point
    :param obstacles: list of obstacles
    :param max_iter: maximum number of iterations
    :param delta: step size
    :param epsilon: goal tolerance
    :return: path from x0 to goal
    """
    # initialize the tree with the initial point
    tree = [x0]
    # iterate until max_iter
    for i in range(max_iter):
        # sample a random point
        x_rand = sample_random()
        # find the nearest point in the tree
        x_near = find_nearest(tree, x_rand)
        # extend the tree towards x_rand
        x_new = extend(x_near, x_rand, delta)
        # check if x_new is in collision with any of the obstacles
        if not is_collision(x_new, obstacles):
            # add x_new to the tree
            tree.append(x_new)
            # check if x_new is close enough to the goal
            if is_close(x_new, goal, epsilon):
                # return the path from x0 to x_new
                return path(x_new, tree)
    # return an empty path if no path is found
    return []


if __name__ == '__main__':
    # define the initial point
    x0 = [0.1, 0.1]

    # define 2 goal points
    goal1 = [0.9, 0.9]
    goal2 = [0.9, 0.1]

    # define the obstacles
    obstacles = [[0.5, 0.5, 0.1], [0.2, 0.8, 0.1]]

    # find a path from x0 to goal1
    path1 = RRT(x0, goal1, obstacles)
    print(path1)
