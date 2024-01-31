from scipy.interpolate import CubicSpline
import numpy as np

def smooth_trajectory(trajectory, num_points=100):
    """ Smooth a trajectory using a cubic spline.

    Args:
        trajectory (list): A list of (x, y, theta) tuples.
        num_points (int): The number of points to interpolate.

    Returns:
        list: A list of (x, y, theta) tuples.
    """

    x = [p[0] for p in trajectory]
    y = [p[1] for p in trajectory]
    theta = [p[2] for p in trajectory]

    cs_x = CubicSpline(range(len(x)), x)
    cs_y = CubicSpline(range(len(y)), y)
    cs_theta = CubicSpline(range(len(theta)), theta)

    xs = cs_x(np.linspace(0, len(x) - 1, num_points))
    ys = cs_y(np.linspace(0, len(y) - 1, num_points))
    thetas = cs_theta(np.linspace(0, len(theta) - 1, num_points))

    return [(x, y, theta) for x, y, theta in zip(xs, ys, thetas)]