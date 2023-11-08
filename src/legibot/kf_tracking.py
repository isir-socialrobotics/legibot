import numpy as np
from filterpy.kalman import KalmanFilter


if __name__ == "__main__":
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.x = np.zeros((4,))
    dt = 1.0  # time step
    kf.F = np.array([[1., 0., dt, 0.],
                     [0., 1., 0., dt],
                     [0., 0., 1., 0.],
                     [0., 0., 0., 1.]])  # state transition matrix
    kf.H = np.array([[1., 0., 0., 0.],
                     [0., 1., 0., 0.]])  # Measurement function
    kf.P *= 1000.  # covariance matrix
    kf.R = np.eye(2) * 5  # state uncertainty
    kf.Q = np.eye(4) * 0.1  # process uncertainty

    p1 = np.array([2, 3])
    p2 = np.array([8, 5])
    measurements = [(p1[0] * (1-t) + p2[0] * t, p1[1] * (1-t) + p2[1] * t) for t in np.linspace(0, 1, 10)]

    # add noise
    measurements = [np.array(m) + np.random.normal(0, 0.4, 2) for m in measurements]

    for i in range(len(measurements)):
        kf.predict()
        kf.update(measurements[i])
        print(f"i: {i}, x: {kf.x[0]:.2f}, y: {kf.x[1]:.2f}, vx: {kf.x[2]:.2f}, vy: {kf.x[3]:.2f}")



