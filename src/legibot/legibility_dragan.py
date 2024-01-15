import numpy as np


def min_cost(a, b) -> float:
    """cost of travelling from point a to b (Euclidean distance)"""
    return np.linalg.norm(b - a)


def actual_cost(traj) -> float:
    """cost of travelling along a trajectory (sum of Euclidean distances between consecutive points)"""
    cost_ = 0
    for p1, p2 in zip(traj[:-1], traj[1:]):
        cost_ += np.linalg.norm(p2 - p1)
    return cost_


def calc_legibility(goals_xy, cur_traj, cost_scaler=1.0):
    p_g_posterior = [0] * len(goals_xy)
    for ii, g in enumerate(goals_xy):
        p_g_prior = 1 / len(goals_xy)  # assuming a uniform P(G) distribution
        cost_sg = min_cost(cur_traj[0], g) * cost_scaler
        cost_qg = min_cost(cur_traj[-1], g) * cost_scaler
        cost_sq = actual_cost(cur_traj) * cost_scaler

        p_g_posterior[ii] = np.exp(-cost_sq - cost_qg) / np.exp(-cost_sg) * p_g_prior

    # normalize
    p_g_posterior = [x / sum(p_g_posterior) for x in p_g_posterior]
    return p_g_posterior


def EoR(full_traj, goal_xy):
    """Envelope of Readiness: final percentage of the trajectory that is legible"""
    pass


if __name__ == "__main__":
    a = np.array([0, 0, 0])
    b = np.array([1, 3, 3])

    traj = np.array([[0, 0],
                     [1, 2],
                     [3, 4],
                     [4, 6]])

    print(min_cost(a, b))
    print(actual_cost(traj))
