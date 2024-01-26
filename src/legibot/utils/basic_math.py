import numpy as np

def cos_2_vecs(v1, v2):
    return np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)

def norm(x, axis=None):
    return np.linalg.norm(x, axis=axis)

def find_intersections(lines1, lines2):
    # Extract line endpoints
    # Reshape lines into (N, 2, 2) and (M, 2, 2) arrays
    lines1 = np.array(lines1).reshape(-1, 2, 2)
    lines2 = np.array(lines2).reshape(-1, 2, 2)

    # Compute direction vectors for lines
    directions1 = lines1[:, 1]
    directions2 = lines2[:, 1]


    # Compute normal vectors for lines by swapping and negating components
    normals1 = np.stack([-directions1[:, 1], directions1[:, 0]], axis=-1)
    normals2 = np.stack([-directions2[:, 1], directions2[:, 0]], axis=-1)

    # Compute the dot products of normals with themselves
    dot_normals1 = np.sum(normals1 * normals1, axis=-1)
    dot_normals2 = np.sum(normals2 * normals2, axis=-1)

    # Compute the dot products of normals with each other
    dot_normals12 = np.dot(normals1, normals2.T)

    # Compute the denominators for the intersection points
    denominators = dot_normals1 * dot_normals2 - dot_normals12 ** 2

    # Compute the differences between line endpoints
    diff = lines2[:, None, 0] - lines1[:, 0]

    # Compute the parametric values for the intersections
    u = (dot_normals12 * np.dot(normals1, diff.T) - dot_normals2 * np.dot(normals2, diff.T)) / denominators

    # Compute the intersection points
    intersections = lines1[:, 0] + u[:, None] * directions1

    return intersections


def pairwise_intersections(lines: []):
    lines_np = np.array(lines).reshape(-1, 2, 2)
    # y = mx + b  =>  m = dy / dx
    m = lines_np[:, 1, 1] / (lines_np[:, 1, 0] + 1e-6)  # N x 1
    # b = y0 - m * x0
    b = lines_np[:, 0, 1] - m * lines_np[:, 0, 0]  # N x 1
    db = b[:, None] - b[None, :]  # N x N
    dm = m[:, None] - m[None, :]  # N x N
    # intersections_x = - db / (dm + 1e-6)
    intersections_x = db / (dm)
    intersections_y = m[:, None] * intersections_x + b[:, None]
    intersections = np.stack([intersections_x, intersections_y], axis=-1)
    N = intersections.shape[0]
    all_intersections = np.array([intersections[i, j] for i in range(N) for j in range(i + 1, N)])
    return all_intersections


if __name__ == "__main__":
    # x0, y0, dx, dy
    lines = [[0, 0, 1, 0],
             [0, 1, 1, 1],
             [0, 2, 0, 0.1],
                [0, 3, -1, 0.1],
                [0, 4, 0.2, -0.1],
            ]

    lines = np.array(lines).reshape(-1, 2, 2)

    all_intersections = pairwise_intersections(lines)
    print(np.round(all_intersections, 3))

