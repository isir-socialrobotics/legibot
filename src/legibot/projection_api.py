import numpy as np
import math
import cv2


# def project_point(point, homog):
#     p = np.array([point[1], point[0], 1], dtype=np.float32)
#     p_proj = (homog @ p.T).T
#     return p_proj[0] / p_proj[2], p_proj[1] / p_proj[2]


def project_points(points, homog):
    points = np.array(points, dtype=np.float32)
    p = np.array([points[:, 0], points[:, 1], np.ones(points.shape[0])], dtype=np.float32)
    p_proj = (homog @ p).T
    p_norm = np.stack([p_proj[:, 1] / p_proj[:, 2], p_proj[:, 0] / p_proj[:, 2]], axis=1)
    return np.reshape(p_norm, (points.shape[0], 2))


def draw_projected_grid(im, grid, homog, line_color_bgr=(0, 255, 0), line_width=1):
    grid_lines = []
    for i in range(grid[0].shape[0]):
        for j in range(grid[1].shape[0]):
            if i != 0:
                grid_lines.append([[grid[0][i - 1], grid[1][j], 1], [grid[0][i], grid[1][j], 1]])
            if j != 0:
                grid_lines.append([[grid[0][i], grid[1][j - 1], 1], [grid[0][i], grid[1][j], 1]])

    grid_lines = np.array(grid_lines, dtype=np.float32)
    grid_lines_proj = (homog @ grid_lines.reshape(-1, 3).T).T.reshape(-1, 2, 3)

    for i in range(grid_lines_proj.shape[0]):
        l = grid_lines_proj[i]
        p0 = (int(l[0, 0] / l[0, 2]), int(l[0, 1] / l[0, 2]))
        p1 = (int(l[1, 0] / l[1, 2]), int(l[1, 1] / l[1, 2]))
        cv2.line(im, p0, p1, line_color_bgr, line_width)


def make_homog(im_size=(720, 1280), cam_pitch_deg=-27, cam_height=1.6):  # todo: get image size
    focal_length = 600  # fixme: Focal length in pixels
    image_width = im_size[1]
    image_height = im_size[0]

    cx = (image_width / 2) - 0.5
    cy = (image_height / 2) - 0.5

    K = np.array([[focal_length, 0, cx],
                  [0, focal_length, cy],
                  [0, 0, 1]])

    # Pitch angle in degrees
    pitch_degrees = cam_pitch_deg + 90  # fixme: the pitch angle of observer
    pitch_radians = math.radians(pitch_degrees)

    # Rotation matrix for pitch
    R = np.array([[1, 0, 0],
                  [0, math.cos(pitch_radians), -math.sin(pitch_radians)],
                  [0, math.sin(pitch_radians), math.cos(pitch_radians)]])
    r_world = R.T

    # Translation vector (height of the camera above the ground)
    T = np.array([0, 0, cam_height])
    t_world2cam = -R.T @ T
    E = [r_world[:, 0], r_world[:, 1], t_world2cam]
    E = np.vstack(E)

    h_world2cam = K @ E
    return h_world2cam, np.linalg.inv(h_world2cam)


def make_grid():
    xs = np.arange(0.0, 15.01, 1)
    ys = np.arange(-5, 5.01, 1)
    return ys, xs


def capture_click(event, x, y, flags, param):  # print back-projected point
    if event == cv2.EVENT_LBUTTONDOWN:
        homog_inv = param[0]
        pnt = np.array([x, y, 1])
        pnt_proj = homog_inv @ pnt
        pnt_proj = [pnt_proj / pnt_proj[2]]
        pnt_proj = [pnt_proj[0][1], pnt_proj[0][0]]
        print(f"({x}, {y}) -> ({round(pnt_proj[0], 2)}, {round(pnt_proj[1], 2)})")


if __name__ == '__main__':
    h, h_inv = make_homog()

    im = np.zeros((720, 1280, 3), dtype=np.uint8)
    cv2.namedWindow("im")
    cv2.setMouseCallback('im', capture_click, (h_inv,))

    g = make_grid()
    draw_projected_grid(im, g, h)
    cv2.imshow("im", im)
    cv2.waitKey(0)
