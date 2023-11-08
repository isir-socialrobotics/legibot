import numpy as np
import math
import cv2


def draw_projected_grid(im, grid, homog):
    grid_lines = []
    for i in range(grid[0].shape[0]):
        grid_lines.append(np.array([[grid[0][i], grid[1][0], 1],
                                    [grid[0][i], grid[1][-1], 1]]))
    for i in range(grid[1].shape[0]):
        grid_lines.append(np.array([[grid[0][0], grid[1][i], 1],
                                    [grid[0][-1], grid[1][i], 1]]))

    grid_lines = np.array(grid_lines, dtype=np.float32)
    xx = (homog @ grid_lines.reshape(-1, 3).T).T.reshape(-1, 2, 3)
    im_points = xx[:, :, :2] / xx[:, :, 2:]

    for i in range(im_points.shape[0]):
        for j in range(im_points.shape[1]):
            p = im_points[i, j].astype(np.int32)
            # print(p , grid[0][j], grid[1][i])
            cv2.circle(im, (int(im_points[i, j, 0]), int(im_points[i, j, 1])), 3, (0, 255, 0), -1)

    cv2.imshow('im', im)
    cv2.waitKey(0)


def make_homog():
    focal_length = 600  # Focal length in pixels
    image_width = 1280
    image_height = 720

    cx = (image_width / 2) - 0.5
    cy = (image_height / 2) - 0.5

    K = np.array([[focal_length, 0, cx],
                  [0, focal_length, cy],
                  [0, 0, 1]])

    # Pitch angle in degrees
    pitch_degrees = -20 + 90
    pitch_radians = math.radians(pitch_degrees)

    # Rotation matrix for pitch
    R = np.array([[1, 0, 0],
                  [0, math.cos(pitch_radians), -math.sin(pitch_radians)],
                  [0, math.sin(pitch_radians), math.cos(pitch_radians)]])
    r_world = R.T

    # Translation vector (height of the camera above the ground)
    camera_height = 1.6  # meters
    T = np.array([0, 0, -camera_height])
    t_world2cam = -R.T @ T
    E = [r_world[:, 0], r_world[:, 1], t_world2cam]
    E = np.vstack(E)

    h_world2cam = K @ E
    return h_world2cam


def capture_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # todo: to print back-projected point
        pnt = np.array([1280-x, 720-y, 1])
        pnt_proj = homog_inv @ pnt
        pnt_proj = [pnt_proj / pnt_proj[2]]
        pnt_proj = [pnt_proj[0][1], pnt_proj[0][0]]
        print(f"({x}, {y}) -> ({round(pnt_proj[0], 2)}, {round(pnt_proj[1], 2)})")


if __name__ == '__main__':
    # test code
    homog = make_homog()
    homog_inv = np.linalg.inv(homog)

    cv2.namedWindow("im")
    cv2.setMouseCallback('im', capture_click)

    # test_pnts = [[5,  0.2], [5, -0.2]]
    # for pnt in test_pnts:
    #     pnt_proj = homog @ np.array([pnt[0], pnt[1], 1])
    #     pnt_proj = pnt_proj / pnt_proj[2]

    im = np.zeros((720, 1280, 3), dtype=np.uint8)
    grid = (np.arange(0, 20, 0.1), np.arange(-5, 5, 0.2))

    draw_projected_grid(im, grid, homog)