#!/usr/bin/env python
import os
import cv2
import numpy as np

from legibot.legibility_score import calc_legibility
from legibot.projection_utils import make_grid, make_homog, draw_projected_grid, project_points
from vive_ai.logger.logger_factory import logger
from legibot.colors import BGR_RED, BGR_GREEN, BGR_BLUE, BGR_YELLOW, BGR_WHITE, BGR_BLACK, BGR_ORANGE

from legibot.capture_input import read_frame_file
from legibot.cv_utils import capture_click
from legibot.kf_tracking import KalmanFilter2D
from legibot.opf_utils import background_subtraction_mask, detect_objects

frame_index = -1
output_dir = ""


frame = None
prvs = None
flow = None
projection_grid = make_grid()
projection_homog, projection_homog_inv = make_homog()

robot_tracker = KalmanFilter2D()
robot_pred_tracker = KalmanFilter2D()

while True:
    frame = read_frame_file()  # read source frame
    # frame = read_frame_ros()
    if frame is None: continue
    frame_index += 1

    next = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    if prvs is None:
        prvs = next
        continue

    flags = [0, cv2.OPTFLOW_USE_INITIAL_FLOW][flow is not None]
    flow = cv2.calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, flags)
    mask = background_subtraction_mask(prvs, next)
    flow = cv2.bitwise_and(flow, flow, mask=mask)

    prvs = next

    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    flow_hsv = np.ones_like(frame) * 100
    flow_hsv[..., 0] = ang * 180 / np.pi / 2
    flow_hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
    flow_bgr = cv2.cvtColor(flow_hsv, cv2.COLOR_HSV2BGR)

    # detect objects (cluster of pixels) in the image
    object_contours = detect_objects(flow_bgr)

    if len(object_contours) > 0:
        Robot_CoM = np.mean(object_contours[0], axis=0)[0]  # Center of Mass
        # Object Ground Point
        # Robot_GCP = min(object_contours[0], key=lambda x: x[0, 1])[0]  #  Ground contact point
        det_object_pos, det_object_vel = robot_tracker.update(Robot_CoM)  # is needed to update KF and robot trajectory
        robot_track_world = project_points(robot_tracker.trajectory, projection_homog_inv)

        # Direction of Motion (in image frame)
        DoM = np.mean(flow[int(Robot_CoM[1]) - 5:int(Robot_CoM[1]) + 5, int(Robot_CoM[0]) - 5:int(Robot_CoM[0]) + 5], axis=(0, 1))
        pred_x, pred_y = Robot_CoM[0] + DoM[0] * 10, Robot_CoM[1] + DoM[1] * 10
        robot_pred_tracker.update((pred_x, pred_y))

        # project potential goals to world coordinates
        robot_pred_track_world = project_points(robot_pred_tracker.trajectory, projection_homog_inv)
        logger.polygon(robot_pred_track_world, color=BGR_RED, legend='robot_track_world')

        potential_goals_uv = [(0.19, 0.39), (0.43, 0.68), (0.74, 0.36)]  # (x, y) normalized
        potential_goals_uv = [(int(g[0] * frame.shape[1]), int(g[1] * frame.shape[0])) for g in potential_goals_uv]
        potential_goals_world = project_points(potential_goals_uv, projection_homog_inv)

        dist_to_goals = [np.linalg.norm(np.array(g) - robot_track_world[-1]) for g in potential_goals_world]
        argmin_dist = np.argmin(dist_to_goals)
        legib_values = calc_legibility(potential_goals_world, robot_track_world)
        logger.plot(legib_values[0], legend='g1', topic='/legibility/g1')

        # ======================= VISUALIZATION =======================
        flow_bgr = 255 - flow_bgr
        # draw flow vectors
        step = 20
        opf_scale = 2
        for i in range(0, flow.shape[0], step):
            for j in range(0, flow.shape[1], step):
                if mask[i, j] == 0: continue
                cv2.arrowedLine(flow_bgr, (j, i), (int(j + flow[i, j, 0] * opf_scale),
                                                   int(i + flow[i, j, 1] * opf_scale)), (0, 0, 255), 1)
        cv2.arrowedLine(flow_bgr, (int(Robot_CoM[0]), int(Robot_CoM[1])), (int(pred_x), int(pred_y)), (0, 255, 0), 3)

        # draw bbox around detected objects
        for cnt in object_contours:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), BGR_RED, 2)

        cv2.circle(frame, (int(Robot_CoM[0]), int(Robot_CoM[1])), 5, (200, 0, 200), -1)
        cv2.arrowedLine(frame, (int(Robot_CoM[0]), int(Robot_CoM[1])), (int(pred_x), int(pred_y)), (0, 255, 0), 2)

        for ii, g in enumerate(potential_goals_uv):
            # draw arrow from Robot_CoM to goal
            color = BGR_ORANGE
            if ii == argmin_dist:
                color = (235, 206, 135)
            if len(object_contours) > 0:
                cv2.arrowedLine(frame, (int(Robot_CoM[0]), int(Robot_CoM[1])), g, (100, 200, 50), 2)
                cv2.circle(frame, g, 25, color, 5)
                cv2.putText(frame, f"{legib_values[ii]:.2f}", g,
                            cv2.FONT_HERSHEY_SIMPLEX, 1, BGR_WHITE, 2)
        # =============================================================

    # ================== VISUALIZATION ==================
    draw_projected_grid(frame, projection_grid, projection_homog, BGR_GREEN)
    cv2.imshow('flow', flow_bgr)
    cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('frame', capture_click, (projection_homog_inv,))
    cv2.imshow('frame', frame)
    cv2.imwrite(os.path.join(output_dir, f"frame-{frame_index}.png"), frame)
    cv2.imwrite(os.path.join(output_dir, f"flow-{frame_index}.png"), flow_bgr)
    # ====================================================
    k = cv2.waitKey(20) & 0xff
    if k == 27:  # ESC
        break
    elif k == 32:  # Space => pause
        cv2.waitKey(0)


cv2.waitKey(0)
