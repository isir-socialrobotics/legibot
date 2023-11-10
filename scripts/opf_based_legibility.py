#!/usr/bin/env python

import cv2
import numpy as np
import os

import rospy
# import pykalman
from filterpy.kalman import KalmanFilter
from sensor_msgs.msg import Image

from legibot.legibility_core import calc_legibility
from legibot.projection_api import make_grid, make_homog, draw_projected_grid, project_points
from vive_ai.logger.logger_factory import logger
from vive_ai.consts.consts import BGR_RED, BGR_GREEN, BGR_BLUE, BGR_YELLOW, BGR_WHITE, BGR_BLACK, BGR_ORANGE


def consistent_optical_flow(frame1, frame2):
    flow_forward = cv2.calcOpticalFlowFarneback(frame1, frame2, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    flow_backward = cv2.calcOpticalFlowFarneback(frame2, frame1, None, 0.5, 3, 15, 3, 5, 1.2, 0)

    # Define a consistency threshold (you can adjust this value)
    threshold = 2.0  # Adjust as needed

    # Initialize an empty mask to mark inconsistent flow vectors
    # Calculate the Euclidean distance between flow vectors
    distances = np.linalg.norm(flow_forward - flow_backward, axis=2)

    # Create an inconsistency mask based on the threshold
    inconsistent_mask = (distances > threshold).astype(np.uint8) * 255

    # cv2.imshow('mask', inconsistent_mask)
    # updated_flow = flow_forward * inconsistent_mask[:, :, np.newaxis]

    return inconsistent_mask


def background_subtraction_mask(frame1, frame2):
    # blur
    blur1 = cv2.GaussianBlur(frame1, (5, 5), 0)
    blur2 = cv2.GaussianBlur(frame2, (5, 5), 0)

    diff = cv2.absdiff(blur1, blur2)
    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
    kernel = np.ones((10, 10), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=3)

    # cv2.imshow('mask-bg', dilated)
    return dilated


def detect_objects(flow_im):
    gray = cv2.cvtColor(flow_im, cv2.COLOR_BGR2GRAY)
    # gray = cv2.erode(gray, np.ones((2, 2), np.uint8), iterations=1)
    gray = cv2.dilate(gray, np.ones((7, 7), np.uint8), iterations=1)
    ret_, thresh = cv2.threshold(gray, 20, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > 100]
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    return contours[:1]


def capture_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pnts = [[x, y]]
        pnts_proj = project_points(pnts, param[0])
        print(f"{pnts} => {pnts_proj}")


class KalmanFilter2D:
    def __init__(self, dt=1.0):
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.zeros((4,))
        self.kf.F = np.array([[1., 0., dt, 0.],
                             [0., 1., 0., dt],
                             [0., 0., 1., 0.],
                             [0., 0., 0., 1.]])  # state transition matrix
        self.kf.H = np.array([[1., 0., 0., 0.],
                         [0., 1., 0., 0.]])  # Measurement function
        self.kf.P *= 1000.  # covariance matrix
        self.kf.R = np.eye(2) * 5  # state uncertainty
        self.kf.Q = np.eye(4) * 0.1  # process uncertainty
        
        self.initialized = False
        self.trajectory = []

    def update(self, z):
        if not self.initialized:
            self.kf.x[:2] = z
            self.initialized = True

        self.kf.predict()
        self.kf.update(z)
        self.trajectory.append(self.kf.x[:2])
        return self.kf.x[:2], self.kf.x[2:]


cap = None
frame_index = -1
output_dir = ""
def read_frame_cap():
    global frame_index, cap, output_dir
    if cap is None:
        # video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-10-30_19-04-07.avi")
        # video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-10-31_14-32-19.avi")
        # video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-10-31_15-35-32.avi")
        # video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-11-02_12-57-36.avi")
        video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-11-02_14-41-42.avi")  # legible
        # video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-11-02_14-50-21.avi")  # illegible
        output_dir = os.path.join("/home/javad/workspace/catkin_ws/src/legibot", "output",
                                  os.path.basename(video_source)[0:-4])
        if not os.path.exists(output_dir): os.makedirs(output_dir)
        cap = cv2.VideoCapture(video_source)

    ret, frame = cap.read()
    if not ret: return None
    frame_index += 1
    return frame


def read_frame_ros():
    global frame_index, cap, output_dir
    def image_callback(msg):
        global frame_index, frame2
        frame2 = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)
        frame_index += 1
        print(f"New frame: {frame_index}")

    if cap is None:
        "/observer/camera/image_raw"
        cap = rospy.Subscriber("/observer/camera/image_raw", Image, callback=image_callback, queue_size=1)
        output_dir = os.path.join("/home/javad/workspace/catkin_ws/src/legibot", "output", "ros")
        if not os.path.exists(output_dir): os.makedirs(output_dir)

    return frame2

frame2 = None
prvs = None
flow = None
projection_grid = make_grid()
projection_homog, projection_homog_inv = make_homog()

robot_tracker = KalmanFilter2D()
robot_pred_tracker = KalmanFilter2D()

while True:
    frame2 = read_frame_cap()  # read source frame
    # frame2 = read_frame_ros()
    if frame2 is None or frame_index % 8 != 0: continue

    next = cv2.GaussianBlur(cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    if prvs is None:
        prvs = next
        continue

    flags = [0, cv2.OPTFLOW_USE_INITIAL_FLOW][flow is not None]
    flow = cv2.calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, flags)
    mask = background_subtraction_mask(prvs, next)
    flow = cv2.bitwise_and(flow, flow, mask=mask)

    prvs = next

    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    flow_hsv = np.ones_like(frame2) * 100
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
        potential_goals_uv = [(int(g[0] * frame2.shape[1]), int(g[1] * frame2.shape[0])) for g in potential_goals_uv]
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
            cv2.rectangle(frame2, (x, y), (x + w, y + h), BGR_RED, 2)

        cv2.circle(frame2, (int(Robot_CoM[0]), int(Robot_CoM[1])), 5, (200, 0, 200), -1)
        cv2.arrowedLine(frame2, (int(Robot_CoM[0]), int(Robot_CoM[1])), (int(pred_x), int(pred_y)), (0, 255, 0), 2)

        for ii, g in enumerate(potential_goals_uv):
            # draw arrow from Robot_CoM to goal
            color = BGR_ORANGE
            if ii == argmin_dist:
                color = (235, 206, 135)
            if len(object_contours) > 0:
                cv2.arrowedLine(frame2, (int(Robot_CoM[0]), int(Robot_CoM[1])), g, (100, 200, 50), 2)
                cv2.circle(frame2, g, 25, color, 5)
                cv2.putText(frame2, f"{legib_values[ii]:.2f}", g,
                            cv2.FONT_HERSHEY_SIMPLEX, 1, BGR_WHITE, 2)
        # =============================================================

    # ================== VISUALIZATION ==================
    draw_projected_grid(frame2, projection_grid, projection_homog, BGR_GREEN)
    cv2.imshow('flow', flow_bgr)
    cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('frame', capture_click, (projection_homog_inv,))
    cv2.imshow('frame', frame2)
    cv2.imwrite(os.path.join(output_dir, f"frame-{frame_index}.png"), frame2)
    cv2.imwrite(os.path.join(output_dir, f"flow-{frame_index}.png"), flow_bgr)
    # ====================================================
    k = cv2.waitKey(20) & 0xff
    if k == 27:  # ESC
        break
    elif k == 32:  # Space => pause
        cv2.waitKey(0)


cv2.waitKey(0)
