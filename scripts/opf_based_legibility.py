import cv2
import numpy as np
import os
# import pykalman
from filterpy.kalman import KalmanFilter
from legibot.legibility_core import calc_legibility
from vive_ai.logger.logger_factory import logger


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
        print(x, y)


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


# video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-10-30_19-04-07.avi")
# video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-10-31_14-32-19.avi")
# video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-10-31_15-35-32.avi")
# video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-11-02_12-57-36.avi")
video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-11-02_14-41-42.avi")  # legible
# video_source = os.path.join("/home/javad/Videos/legibot", "observer-2023-11-02_14-50-21.avi")  # illegible
output_dir = os.path.join("/home/javad/workspace/catkin_ws/src/legibot", "output", os.path.basename(video_source)[0:-4])
if not os.path.exists(output_dir): os.makedirs(output_dir)
cap = cv2.VideoCapture(video_source)

prvs = None
flow = None

object_tracker = KalmanFilter2D()

frame_index = 0
while True:
    ret, frame2 = cap.read()
    frame_index += 1
    if not ret: break
    if frame_index % 8 != 0: continue
    next = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    next = cv2.GaussianBlur(next, (5, 5), 0)
    if prvs is None:
        prvs = next
        continue

    flags = [0, cv2.OPTFLOW_USE_INITIAL_FLOW][flow is not None]
    flow = cv2.calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, flags)

    # mask = consistent_optical_flow(prvs, next)
    mask = background_subtraction_mask(prvs, next)
    flow = cv2.bitwise_and(flow, flow, mask=mask)
    prvs = next

    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    flow_hsv = np.zeros_like(frame2)
    flow_hsv[..., 0] = ang * 180 / np.pi / 2
    flow_hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
    flow_hsv[..., 1] = 100
    flow_bgr = cv2.cvtColor(flow_hsv, cv2.COLOR_HSV2BGR)

    # detect objects (cluster of pixels) in the image
    object_contours = detect_objects(flow_bgr)
    for cnt in object_contours:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame2, (x, y), (x + w, y + h), (0, 255, 0), 2)

    flow_bgr = 255 - flow_bgr
    if len(object_contours) > 0:
        CoM = np.mean(object_contours[0], axis=0)  # center of mass
        # direction of motion
        DoM = np.mean(flow[int(CoM[0, 1]) - 5:int(CoM[0, 1]) + 5, int(CoM[0, 0]) - 5:int(CoM[0, 0]) + 5], axis=(0, 1))
        pred_x, pred_y = CoM[0, 0] + DoM[0] * 10, CoM[0, 1] + DoM[1] * 10

        cv2.circle(frame2, (int(CoM[0, 0]), int(CoM[0, 1])), 5, (200, 0, 200), -1)
        cv2.arrowedLine(frame2, (int(CoM[0, 0]), int(CoM[0, 1])), (int(pred_x), int(pred_y)), (0, 255, 0), 2)
        cv2.arrowedLine(flow_bgr, (int(CoM[0, 0]), int(CoM[0, 1])), (int(pred_x), int(pred_y)), (0, 255, 0), 3)

        x, v = object_tracker.update(CoM)
        # cv2.circle(frame2, (int(x[0]), int(x[1])), 5, (40, 210, 150), -1)
        # cv2.arrowedLine(frame2, (int(x[0]), int(x[1])), (int(x[0] + v[0] * 10), int(x[1] + v[1] * 10)),
        #                 (0, 255, 255), 2)

    # draw sampled flow vectors sparsely
    step = 20
    opf_scale_factor = 2
    for i in range(0, flow.shape[0], step):
        for j in range(0, flow.shape[1], step):
            if mask[i, j] == 0: continue
            cv2.arrowedLine(flow_bgr, (j, i), (int(j + flow[i, j, 0] * opf_scale_factor),
                                               int(i + flow[i, j, 1] * opf_scale_factor)),
                            (0, 0, 255), 1)

    # project potential goals to world coordinates
    # cv2.projectPoints()

    # fixme
    potential_goals_uv = [(0.19, 0.39), (0.43, 0.68), (0.74, 0.36)]  # (x, y) normalized
    potential_goals_uv = [(int(g[0] * frame2.shape[1]), int(g[1] * frame2.shape[0])) for g in potential_goals_uv]

    dist_to_goals = [np.linalg.norm(np.array(g) - CoM[0]) for g in potential_goals_uv]
    argmin_dist = np.argmin(dist_to_goals)
    legib_values = calc_legibility(potential_goals_uv, object_tracker.trajectory)
    logger.plot(legib_values[0], legend='g1', topic='/legibility/g1')
    for ii, g in enumerate(potential_goals_uv):
        # draw arrow from CoM to goal
        color = (0, 165, 255)
        if ii == argmin_dist:
            color = (235, 206, 135)
        if len(object_contours) > 0:
            cv2.arrowedLine(frame2, (int(CoM[0, 0]), int(CoM[0, 1])), g, (100, 200, 50), 2)
            cv2.circle(frame2, g, 25, color, 5)
            cv2.putText(frame2, f"{legib_values[ii]:.2f}", g,
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    cv2.imshow('flow', flow_bgr)
    cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('frame', capture_click)
    cv2.imshow('frame', frame2)
    cv2.imwrite(os.path.join(output_dir, f"frame-{frame_index}.png"), frame2)
    cv2.imwrite(os.path.join(output_dir, f"flow-{frame_index}.png"), flow_bgr)
    k = cv2.waitKey(20) & 0xff
    if k == 27:
        break
    # Space => pause
    elif k == 32:
        cv2.waitKey(0)


cv2.waitKey(0)
