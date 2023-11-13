import cv2
import numpy as np


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