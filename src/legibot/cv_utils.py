import cv2
from legibot.projection_utils import project_points


def capture_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pnts = [[x, y]]
        pnts_proj = project_points(pnts, param[0])
        print(f"{pnts} => {pnts_proj}")