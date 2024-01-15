import os.path
import sys
import torch
import argparse
import cv2
import numpy as np

from legibot.utils.basic_math import pairwise_intersections
# from scipy.optimize._lsq.dogbox import find_intersection
sys.path.append('/home/javad/workspace/RAFT/core')  # fixme
from raft import RAFT
from utils import flow_viz
from utils.utils import InputPadder


class RaftModel:
    DEVICE = 'cuda'

    def __init__(self, args):
        self.model = torch.nn.DataParallel(RAFT(args))
        self.model.load_state_dict(torch.load(args.model))

        self.model = self.model.module
        self.model.to(self.DEVICE)
        self.model.eval()

    def __call__(self, image1_np, image2_np, iters=20, test_mode=True):
        image1_torch = torch.from_numpy(image1_np).permute(2, 0, 1).float()
        image1_torch = image1_torch[None].to(self.DEVICE)
        image2_torch = torch.from_numpy(image2_np).permute(2, 0, 1).float()
        image2_torch = image2_torch[None].to(self.DEVICE)

        padder = InputPadder(image1_torch.shape)
        image1_torch, image2_torch = padder.pad(image1_torch, image2_torch)

        with torch.no_grad():
            flow_low, flow_up = self.model(image1_torch, image2_torch, iters=iters, test_mode=test_mode)

        flow_low_np = flow_low[0].permute(1, 2, 0).cpu().numpy()
        flow_up_np = flow_up[0].permute(1, 2, 0).cpu().numpy()
        return flow_up_np, flow_low_np

    def segment_of(self, of_img):
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help="restore checkpoint")
    parser.add_argument('--small', action='store_true', help='use small model')
    parser.add_argument('--mixed_precision', action='store_true', help='use mixed precision')
    parser.add_argument('--alternate_corr', action='store_true', help='use efficent correlation implementation')
    parser.add_argument('--source', help="input video file or folder")
    args = parser.parse_args()

    raft_model = RaftModel(args)
    if os.path.isfile(args.source):
        cap = cv2.VideoCapture(args.source)
        last_frame = None
        frame_counter = 0
        while True:
            ret, frame = cap.read()
            # resize to max 640x480
            frame_counter += 1
            if not ret:
                break
            if frame_counter % 2 == 0:
                continue

            if last_frame is not None:
                frame = cv2.resize(frame, (640, 480))
                flow_up, flow_low = raft_model(frame, last_frame)

                frame_down = cv2.resize(frame, (flow_low.shape[1], flow_low.shape[0]))
                last_frame_down = cv2.resize(last_frame, (flow_low.shape[1], flow_low.shape[0]))
                subtracted = cv2.absdiff(frame_down, last_frame_down)
                subtracted = cv2.cvtColor(subtracted, cv2.COLOR_BGR2GRAY)
                subtracted = cv2.erode(subtracted, np.ones((2, 2), np.uint8), iterations=1)
                subtracted = cv2.dilate(subtracted, np.ones((5, 5), np.uint8), iterations=1)
                subtracted = cv2.resize(subtracted, (flow_up.shape[1], flow_up.shape[0]))
                # subtracted = cv2.erode(subtracted, np.ones((3, 3), np.uint8), iterations=1)
                subtracted = cv2.threshold(subtracted, 15, 255, cv2.THRESH_BINARY)[1]

                flo = flow_viz.flow_to_image(flow_up)
                flo = cv2.bitwise_and(flo, flo, mask=subtracted)
                flow_up_mag = np.linalg.norm(flow_up, axis=2)
                flow_nz = np.nonzero(cv2.threshold(flow_up_mag * subtracted, 128, 1, cv2.THRESH_BINARY)[1])
                # flow_nz =
                if len(flow_nz[0]) > 0:
                    flow_nz = np.stack(flow_nz, axis=1)
                    samples_inds = np.random.choice(range(len(flow_nz)), min(40, len(flow_nz)), replace=False)
                    flow_nz = flow_nz[samples_inds]
                    # flow_nz = random.sample(flow_nz[0], min(20, len(flow_nz[0])))
                    lines = []
                    for i in range(len(flow_nz)):
                        lines.append([flow_nz[i, 1],
                                      flow_nz[i, 0],
                                      flow_up[flow_nz[i, 0], flow_nz[i, 1], 0],
                                      flow_up[flow_nz[i, 0], flow_nz[i, 1], 1]
                                      ])
                        cv2.line(frame, (flow_nz[i, 1], flow_nz[i, 0]),
                                    (flow_nz[i, 1] - int(flow_up[flow_nz[i, 0], flow_nz[i, 1], 0] * 10),
                                    flow_nz[i, 0] - int(flow_up[flow_nz[i, 0], flow_nz[i, 1], 1] * 10)),
                                    (0, 255, 0), 1)
                    intersections = pairwise_intersections(lines)
                    for intersect in intersections:
                        cv2.circle(frame, (int(intersect[0]), int(intersect[1])), 5, (0, 0, 255), -1)
                    intersections_mean = np.mean(intersections, axis=0)

                img_flo = np.concatenate([frame, flo[:,:,::-1]], axis=0)
                cv2.imshow('image', img_flo[:, :, :] / 255.0)
                k = cv2.waitKey(2)
                # if ESC pressed, exit loop
                if k == 27:
                    break
                elif k == 32:  # SPACE pressed, pause video
                    cv2.waitKey(0)

            last_frame = frame



