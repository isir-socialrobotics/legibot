#!/usr/bin/env python

from datetime import datetime
import os

import argparse
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
# from legibot.optical_flow.raft_optical_flow import RaftModel #, vizualize_optical_flow


class ROSVideoRecorder:
    def __init__(self, args, topic: str, output_path: str, fps: int = 25):
        self.topic = topic
        self.output_path = output_path
        self.fps = fps
        self.is_recording = False
        # self.raft_optical_flow_model = RaftModel(args)

        self.writer = None
        self.image_size = None
        self.last_image = None
        self.image_sub = rospy.Subscriber(self.topic + '/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(self.topic + '/camera_info', CameraInfo, self.camera_info_callback)
        # self.flow_image_pub = rospy.Publisher(self.topic + '/optical_flow', Image, queue_size=1)

    def optical_flow(self, image1, image2):
        blank_im = np.zeros_like(image1[:, :, 0])

        flow_up, flow_low = self.raft_optical_flow_model(image1, image2)
        flow_im = np.stack([flow_up[:, :, 0], flow_up[:, :, 1], blank_im], axis=2)

        # image1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        # image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
        # flow_im = cv2.calcOpticalFlowFarneback(image1, image2, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        # flow_im = np.stack([flow_im[:, :, 0], flow_im[:, :, 1], blank_im], axis=2)

        # vizualize_optical_flow(image1, flow_im)
        cv2.imshow('flow_im', flow_im)
        cv2.waitKey(1)
        return flow_im

    def image_callback(self, msg: Image):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if self.writer is None and self.image_size is not None:
            self.writer = cv2.VideoWriter(self.output_path, cv2.VideoWriter_fourcc(*'XVID'), self.fps, self.image_size)

        if self.is_recording and self.writer is not None:
            self.writer.write(image)

        # if self.last_image is not None:
        #     flow_im = self.optical_flow(self.last_image, image)
        #     self.flow_image_pub.publish(Image(data=flow_im.tobytes(),
        #                                       width=flow_im.shape[1], height=flow_im.shape[0], encoding='bgr8'))
        self.last_image = image
        # print("Recorder: Recording" if self.is_recording else "Recorder: Waiting for start...")

        cv2.imshow('image', image)
        k = cv2.waitKey(1)
        if k == ord('q'):
            self.stop()

    def camera_info_callback(self, msg):
        self.image_size = (msg.width, msg.height)
        self.camera_info_sub.unregister()

    def start(self):
        self.is_recording = True

    def stop(self):
        self.is_recording = False
        self.writer.release()
        print("Recorder: Done!")


if __name__ == "__main__":
    recorder = None
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--model', help="restore checkpoint",
                            default=os.path.join(os.path.dirname(__file__), '..', 'weights', 'raft-things.pth'))
        parser.add_argument('--small', action='store_true', help='use small model')
        parser.add_argument('--mixed_precision', action='store_true', help='use mixed precision')
        parser.add_argument('--alternate_corr', action='store_true', help='use efficent correlation implementation')
        # parser.add_argument('--source', help="input video file or folder")
        args = parser.parse_known_args()[0]
        # args = parser.parse_args()

        rospy.init_node('record_observer')
        datatime_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        try:
            target_dir = os.path.join(os.path.expanduser("~"), "Videos", "legibot")
            if not os.path.exists(target_dir):
                os.makedirs(target_dir)
            recorder = ROSVideoRecorder(args, '/observer/camera', os.path.join(target_dir, f'observer-{datatime_str}.avi'))

            recorder.start()
            rospy.spin()
        except KeyboardInterrupt:
            print("Recorder: KeyboardInterrupt")

    except Exception as e:
        print("record_observer.py - Error:", e)

    if recorder is not None:
        recorder.stop()
