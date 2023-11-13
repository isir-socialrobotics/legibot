import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


def read_frame_file():
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