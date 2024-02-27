import cv2
import sys
import os

inp_video = sys.argv[1]

cap = cv2.VideoCapture(inp_video)
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

print(f"Frame count: {frame_count}, FPS: {fps}")
cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

pause = False
frame_idx = 0
im_size = []
while True:
    if not pause:
        ret, frame = cap.read()
        frame_idx += 1
        if not ret:
            break
        elif len(im_size) == 0:
            im_size = frame.shape[:2]
        cv2.putText(frame, f"Frame: {frame_idx}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow('frame', frame)
    # else:
    #     pass

    key = cv2.waitKey(2)
    if key == 27:
        break
    elif key == 32:
        pause = not pause
        if not pause:
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)

    elif key == 81: # left arrow
        frame_idx = max(0, frame_idx - 1)
        print("Frame index:", frame_idx)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()


    elif key == 83: # right arrow
        frame_idx = min(frame_count, frame_idx + 1)
        print("Frame index:", frame_idx)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()


frame_idx_start = 70
frame_idx_end = 655

cap = cv2.VideoCapture(inp_video)
frame_idx = 0
while frame_idx < frame_idx_start:
    ret, frame = cap.read()
    frame_idx += 1

for part in range(1, 4):
    out_video = f"{inp_video[:-4]}_{part*25}.avi"
    wrt = cv2.VideoWriter(out_video, cv2.VideoWriter_fourcc(*'XVID'), fps * 2, im_size[::-1])
    # cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx_start)
    end_frame = frame_idx_start + int(part * (frame_idx_end - frame_idx_start) / 4)
    for i in range(frame_idx_start, end_frame):
        ret, frame = cap.read()
        if not ret:
            print("Frame read error")
            break
        wrt.write(frame)
        cv2.imshow('frame', frame)
        cv2.waitKey(5)
    wrt.release()
    frame_idx_start = end_frame

    # convert avi to mp4
    if os.path.exists(out_video.replace('.avi', '.mp4')):
        os.remove(out_video.replace('.avi', '.mp4'))
    os.system(f"ffmpeg -i {out_video} -vcodec libx264 -crf 28 {out_video.replace('.avi', '.mp4')}")
    print("Recorder: Video compressed to", out_video.replace('.avi', '.mp4'))
