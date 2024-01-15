import glob

from PIL import Image


def make_gif(frame_filenames, gif_filename):
    frames = [Image.open(image) for image in frame_filenames]
    frame_one = frames[0]
    frame_one.save(f"{gif_filename}.gif", format='GIF', append_images=frames[1:],
                   save_all=True, duration=300, loop=0)


if __name__ == '__main__':
    frames = glob.glob("/home/javad/workspace/legibot/out/20240115-121518-*.png")
    frames.sort()
    make_gif(frames, "/home/javad/workspace/legibot/out/20240115-121518")
