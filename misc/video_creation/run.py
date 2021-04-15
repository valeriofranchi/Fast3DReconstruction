from event_to_images import *
import numpy as np
import cv2
import pickle
import imageio

import winsound
pickle_file = "pickle_dat.p"
"""
path = "./../../code/data/1hzcannm2_td.dat"
delta_ts = 1000
shape = (360,480)

ims = []

print("Getting images")

for im in get_fancy_image(path, delta_ts, 0.01, shape):
    #cv2.imshow("frame", im)
    #key = cv2.waitKey(1) & 0xFF
    #if key == ord("q") or key == ord("Q"):
    #    break
    ims.append((im*255).astype('uint8'))
cv2.destroyAllWindows()

winsound.MessageBeep()

print(f"Pickling {len(ims)} images")


pickle.dump(ims, open(pickle_file, "wb"))
"""
print("Unpickling images")

ims = pickle.load(open(pickle_file, "rb"))

print("Writing data out")
"""
video_path = "fade.avi"
shape = (ims[0].shape[1], ims[1].shape[0])
fps = 60
out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'DIVX'), fps, shape)
for im in ims:
    im_frame = cv2.cvtColor(im,cv2.COLOR_GRAY2RGB)
    out.write(im_frame)
out.release()
"""
gif_path = "fade.gif"
imageio.mimsave(gif_path, ims, fps=55)

