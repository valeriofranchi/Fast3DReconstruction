from load_data import *
from read_region import *
import numpy as np
import cv2

import winsound

path = "./../../code/data/1hzangle_td.dat"

region = np.ones((360,480))
#region[360//3:360//2, 480//6:480//3] = 1
cv2.imshow("region", region)
cv2.waitKey(0)

print("Making pixel list")

pixel_list = []
for y in range(region.shape[0]):
    for x in range(region.shape[1]):
        if region[y,x]:
            pixel_list.append((y,x))

print("Reading region")

v_mean_map = read_region(region, pixel_list, path, 339)

winsound.MessageBeep()

print(v_mean_map)
v_max = max(v_mean_map.flatten())

print("Displaying")

im = v_mean_map / v_max

cv2.imshow("v_mean", im)
cv2.waitKey(0)
print("Saving")
cv2.imwrite("v_mean_new.png",im*255)

