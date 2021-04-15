import numpy as np
from load_data import *
import cv2
import matplotlib.pyplot as plt


def angle_from_v(v):
    cy = 168.3
    fy = 419.6
    dz = 0.26
    lcy = -0.135
    lcz = 0.150
    tan_phi = (cy - v) / fy
    dy = dz * tan_phi
    theta = np.arctan2(dy - lcy, dz - lcz)
    return theta


print("Starting")
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
raw_xs = []
raw_ys = []
print("Displaying empty plot")
plt.ion()
plt.show()
column = 270

print("Setting up scan display")
w = 480
h = 360

im_disp = np.zeros((h,w), dtype=float)

data_path = "./../../code/data/thetacalc1hzlowsen_td.dat"

ts_display_next = -np.inf
ts_display_gap = 1e4

# Set up filter
tracking_columns = np.zeros((w))
tracking_columns[260:300] = 1

block_current_start = 0
block_time_us = 5e3         # us, expected worst-case is 5ms/200Hz
block_x_current = []        # Track block, then take mean once block time hit
block_y_current = []
block_xs = []               # Block mean values to plot
block_ys = []

print("Starting loop")

for p in data_points(data_path):
    t, x, y, pol = p
    if pol == 1:
        im_disp[y, x] = 1
        theta = angle_from_v(y) / np.pi * 180
        if x == column:
            raw_xs.append(t)
            raw_ys.append(theta)
        if tracking_columns[x] == 1:
            #raw_xs.append(t)
            #raw_ys.append(y)
            if t > block_current_start + block_time_us:
                if len(block_x_current) > 0:
                    block_xs.append(sum(block_x_current) / len(block_x_current))
                    block_ys.append(sum(block_y_current) / len(block_y_current))
                    block_x_current = []
                    block_y_current = []
                    block_current_start = t + block_time_us
            block_x_current.append(t)
            block_y_current.append(theta)
    if t > ts_display_next:
        # Update frame
        ts_display_next = t + ts_display_gap
        cv2.imshow("Camera", im_disp)
        key = cv2.waitKey(10) & 0xff
        if key == ord('q') or key == ord('Q'):
            break
        elif key == ord('c'):
            # Print frame to file
            print("Writing to file")
            im_out = np.zeros((h, w, 3), dtype=np.uint8)
            im_out[:,:,0] = (im_disp*255).astype(np.uint8)
            im_out[:,:,1] = (im_disp*255).astype(np.uint8)
            im_out[:,:,2] = (im_disp*255).astype(np.uint8)
            cv2.imshow("Out", im_out)
            cv2.imwrite("camera_frame.png", im_out)
        im_disp = np.zeros((h,w))
        # Update plot
        ax.clear()
        ax.plot(raw_xs, raw_ys)
        ax.plot(block_xs, block_ys, '-')
        plt.ylabel("θ (°)")
        plt.xlabel("t (μs)")
        plt.legend(["Unfiltered", "Filtered"])
        plt.pause(1e-3)
