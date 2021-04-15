from load_data import *
import numpy as np

def read_region(region, pixel_list, path):

    h,w = region.shape
    track_angles = [[[] for _ in range(w)] for _ in range(h)]

    print(f"[a,b]: {len(track_angles)},{len(track_angles[0])}")
    
    for i in data_points(path):
        ts,x,y,p = i

        # TODO: calculate angle
        theta = 5
        
        if region[y,x] == 0:
            continue
        
        track_angles[y][x].append(theta)

    angle_mean_map = np.zeros(region.shape, dtype=float)
    for p in pixel_list:
        angle_list = track_angles[p[0]][p[1]]
        angle_mean_map[p] = sum(angle_list) / len(angle_list)
    return angle_mean_map
