from load_data import *
import numpy as np

def read_region(region, pixel_list, path, wall_column):

    h,w = region.shape
    track_v = [[[] for _ in range(w)] for _ in range(h)]

    v_last = None
    
    for i in data_points(path):
        ts,u,v,p = i

        if u == wall_column:
            v_last = v
        
        if region[v,u] == 0:
            continue
        if v_last == None:
            continue
        
        track_v[v][u].append(v_last)

    v_mean_map = np.zeros(region.shape, dtype=float)
    for p in pixel_list:
        v_list = track_v[p[0]][p[1]]
        if len(v_list) < 5:
            pass
            #print(f"No value found for {p[1]},{p[0]}")
        else:
            v_mean_map[p] = sum(v_list) / len(v_list)
    return v_mean_map
