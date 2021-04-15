from load_data import *
import numpy as np

def get_image(path, delta_timestamp, shape):

    ts_next = -np.inf
    im = np.zeros(shape)

    for i in data_points(path):
        ts, u, v, p = i
        
        # Update image
        im[v,u] = 1 if p==1 else 0
                
        if ts > ts_next:
            # Return image
            ts_next = ts + delta_timestamp
            yield im


def get_fancy_image(path, delta_timestamp, fade_step, shape):

    ts_next = -np.inf
    im = np.zeros(shape)

    for i in data_points(path):
        ts, u, v, p = i
        
        # Update image
        if p==1:
            im[v,u] = 1
                
        if ts > ts_next:
            # Return image
            ts_next = ts + delta_timestamp
            yield im
            im -= fade_step
            im = np.maximum(im, 0)
