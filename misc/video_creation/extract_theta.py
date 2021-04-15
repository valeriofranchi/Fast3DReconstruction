import numpy as np

def get_theta(lcy, lcz, cy, fy, dz, v):
    tan_phi = (cy - v) / fy
    dy = dz * tan_phi
    theta = np.arctan2(dy - lcy, dz - lcz)
    return theta
