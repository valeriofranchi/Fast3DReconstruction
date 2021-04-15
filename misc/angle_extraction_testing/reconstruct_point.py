import numpy as np

def uvtheta_to_point(u, v, theta, lcy, lcz, fx, fy, cx, cy):
    tan_theta = np.tan(theta)
    tan_phi = (cy - v) / fy
    
    z = (lcy - lcz * tan_theta) / (tan_phi - tan_theta)

    x = z * (cx - u) / fx
    y = z * (cy - v) / fy

    return (x, y, z)
