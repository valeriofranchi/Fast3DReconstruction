import numpy as np

"""
Required constant parameters

self.height     - Number of rows in camera
self.d          - Distance to wall
self.c_y        - Central y-pixel in camera
self.f_y        - Camera focal length in y-direction
self.l_cy       - Offset between laser and camera in y-direction
self.l_cz       - Offset between laser and camera in z-direction
"""

"""
Calculates the angle of the laser plane given a laser line and reference column
v_in:       Vertical coordinate pixel excited in image plane
returns angle of the laser plane
Note - if line at reference column is NaN, returned value will be NaN (probably)
The wall is assumed to be parallel to the camera plane
"""
def extract_theta(v_in):

    # Calculate theta
    tan_phi = (self.c_y - v_in) / self.f_y
    dy = self.d * tan_phi
    theta = np.atan2(dy - self.l_cy, self.d - self.l_cz)

    # Return theta
    return theta
