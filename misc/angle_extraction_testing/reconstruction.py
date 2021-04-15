import numpy as np

"""
Required constant parameters

self.c_x        - Centre pixel x
self.c_y        - Centre pixel y

self.l_cy       - Offset between camera and laser in y-direction
self.l_cz       - Offset between camera and laser in z-direction

self.f_x        - Focal length of camera in x-direction
self.f_y        - Focal length of camera in y-direction
"""

"""
Reconstructs a line of excited pixels given the laser angle.
line:  numpy array of size (width), of excited pixel heights (may be NaN)
theta: angle of the laser plane
returns [x y z] numpy.ndarray of points in 3D space
Note - no filtering for impossible depths is performed
"""
def reconstruct_line(line, theta):
    # Obtain points in image coordinates
    u = np.arange(0, len(line))
    v = line

    # Calculate tan of theta and phi
    tan_theta = np.tan(theta)
    tan_phi = (self.c_y - v) / self.f_y

    # Calculate z offset
    z = (self.l_cy - self.l_cz * tan_theta) / (tan_phi - tan_theta)
    # Calculate x and y offset
    x = np.multiply(z, (self.c_x - u) / self.f_x)
    y = np.multiply(z, (self.c_y - v) / self.f_y)

    # Return [x, y, z] columns
    return np.vstack((x, y, z)).T

