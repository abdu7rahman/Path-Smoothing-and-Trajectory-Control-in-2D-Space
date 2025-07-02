import numpy as np
from scipy.interpolate import splprep, splev

def bspline_path(waypoints, smoothing=0.0, resolution=100):
    x, y = zip(*waypoints)
    tck, _ = splprep([x, y], s=smoothing)
    u = np.linspace(0, 1, resolution)
    x_i, y_i = splev(u, tck)
    return list(zip(x_i, y_i))
