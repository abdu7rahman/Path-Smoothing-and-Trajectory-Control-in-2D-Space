import numpy as np
from scipy.interpolate import CubicSpline

def cubic_spline_path(waypoints, resolution=100):
    x, y = zip(*waypoints)
    t = np.arange(len(x))
    t_new = np.linspace(0, len(x)-1, resolution)
    cs_x = CubicSpline(t, x)
    cs_y = CubicSpline(t, y)
    return list(zip(cs_x(t_new), cs_y(t_new)))
