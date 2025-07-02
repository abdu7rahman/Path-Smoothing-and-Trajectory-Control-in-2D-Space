import math

def is_point_near_obstacle(point, obstacles, buffer=0.1):
    px, py = point
    for ox, oy, r in obstacles:
        dist = math.hypot(px - ox, py - oy)
        if dist <= r + buffer:
            return True
    return False

def filter_path(path, obstacles, buffer=0.1):
    """Remove points that are too close to any obstacle."""
    return [pt for pt in path if not is_point_near_obstacle(pt, obstacles, buffer)]
