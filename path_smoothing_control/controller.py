import math

def get_distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def pure_pursuit(current_pose, trajectory, lookahead_dist=0.3, max_lin=0.22, max_ang=2.84):
    x, y, theta = current_pose

    for target in trajectory:
        dist = get_distance((x, y), (target[0], target[1]))
        if dist >= lookahead_dist:
            goal = target
            break
    else:
        goal = trajectory[-1]

    dx = goal[0] - x
    dy = goal[1] - y
    goal_angle = math.atan2(dy, dx)
    alpha = goal_angle - theta

    alpha = math.atan2(math.sin(alpha), math.cos(alpha))

    linear_vel = min(max_lin, lookahead_dist)
    angular_vel = max(-max_ang, min(max_ang, 2.0 * math.sin(alpha) / lookahead_dist))

    return linear_vel, angular_vel
