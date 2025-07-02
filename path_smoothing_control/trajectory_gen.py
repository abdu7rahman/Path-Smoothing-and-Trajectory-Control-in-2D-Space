def generate_trajectory(smoothed_path, velocity=0.1):
    trajectory = []
    total_time = 0.0
    for i in range(1, len(smoothed_path)):
        x0, y0 = smoothed_path[i - 1]
        x1, y1 = smoothed_path[i]
        dist = ((x1 - x0)**2 + (y1 - y0)**2)**0.5
        dt = dist / velocity
        total_time += dt
        trajectory.append((x1, y1, total_time))
    return trajectory
