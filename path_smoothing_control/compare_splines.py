import matplotlib.pyplot as plt
import numpy as np
from .smooth_bsplines import bspline_path
from .smooth_cubic import cubic_spline_path
from .obstacle_utils import filter_path, obstacles
import math

waypoints = [(0, 0), (1, 0), (1, 1), (0.5, 2), (0, 2)]

def path_length(path):
    return sum(math.hypot(x2 - x1, y2 - y1)
               for (x1, y1), (x2, y2) in zip(path[:-1], path[1:]))

def average_curvature(path):
    curvatures = []
    for i in range(1, len(path) - 1):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]
        x3, y3 = path[i + 1]

        a = np.array([x2 - x1, y2 - y1])
        b = np.array([x3 - x2, y3 - y2])

        norm_a = np.linalg.norm(a)
        norm_b = np.linalg.norm(b)
        if norm_a == 0 or norm_b == 0:
            continue

        angle = np.arccos(np.clip(np.dot(a, b) / (norm_a * norm_b), -1.0, 1.0))
        curvature = angle / norm_a
        curvatures.append(curvature)

    return sum(curvatures) / len(curvatures) if curvatures else 0

def main():
    b_path_raw = bspline_path(waypoints)
    c_path_raw = cubic_spline_path(waypoints)

    b_path = filter_path(b_path_raw)
    c_path = filter_path(c_path_raw)

    b_path_np = np.array(b_path)
    c_path_np = np.array(c_path)
    wpts_np = np.array(waypoints)

    plt.figure(figsize=(8, 6))
    plt.plot(b_path_np[:, 0], b_path_np[:, 1], label='B-spline', linewidth=2)
    plt.plot(c_path_np[:, 0], c_path_np[:, 1], label='Cubic spline', linewidth=2)
    plt.plot(wpts_np[:, 0], wpts_np[:, 1], 'ko--', label='Waypoints')

    for (ox, oy, r) in obstacles:
        circle = plt.Circle((ox, oy), r, color='gray', fill=True, alpha=0.3, label='Obstacle')
        plt.gca().add_patch(circle)

    plt.title("Trajectory Smoothing with Obstacle Avoidance")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.savefig("spline_comparison.png")
    plt.show()

    print("📈 Path Metrics (Obstacle-Aware):")
    print(f"B-spline: length = {path_length(b_path):.2f}, avg curvature = {average_curvature(b_path):.4f}, points = {len(b_path)}")
    print(f"Cubic   : length = {path_length(c_path):.2f}, avg curvature = {average_curvature(c_path):.4f}, points = {len(c_path)}")

if __name__ == '__main__':
    main()
