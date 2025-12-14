import numpy as np
import matplotlib.pyplot as plt


#   Potential functions

def total_potential(x, y, goal, obs_list, k_att=1.0, k_rep=1.2):
    """
    Compute total potential U(x,y):
        U = U_attractive + sum U_repulsive
    """
    # Attractive: quadratic well around the goal
    U_att = 0.5 * k_att * ((x - goal[0])**2 + (y - goal[1])**2)

    # Repulsive: 1 / distance^2 around each obstacle
    U_rep = 0.0
    for ox, oy in obs_list:
        d2 = (x - ox)**2 + (y - oy)**2
        U_rep += k_rep / (d2 + 1e-3)       # epsilon avoids division by zero

    return U_att + U_rep


def follow_gradient(start, goal, obs_list,
                    step=0.05, max_steps=1500,
                    goal_radius=0.15, force_tol=1e-3):
    """
    Gradient-descent navigation in the potential field.

    Stops either:
      • when the robot reaches the goal region, or
      • when the force (−∇U) is very small while still far from the goal
        -> interpreted as getting stuck in a local minimum.
    """
    eps = 1e-3
    x, y = start
    path = [(x, y)]

    def U(px, py):
        return total_potential(px, py, goal, obs_list)

    for _ in range(max_steps):
        # finite-difference gradient of U
        dUx = (U(x + eps, y) - U(x - eps, y)) / (2 * eps)
        dUy = (U(x, y + eps) - U(x, y - eps)) / (2 * eps)

        # virtual force = −∇U
        Fx, Fy = -dUx, -dUy
        Fmag = np.hypot(Fx, Fy)

        # distance to goal
        dist_goal = np.hypot(x - goal[0], y - goal[1])

        # 1) goal reached?
        if dist_goal < goal_radius:
            break

        # 2) local minimum: almost no force but not at goal
        if Fmag < force_tol:
            break

        # 3) take a step along the force
        x += step * Fx
        y += step * Fy
        path.append((x, y))

    return np.asarray(path)

#   Plotting utilities

def draw_potential(ax, goal, obs_list, title):
    """
    Draw background potential field (contours) plus goal and obstacles.
    """
    xs = np.linspace(-1, 6, 80)
    ys = np.linspace(-1, 6, 80)
    X, Y = np.meshgrid(xs, ys)

    U = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            U[i, j] = total_potential(X[i, j], Y[i, j], goal, obs_list)

    U_vis = np.clip(U, 0, np.percentile(U, 95))

    ax.contourf(X, Y, U_vis, levels=40, cmap='viridis')
    ax.contour(X, Y, U_vis, levels=15, colors='k', linewidths=0.3)

    # Mark goal and obstacles
    ax.plot(goal[0], goal[1], 'r*', markersize=14, label="Goal")
    for ox, oy in obs_list:
        ax.plot(ox, oy, 'ks', markersize=8, label="Obstacle")

    ax.set_xlim(-1, 6)
    ax.set_ylim(-1, 6)
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.set_xlabel("x")
    ax.set_ylabel("y")

#   Main: two scenarios (success + failure)

if __name__ == "__main__":
    start = (0.0, 0.0)
    goal  = (5.0, 1)

    # Case 1: obstacle far off the straight line -> robot reaches goal
    obstacles_ok = [(8.0, 1.0)]

    # Case 2: obstacle near the line between start and goal -> local minimum
    obstacles_bad = [(2.5, 0.5)]

    # Generate paths using gradient descent on U
    path_ok   = follow_gradient(start, goal, obstacles_ok)
    path_fail = follow_gradient(start, goal, obstacles_bad)

    # side-by-side plots
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    draw_potential(axes[0], goal, obstacles_ok,   "Scenario 1: Reaches Goal")
    draw_potential(axes[1], goal, obstacles_bad, "Scenario 2: Stuck in Local Minimum")

    # Mark start on both views
    for ax in axes:
        ax.plot(start[0], start[1], 'go', markersize=10, label="Start")

    # Animated markers and traces
    robot1, = axes[0].plot([], [], 'bo', markersize=8)
    robot2, = axes[1].plot([], [], 'bo', markersize=8)

    trace1, = axes[0].plot([], [], 'w-', linewidth=2)
    trace2, = axes[1].plot([], [], 'w-', linewidth=2)

    max_frames = max(len(path_ok), len(path_fail))

    plt.ion() 

    for k in range(max_frames):
        if k < len(path_ok):
            robot1.set_data([path_ok[k, 0]], [path_ok[k, 1]])
            trace1.set_data(path_ok[:k+1, 0], path_ok[:k+1, 1])

        if k < len(path_fail):
            robot2.set_data([path_fail[k, 0]], [path_fail[k, 1]])
            trace2.set_data(path_fail[:k+1, 0], path_fail[:k+1, 1])

        plt.pause(0.09)

    plt.ioff()
    plt.show()
