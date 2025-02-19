#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

def transform_velocity(v_mid, d_x, d_y, theta):
    """
    Transform velocity from mid-frame to side-frame.

    Parameters:
        v_mid: tuple (v_x, v_y, omega) -> Velocity of the mid-frame
        d_x: float -> X offset of side-frame from mid-frame
        d_y: float -> Y offset of side-frame from mid-frame
        theta: float -> Yaw angle of the side frame w.r.t. mid-frame (in radians)

    Returns:
        v_side: tuple (v_x_side, v_y_side, omega_side) -> Velocity of the side frame
    """
    v_x, v_y, omega = v_mid

    # Compute velocity in the global frame
    v_x_side_global = v_x - omega * d_y
    v_y_side_global = v_y + omega * d_x

    # Rotation matrix for the side frame
    R_theta = np.array([
        [np.cos(theta), np.sin(theta)],
        [-np.sin(theta), np.cos(theta)]
    ])

    # Transform velocity to side-frame coordinates
    v_side_local = R_theta @ np.array([v_x_side_global, v_y_side_global])

    # Angular velocity remains the same
    omega_side = omega

    return (v_side_local[0], v_side_local[1], omega_side)

# Simulation parameters
dt = 0.05  # Time step (50ms)
total_time = 2.0  # Run for 2 seconds
num_steps = int(total_time / dt)

# Initial positions
mid_x, mid_y, mid_theta = 0.0, 0.0, 0.0
side_x, side_y, side_theta = 0.5, 0.2, np.radians(30)  # Side frame initial position and yaw

# Velocity of mid-frame
v_mid = (1.0, 0.5, 0.2)  # (v_x, v_y, omega)

# Store positions for plotting
mid_positions = []
side_positions = []

for _ in range(num_steps):
    # Transform velocity for the side frame
    v_side = transform_velocity(v_mid, side_x - mid_x, side_y - mid_y, side_theta)

    # Update mid-frame position
    mid_x += v_mid[0] * dt
    mid_y += v_mid[1] * dt
    mid_theta += v_mid[2] * dt

    # Update side-frame position
    side_x += v_side[0] * dt
    side_y += v_side[1] * dt
    side_theta += v_side[2] * dt

    # Store for plotting
    mid_positions.append((mid_x, mid_y))
    side_positions.append((side_x, side_y))

# Convert to arrays for plotting
mid_positions = np.array(mid_positions)
side_positions = np.array(side_positions)

# Plot trajectories
plt.figure(figsize=(8, 6))
plt.plot(mid_positions[:, 0], mid_positions[:, 1], 'bo-', label="Mid-frame")
plt.plot(side_positions[:, 0], side_positions[:, 1], 'ro-', label="Side-frame")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Trajectory of Mid-frame and Side-frame")
plt.legend()
plt.grid()
plt.show()
