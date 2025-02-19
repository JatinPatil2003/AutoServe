#!/usr/bin/env python3
import numpy as np

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

# Example usage
v_mid = (1.0, 0.5, 0.2)  # Example velocity of mid-frame (v_x, v_y, omega)
d_x = 0.5  # Offset along x from mid
d_y = 0.2  # Offset along y from mid
theta = np.radians(0)  # Yaw angle of side-frame in radians

v_side = transform_velocity(v_mid, d_x, d_y, theta)
print("Velocity of Side Frame:", v_side)

# https://chatgpt.com/share/67b639a6-e0ec-800d-b1b3-b812b828df15