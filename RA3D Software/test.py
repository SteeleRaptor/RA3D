import numpy as np
import math

def shift_zero_orientation(yaw, pitch, roll):

    R = euler_zyx_to_matrix(yaw, pitch, roll)

    R_offset = rot_y(np.radians(-90))

    R_new = R_offset.T @ R   # inverse = transpose

    return matrix_to_euler_zyx(R_new)

def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([
        [1,0,0],
        [0,c,-s],
        [0,s,c]
    ])

def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([
        [c,0,s],
        [0,1,0],
        [-s,0,c]
    ])

def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([
        [c,-s,0],
        [s,c,0],
        [0,0,1]
    ])
def euler_zyx_to_matrix(yaw, pitch, roll):
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
def matrix_to_euler_zyx(R):
    if abs(R[2,0]) < 1:
        pitch = -np.arcsin(R[2,0])
        yaw = np.arctan2(R[1,0], R[0,0])
        roll = np.arctan2(R[2,1], R[2,2])
    else:
        pitch = np.pi/2 if R[2,0] <= -1 else -np.pi/2
        yaw = np.arctan2(-R[0,1], R[1,1])
        roll = 0.0

    return yaw, pitch, roll


def aer_to_euler_zyx2(azimuth, elevation, roll):
    
    
    azimuth = math.radians(azimuth)
    elevation = math.radians(elevation)
    roll = math.radians(roll)
    # ---- Step 1: Forward vector from spherical coordinates ----
    fx = np.cos(elevation) * np.cos(azimuth)
    fy = np.cos(elevation) * np.sin(azimuth)
    fz = np.sin(elevation)

    forward = np.array([fx, fy, fz])
    forward = forward / np.linalg.norm(forward)

    # ---- Step 2: Build orthonormal frame ----
    world_up = np.array([0.0, 0.0, 1.0])

    right = np.cross(world_up, forward)
    right = right / np.linalg.norm(right)

    up = np.cross(forward, right)

    # ---- Step 3: Apply roll about forward axis ----
    # Rodrigues rotation formula
    K = np.array([
        [0, -forward[2], forward[1]],
        [forward[2], 0, -forward[0]],
        [-forward[1], forward[0], 0]
    ])

    R_roll = np.eye(3) + np.sin(roll) * K + (1 - np.cos(roll)) * (K @ K)

    R_no_roll = np.column_stack((right, up, forward))
    R = R_no_roll @ R_roll

    # ---- Step 4: Extract ZYX Euler angles ----
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)

    if abs(R[2,0]) < 1.0:
        pitch = -np.arcsin(R[2,0])
        yaw = np.arctan2(R[1,0], R[0,0])
        roll_zyx = np.arctan2(R[2,1], R[2,2])
    else:
        # Gimbal lock case
        pitch = np.pi/2 if R[2,0] <= -1 else -np.pi/2
        yaw = np.arctan2(-R[0,1], R[1,1])
        roll_zyx = 0.0
    yaw,pitch,roll_zyx = shift_zero_orientation(yaw, pitch, roll_zyx)
    yaw = math.degrees(yaw)
    pitch = math.degrees(pitch)
    roll_zyx = math.degrees(roll_zyx)
    
    return yaw,pitch,roll_zyx

def aer_to_euler_zyx_with_comfort_roll(
    azimuth, elevation,
    tool_position,
    comfort_point):
    """
    azimuth, elevation in radians
    tool_position: np.array([x,y,z])
    comfort_point: np.array([x,y,z])
    """
    azimuth = math.radians(azimuth)
    elevation = math.radians(elevation)
    # ---- Forward from AER ----
    fx = np.cos(elevation) * np.cos(azimuth)
    fy = np.cos(elevation) * np.sin(azimuth)
    fz = np.sin(elevation)

    forward = np.array([fx, fy, fz])
    forward /= np.linalg.norm(forward)

    # ---- Vector toward comfort point ----
    v = comfort_point - tool_position

    # ---- Remove component along forward (projection onto roll plane) ----
    v_perp = v - np.dot(v, forward) * forward

    if np.linalg.norm(v_perp) < 1e-8:
        # Degenerate case: choose world up fallback
        v_perp = np.array([0, 0, 1]) - np.dot([0,0,1], forward)*forward

    up = v_perp / np.linalg.norm(v_perp)

    # ---- Compute right vector ----
    right = np.cross(up, forward)
    right /= np.linalg.norm(right)

    # Re-orthogonalize up
    up = np.cross(forward, right)

    # ---- Build rotation matrix ----
    # Tool X = forward
    # Tool Y = right
    # Tool Z = up
    R = np.column_stack((forward, right, up))

    # ---- Extract ZYX Euler ----
    if abs(R[2,0]) < 1:
        pitch = -np.arcsin(R[2,0])
        yaw = np.arctan2(R[1,0], R[0,0])
        roll = np.arctan2(R[2,1], R[2,2])
    else:
        pitch = np.pi/2 if R[2,0] <= -1 else -np.pi/2
        yaw = np.arctan2(-R[0,1], R[1,1])
        roll = 0.0

    yaw = math.degrees(yaw)
    pitch = math.degrees(pitch)
    roll_zyx = math.degrees(roll)
    
    return yaw, pitch, roll_zyx


'''
class AEROrientationSolver:
    def __init__(
                    stiffness_weight=0.5,   # 0 → world up, 1 → comfort bias
                    tau=0.1):               # smoothing time constant (seconds)

        stiffness_weight = stiffness_weight
        tau = tau
        prev_roll = 0.0

    def compute(
                azimuth,
                elevation,
                tool_position,
                comfort_point,
                dt):
        """
        azimuth, elevation in radians
        tool_position, comfort_point → np.array([x,y,z])
        dt → timestep (seconds)
        """
        azimuth = math.radians(azimuth)
        elevation = math.radians(elevation)
        # ---- Forward vector from AER ----
        forward = np.array([
            np.cos(elevation) * np.cos(azimuth),
            np.cos(elevation) * np.sin(azimuth),
            np.sin(elevation)
        ])
        forward /= np.linalg.norm(forward)

        # ---- World-up reference ----
        world_up = np.array([0.0, 0.0, 1.0])
        v_world = world_up - np.dot(world_up, forward) * forward

        # ---- Comfort vector ----
        v_comfort = comfort_point - tool_position
        v_comfort = v_comfort - np.dot(v_comfort, forward) * forward

        # ---- Normalize both ----
        if np.linalg.norm(v_world) < 1e-8:
            v_world = np.array([1.0, 0.0, 0.0])
        else:
            v_world /= np.linalg.norm(v_world)

        if np.linalg.norm(v_comfort) < 1e-8:
            v_comfort = v_world
        else:
            v_comfort /= np.linalg.norm(v_comfort)

        # ---- Blend ----
        alpha = stiffness_weight
        v_blend = (1 - alpha) * v_world + alpha * v_comfort
        v_blend /= np.linalg.norm(v_blend)

        up = v_blend

        # ---- Right vector ----
        right = np.cross(up, forward)
        right /= np.linalg.norm(right)

        # Re-orthogonalize up
        up = np.cross(forward, right)

        # ---- Build rotation matrix ----
        R = np.column_stack((forward, right, up))

        # ---- Extract ZYX Euler ----
        if abs(R[2,0]) < 1:
            pitch = -np.arcsin(R[2,0])
            yaw = np.arctan2(R[1,0], R[0,0])
            roll_raw = np.arctan2(R[2,1], R[2,2])
        else:
            pitch = np.pi/2 if R[2,0] <= -1 else -np.pi/2
            yaw = np.arctan2(-R[0,1], R[1,1])
            roll_raw = 0.0

        # ---- Low-pass filter roll ----
        beta = dt / (tau + dt)
        roll_filtered = prev_roll + beta * (roll_raw - prev_roll)

        prev_roll = roll_filtered
        yaw = math.degrees(yaw)
        pitch = math.degrees(pitch)
        roll_filtered= math.degrees(roll_filtered)
        return yaw, pitch, roll_filtered'''
def aer_to_euler_zyx3(azimuth, elevation, roll):
    """
    Convert AER (Azimuth, Elevation, Roll)
    to Euler ZYX (yaw, pitch, roll)
    Input and output in degrees
    All other angles in radians.
    """
    azimuth = math.radians(azimuth)
    elevation = math.radians(elevation)
    roll = math.radians(roll)

    # ---- Forward vector from spherical coordinates ----
    fx = np.cos(elevation) * np.cos(azimuth)
    fy = np.cos(elevation) * np.sin(azimuth)
    fz = np.sin(elevation)

    forward = np.array([fx, fy, fz])
    forward /= np.linalg.norm(forward)

    world_up = np.array([0.0, 0.0, 1.0])

    # ---- Build orthonormal frame ----
    right = np.cross(world_up, forward)
    right /= np.linalg.norm(right)

    up = np.cross(forward, right)

    # ---- Apply roll about forward axis ----
    K = np.array([
        [0, -forward[2], forward[1]],
        [forward[2], 0, -forward[0]],
        [-forward[1], forward[0], 0]
    ])

    R_roll = np.eye(3) + np.sin(roll) * K + (1 - np.cos(roll)) * (K @ K)

    # Z = forward (tool axis)
    R_no_roll = np.column_stack((right, up, forward))

    R = R_no_roll @ R_roll

    # ---- TOOL OFFSET HERE ----
    R_offset = rot_z(np.radians(-90))
    R = R_offset @ R

    # ---- Extract Euler ----
    yaw, pitch, roll_zyx = matrix_to_euler_zyx(R)
   
    if abs(roll_zyx) > np.pi/2:
        roll_zyx -= np.sign(roll_zyx)*np.pi
        yaw += np.pi
    return (
        math.degrees(yaw),
        math.degrees(pitch),
        math.degrees(roll_zyx)
    )
def aer_to_euler_zyx(azimuth, elevation, roll):
    # Convert inputs to radians
    az = np.radians(azimuth)
    el = np.radians(elevation)
    rl = np.radians(roll)

    # 1. Rotation for Azimuth (Yaw) around Z-axis
    R_z = np.array([
        [np.cos(az), -np.sin(az), 0],
        [np.sin(az),  np.cos(az), 0],
        [0,           0,          1]
    ])

    # 2. Rotation for Elevation (Pitch) around Y-axis
    # Note: Use -el if 'up' is positive elevation in your coordinate system
    R_y = np.array([
        [np.cos(-el),  0, np.sin(-el)],
        [0,            1, 0],
        [-np.sin(-el), 0, np.cos(-el)]
    ])

    # 3. Rotation for Roll around the tool axis (X-axis in ZYX convention)
    R_x = np.array([
        [1, 0,           0],
        [0, np.cos(rl), -np.sin(rl)],
        [0, np.sin(rl),  np.cos(rl)]
    ])

    # Combined Rotation Matrix: R = Rz * Ry * Rx
    R = R_z @ R_y @ R_x
     # ---- TOOL OFFSET HERE ----
    R_offset = rot_y(np.radians(90))
    R = R_offset @ R
    # Extract Euler angles from the matrix
    # Standard ZYX extraction (Yaw, Pitch, Roll)
    pitch = np.arcsin(-R[2, 0])
    
    # Handle gimbal lock (pitch = +/- 90 degrees)
    if np.abs(np.cos(pitch)) > 1e-6:
        yaw = np.arctan2(R[1, 0], R[0, 0])
        roll_out = np.arctan2(R[2, 1], R[2, 2])
    else:
        # Singularity case
        yaw = 0
        roll_out = np.arctan2(-R[0, 1], R[1, 1])

    return np.degrees(yaw), np.degrees(pitch), np.degrees(roll_out)

def aer_to_euler_zyx_final(azimuth, elevation, roll):
    # 1. Setup angles (Elevation is typically -Pitch in this convention)
    theta_in = np.radians(azimuth)
    phi_in   = np.radians(-elevation) # Negative to match 'up' as positive
    psi_in   = np.radians(roll)

    # 2. Build the Matrix R = Rz * Ry * Rx
    # (Same as previous step, combined into the rotation matrix R)
    c_th, s_th = np.cos(theta_in), np.sin(theta_in)
    c_ph, s_ph = np.cos(phi_in),   np.sin(phi_in)
    c_ps, s_ps = np.cos(psi_in),   np.sin(psi_in)

    R = np.array([
        [c_th*c_ph, c_th*s_ph*s_ps - s_th*c_ps, c_th*s_ph*c_ps + s_th*s_ps],
        [s_th*c_ph, s_th*s_ph*s_ps + c_th*c_ps, s_th*s_ph*c_ps - c_th*s_ps],
        [-s_ph,     c_ph*s_ps,                 c_ph*c_ps]
    ])

    # 3. Extraction using the image formulas
    # phi = arctan2(-T(3,1), sqrt(T(1,1)^2 + T(2,1)^2))
    phi = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    
    # Pre-calculate cos(phi) for the denominators
    cos_phi = np.cos(phi)

    # Note: np.arctan2(y/cos_phi, x/cos_phi) is mathematically 
    # the same as np.arctan2(y, x) if cos_phi is positive.
    if np.abs(cos_phi) > 1e-6:
        psi = np.arctan2(R[2, 1] / cos_phi, R[2, 2] / cos_phi)
        theta = np.arctan2(R[1, 0] / cos_phi, R[0, 0] / cos_phi)
    else:
        # Gimbal lock handling (when pointing straight up/down)
        psi = 0
        theta = np.arctan2(R[0, 1], R[1, 1])

    return np.degrees(theta), np.degrees(phi), np.degrees(psi)
import numpy as np
import math

def rot_x(r):
    return np.array([
        [1, 0, 0],
        [0, math.cos(r), -math.sin(r)],
        [0, math.sin(r),  math.cos(r)]
    ])

def rot_y(r):
    return np.array([
        [ math.cos(r), 0, math.sin(r)],
        [0, 1, 0],
        [-math.sin(r), 0, math.cos(r)]
    ])

def rot_z(r):
    return np.array([
        [math.cos(r), -math.sin(r), 0],
        [math.sin(r),  math.cos(r), 0],
        [0, 0, 1]
    ])

def convertZYX_Zup_to_Xup(yaw, pitch, roll):

    yaw = math.radians(yaw)
    pitch = math.radians(pitch)
    roll = math.radians(roll)

    # ZYX rotation
    R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

    T = np.array([
        [0,0,1],
        [0,1,0],
        [-1,0,0]
    ])

    R_new = T @ R @ T.T

    pitch2 = -math.asin(R_new[2,0])
    roll2  = math.atan2(R_new[2,1], R_new[2,2])
    yaw2   = math.atan2(R_new[1,0], R_new[0,0])

    return (
        math.degrees(yaw2),
        math.degrees(pitch2),
        math.degrees(roll2)
    )
#red vector is the one
yaw,pitch,roll=convertZYX_Zup_to_Xup(90,45,0)
print(yaw,pitch,roll)
#yaw,pitch,roll=aer_to_euler_zyx_final(90,22,15)
#print(roll,pitch,yaw)