import numpy as np

def cubic_planning(pos_start, pos_end, T_swing=10, T_stance=300, lift_height=40):
    """
    Cubic polynomial trajectory planning for swing and ground phases.
    This is for comparison with quintic planning to demonstrate why quintic is better.
    
    Parameters:
        pos_start : array-like [x, y, z] — start position (pos_A)
        pos_end   : array-like [x, y, z] — end position   (pos_D)
        T_swing   : int — number of waypoints in swing phase
        T_stance  : int — number of waypoints in ground phase
        lift_height : float — how high to lift foot (mm, positive = up)
    
    Returns:
        waypoint : np.ndarray (T_swing + T_stance, 3) — [x, y, z] trajectory
    """
    
    T_total = T_swing + T_stance
    waypoint = np.zeros((T_total, 3))

    x_start = pos_start[0]
    x_end   = pos_end[0]
    y_start = pos_start[1]
    y_end   = pos_end[1]
    z_start = pos_start[2]
    z_end   = pos_end[2]
    z_ground = min(z_start, z_end)

    # Parabola path planning (swing phase)
    for i in range(T_swing):
        tau = i / (T_swing - 1)                     # tau belongs to [0, 1]
        s   = 3*tau**2 - 2*tau**3                   #   s belongs to [0, 1] (Cubic interpolation)

        x = x_start + s * (x_end - x_start)
        y = y_start + s * (y_end - y_start)
        z = z_ground + lift_height * 4 * s * (1 - s)
        waypoint[i, :] = [x, y, z]

    # Linear path planning (ground phase)
    for i in range(T_stance):
        tau = i / (T_stance - 1)                    # tau belongs to [0, 1]
        s   = 3*tau**2 - 2*tau**3                   #   s belongs to [0, 1] (Cubic interpolation)

        x = x_end + s * (x_start - x_end)
        y = y_end + s * (y_start - y_end)
        z = z_ground
        waypoint[T_swing + i, :] = [x, y, z]

    return waypoint
