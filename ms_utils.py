import numpy as np

def plot_mission_configure(input_waypoint_with_T: np.ndarray):
    
    # plot mission configure using input_waypoint_with_T
    
    # 3D plot. each trajectory sampled point has arrow that noticing yaw direction
    # some information of waypoints are written closing each waypoint
        # informations are (x,y,z,yaw, arrival time)
    
    return


def eval_waypoint_pos_intermsof_coefficient(t):
    # just for defining waypoint constraints trivial.

    return  np.array([1,t,t**2,t**3,t**4,t**5,t**6,t**7])

def eval_waypoint_vel_intermsof_coefficient(t):
    # just for defining waypoint constraints trivial.

    return  np.array([0,1,2*t,3*t**2,4*t**3,5*t**4,6*t**5,7*t**6])

def eval_waypoint_acc_intermsof_coefficient(t):
    # just for defining waypoint constraints trivial.

    return  np.array([0,0,2,6*t,12*t**2,20*t**3,30*t**4,42*t**5])

def eval_waypoint_jerk_intermsof_coefficient(t):
    # just for defining waypoint constraints trivial.

    return  np.array([0,0,0,6,24*t,60*t**2,120*t**3,210*t**4])

def eval_waypoint_yaw_intermsof_coefficient(t):
    # just for defining waypoint constraints trivial.

    return  np.array([1,t,t**2,t**3])

def eval_waypoint_yaw_rate_intermsof_coefficient(t):
    # just for defining waypoint constraints trivial.

    return  np.array([0,1,2*t,3*t**2])

def get_cost_weight(delta_t):   # segment duration time
    n_pos_coeff = 8
    n_yaw_coeff = 4

    # For position:
    # p(t) = x0 + x1*t + ... + x7*t^7
    # p''''(t) = 24*x4 + 120*x5*t + 360*x6*t^2 + 840*x7*t^3
    # integral((p''''(t))^2, t=0..delta_t) = x.T @ Q_snap @ x
    Q_snap = np.zeros((n_pos_coeff, n_pos_coeff))

    # For yaw:
    # yaw(t) = y0 + y1*t + y2*t^2 + y3*t^3
    # yaw''(t) = 2*y2 + 6*y3*t
    # integral((yaw''(t))^2, t=0..delta_t) = y.T @ Q_yaw_acc @ y
    Q_yaw_acc = np.zeros((n_yaw_coeff, n_yaw_coeff))

    for i in range(4, n_pos_coeff):
        for j in range(4, n_pos_coeff):
            ci = i * (i - 1) * (i - 2) * (i - 3)
            cj = j * (j - 1) * (j - 2) * (j - 3)
            power = i + j - 7
            Q_snap[i, j] = ci * cj * delta_t**power / power

    for i in range(2, n_yaw_coeff):
        for j in range(2, n_yaw_coeff):
            ci = i * (i - 1)
            cj = j * (j - 1)
            power = i + j - 3
            Q_yaw_acc[i, j] = ci * cj * delta_t**power / power

    return Q_snap, Q_yaw_acc        # way to cal has to be understands
