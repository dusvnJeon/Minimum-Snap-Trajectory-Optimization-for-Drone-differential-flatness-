import numpy as np

from ms_dataclass import DerivConstraint, Waypoints

senario = 1

traj_up_and_down = Waypoints(

    # 3 waypoint
    waypoints_pos = np.array([[0, 0, 0],[0, 0, 10]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0]),
    waypoints_arrival_time = np.array([0, 10])
)

traj_L = Waypoints(

    # 3 waypoint
    waypoints_pos = np.array([[0, 0, 5],[10, 0, 5], [10, 10, 5]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0]),
    waypoints_arrival_time = np.array([0, 5, 10])
)

traj_straight_round_trip = Waypoints(

    # 3 waypoint
    waypoints_pos = np.array([[0, 0, 5], [5, 5, 5], [0, 0, 5]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0]),
    waypoints_arrival_time = np.array([0, 5, 10])
)

traj_square = Waypoints(

    # 5 waypoint
    waypoints_pos = np.array([[0, 0, 5], [0, 5, 5], [5, 5, 5], [5, 0, 5], [0, 0, 5]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 5, 10, 15, 20])
)

traj_square_helix = Waypoints(

    # 5 waypoint
    waypoints_pos = np.array([[0, 0, 5], [0, 5, 10], [5, 5, 15], [5, 0, 20], [0, 0, 25]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 5, 10, 15, 20])
)

traj_takeoff_hover_square_landing = Waypoints(

    # 9 waypoint
    waypoints_pos = np.array([[0, 0, 0], [0, 0, 5], [0, 10, 5], [0, 15, 5], [5, 15, 5], [5, 10, 5], [0, 10, 5], [0, 0, 5], [0, 0, 0]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 5, 10, 15, 20, 25, 30, 35, 40])
)

traj_sharp_corner = Waypoints(

    # 4 waypoint
    waypoints_pos = np.array([[0, 0, 5], [5, 0, 5], [5, 0.5, 5], [10, 0.5, 5]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 5, 6, 11])
)

traj_altitude_mixed = Waypoints(

    # 4 waypoint
    waypoints_pos = np.array([[0, 0, 2], [5, 0, 8], [10, 5, 4], [15, 5, 10]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 5, 10, 15])
)

traj_circle_like = Waypoints(

    # 5 waypoint
    waypoints_pos = np.array([[5, 0, 5], [0, 5, 5], [-5, 0, 5], [0, -5, 5], [5, 0, 5]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 5, 10, 15, 20])
)

traj_figure_eight = Waypoints(

    # 9 waypoint
    waypoints_pos = np.array([[0, 0, 5], [5, 5, 5], [10, 0, 5], [5, -5, 5], [0, 0, 5], [-5, 5, 5], [-10, 0, 5], [-5, -5, 5], [0, 0, 5]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 4, 8, 12, 16, 20, 24, 28, 32])
)

traj_yaw_hover = Waypoints(

    # 3 waypoint
    waypoints_pos = np.array([[0, 0, 5], [0, 0, 5], [0, 0, 5]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, np.pi, 2 * np.pi]),
    waypoints_arrival_time = np.array([0, 5, 10])
)

traj_revisit_origin = Waypoints(

    # 5 waypoint
    waypoints_pos = np.array([[0, 0, 5], [5, 0, 5], [0, 0, 5], [-5, 0, 5], [0, 0, 5]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 5, 10, 15, 20])
)

traj_mixed_duration = Waypoints(

    # 4 waypoint
    waypoints_pos = np.array([[0, 0, 5], [10, 0, 5], [10, 1, 5], [20, 1, 5]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 10, 11, 25])
)

traj_aggressive_helix = Waypoints(

    # 5 waypoint
    waypoints_pos = np.array([[5, 0, 2], [0, 5, 6], [-5, 0, 10], [0, -5, 14], [5, 0, 18]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0, 0, 0, 0]),
    waypoints_arrival_time = np.array([0, 4, 8, 12, 16])
)

