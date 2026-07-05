import numpy as np
from dataclasses import dataclass

@dataclass
class Waypoints():
    waypoints_pos: np.ndarray
    waypoints_yaw: np.ndarray
    waypoints_arrival_time: np.ndarray  # Accumulated time

    # all of data has to be equal with their num of rows.
        # num of row means num of waypoint.
        
@dataclass
class DerivConstraint():
    waypoint_num: np.ndarray
    which_variable: np.ndarray
    which_deriv: np.ndarray     # order. ex) 1,2,3 ...
    specific_value: np.ndarray

    # which_variable : x y z or yaw.
    # which_deriv : "vel", "acc"

