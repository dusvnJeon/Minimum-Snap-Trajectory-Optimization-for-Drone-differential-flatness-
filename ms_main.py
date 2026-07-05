import numpy as np
import cvxpy as cp
from ms_problem import MinimumSnapProblem
from ms_dataclass import Waypoints, DerivConstraint

waypoints = Waypoints(

    # 3 waypoint
    waypoints_pos = np.array([[0,0,1],[3,5,-2],[30,5,20]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0,3.14, 0]),
    waypoints_arrival_time = np.array([0,10,15])
)

if (np.shape(waypoints.waypoints_pos)[0] != np.shape(waypoints.waypoints_yaw)[0] 
    or np.shape(waypoints.waypoints_pos)[0] != np.shape(waypoints.waypoints_arrival_time)[0] 
    or np.shape(waypoints.waypoints_yaw)[0] != np.shape(waypoints.waypoints_arrival_time)[0]):
            raise ValueError(
                f"components of waypoints have to include same number of waypoints"
            )

constraints1 = DerivConstraint(
    waypoint_num = 1,
    which_variable = "x",
    which_deriv = 2,
    specific_value = 10
)

Pb = MinimumSnapProblem(waypoints)
Pb.add_deriv_equal_contraints(constraints1)

prob, result = Pb.solve_problem()

print("status:", prob.status)
print("optimal value:", result)
print("cx:", Pb.cx.value)
print("cy:", Pb.cy.value)
print("cz:", Pb.cz.value)
print("cyaw:", Pb.cyaw.value)