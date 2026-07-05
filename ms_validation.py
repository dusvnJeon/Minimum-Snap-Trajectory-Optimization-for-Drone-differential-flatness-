import numpy as np

from ms_dataclass import DerivConstraint, Waypoints
from ms_problem import MinimumSnapProblem
from ms_utils import eval_waypoint_pos_intermsof_coefficient, eval_waypoint_yaw_intermsof_coefficient

traj_up_and_down = Waypoints(

    # 3 waypoint
    waypoints_pos = np.array([[0, 0, 0],[0, 0, 10]]),  # [x,y,z]_1, [x,y,z]_2 ...
    waypoints_yaw = np.array([0, 0]),
    waypoints_arrival_time = np.array([0, 10])
)

constraints1 = DerivConstraint(
    waypoint_num = 1,
    which_variable = "x",
    which_deriv = 2,
    specific_value = 10
)

Pb = MinimumSnapProblem(traj_up_and_down)
Pb.add_deriv_equal_contraints(constraints1)

prob, result = Pb.solve_problem()

print("status:", prob.status)
print("optimal value:", result)
print("cx:", Pb.cx.value)
print("cy:", Pb.cy.value)
print("cz:", Pb.cz.value)
print("cyaw:", Pb.cyaw.value)