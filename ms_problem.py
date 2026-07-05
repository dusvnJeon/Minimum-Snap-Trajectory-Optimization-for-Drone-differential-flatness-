import numpy as np
import cvxpy as cp
from ms_dataclass import (
    DerivConstraint,
    Waypoints,
)
from ms_utils import (
    eval_waypoint_pos_intermsof_coefficient,
    eval_waypoint_vel_intermsof_coefficient,
    eval_waypoint_acc_intermsof_coefficient,
    eval_waypoint_jerk_intermsof_coefficient,

    eval_waypoint_yaw_intermsof_coefficient,
    eval_waypoint_yaw_rate_intermsof_coefficient,
    get_cost_weight,

)


class MinimumSnapProblem:
    def __init__(self, waypoints: Waypoints):

        self.waypoints = waypoints

        self.n_pos_coeff = 8 # 7th order polynomial
        self.n_yaw_coeff = 4 # 4th order polynomial
        self.n_waypoint = np.shape(self.waypoints.waypoints_arrival_time)[0]    # m+1: 0~m
        self.n_seg = self.n_waypoint -1     # m : 0~m-1

        self.cx = cp.Variable((self.n_seg, self.n_pos_coeff))
        self.cy = cp.Variable((self.n_seg, self.n_pos_coeff))
        self.cz = cp.Variable((self.n_seg, self.n_pos_coeff))
        self.cyaw = cp.Variable((self.n_seg, self.n_yaw_coeff))

        self.duration_time = self.get_duration_time()

        self.constraints = []
        self.cost = 0
    
    def get_duration_time(self):
        
        duration_time = np.zeros(self.n_seg)

        for i in range(self.n_seg): # 0 ~ m-1
            T_i = self.waypoints.waypoints_arrival_time[i]
            T_i_plus_1 = self.waypoints.waypoints_arrival_time[i+1]
            duration_time[i] = T_i_plus_1 - T_i

        return duration_time
    
    def continuity_contraints_add(self):
        # we minimize norm square of (snap and psi'') where psi is yaw.
        # So, continuity at each waypoint has to be on for (p,v,a,j) & (yaw, omega)

        for i in range(1,self.n_waypoint-1):
          
            t_L = self.duration_time[i-1]
            t_R = 0

            basis_p_L = eval_waypoint_pos_intermsof_coefficient(t_L)
            basis_v_L= eval_waypoint_vel_intermsof_coefficient(t_L)
            basis_a_L= eval_waypoint_acc_intermsof_coefficient(t_L)
            basis_j_L= eval_waypoint_jerk_intermsof_coefficient(t_L)
            basis_yaw_L= eval_waypoint_yaw_intermsof_coefficient(t_L)
            basis_yaw_rate_L= eval_waypoint_yaw_rate_intermsof_coefficient(t_L)

            basis_p_R = eval_waypoint_pos_intermsof_coefficient(t_R)
            basis_v_R= eval_waypoint_vel_intermsof_coefficient(t_R)
            basis_a_R= eval_waypoint_acc_intermsof_coefficient(t_R)
            basis_j_R= eval_waypoint_jerk_intermsof_coefficient(t_R)
            basis_yaw_R= eval_waypoint_yaw_intermsof_coefficient(t_R)
            basis_yaw_rate_R= eval_waypoint_yaw_rate_intermsof_coefficient(t_R)

            # apply each translational constraints for each axis decision variable.
            translation_basis_pairs = [
                (basis_p_L, basis_p_R),
                (basis_v_L, basis_v_R),
                (basis_a_L, basis_a_R),
                (basis_j_L, basis_j_R),
            ]
            for basis_L, basis_R in translation_basis_pairs:
                for axis in [self.cx, self.cy, self.cz]:
                    self.constraints.append(basis_L@axis[i-1] == basis_R@axis[i])

            yaw_basis_pairs = [
                (basis_yaw_L, basis_yaw_R),
                (basis_yaw_rate_L, basis_yaw_rate_R),
            ]

            for basis_L, basis_R in yaw_basis_pairs:
                self.constraints.append(basis_L @ self.cyaw[i - 1] == basis_R @ self.cyaw[i])

        return

    def waypoint_equality_constraints_add(self):

        for i in range(self.n_seg):

            waypoint_i_pos_0 = self.waypoints.waypoints_pos[i]
            waypoint_i_pos_T = self.waypoints.waypoints_pos[i+1]

            waypoint_i_att_0 = self.waypoints.waypoints_yaw[i]
            waypoint_i_att_T = self.waypoints.waypoints_yaw[i+1]

            # waypoint_i_t_0 = self.waypoints.waypoints_arrival_time[i]
            # waypoint_i_t_T = self.waypoints.waypoints_arrival_time[i+1]

            waypoint_i_t_0 = 0
            waypoint_i_t_T = self.duration_time[i]

            basis_pos_0 = eval_waypoint_pos_intermsof_coefficient(waypoint_i_t_0)
            basis_pos_T = eval_waypoint_pos_intermsof_coefficient(waypoint_i_t_T)

            basis_att_0 = eval_waypoint_yaw_intermsof_coefficient(waypoint_i_t_0)
            basis_att_T = eval_waypoint_yaw_intermsof_coefficient(waypoint_i_t_T)

            # initial point of segment
            expected_x_0 = basis_pos_0@self.cx[i]
            expected_y_0 = basis_pos_0@self.cy[i]
            expected_z_0 = basis_pos_0@self.cz[i]
            expected_yaw_0 = basis_att_0@self.cyaw[i]
            expected_i_pos_0 = cp.hstack([expected_x_0, expected_y_0, expected_z_0])
            self.constraints.append(expected_i_pos_0 == waypoint_i_pos_0)
            self.constraints.append(expected_yaw_0 == waypoint_i_att_0)

            # final point of segment
            expected_x_T = basis_pos_T@self.cx[i]
            expected_y_T = basis_pos_T@self.cy[i]
            expected_z_T = basis_pos_T@self.cz[i]
            expected_yaw_T = basis_att_T@self.cyaw[i]
            expected_i_pos_T = cp.hstack([expected_x_T, expected_y_T, expected_z_T])
            self.constraints.append(expected_i_pos_T == waypoint_i_pos_T)
            self.constraints.append(expected_yaw_T == waypoint_i_att_T)
        
        return

    def add_deriv_equal_contraints(self, deriv_constraint: DerivConstraint):
        
        # deriv constraints has to implemment both of inital point and final point of segment.
            # but if given waypoint is initial point or finil point of whole trajectory, only 1 constraints has to be applied.
                # initial point of whole trajectory -> only segment coefficient on left side of point
                # final point of whole trajectory -> only segment coefficient on right side of point

        const_waypoint =  deriv_constraint.waypoint_num

        if  const_waypoint == 0:
            expect_R = self.get_deriv_expected_val_one_side(deriv_constraint, "right")
            self.constraints.append(expect_R == deriv_constraint.specific_value)
        elif const_waypoint == self.n_waypoint -1:
            expect_L = self.get_deriv_expected_val_one_side(deriv_constraint, "left")
            self.constraints.append(expect_L == deriv_constraint.specific_value)
        else:
            expect_L = self.get_deriv_expected_val_one_side(deriv_constraint, "left")
            expect_R = self.get_deriv_expected_val_one_side(deriv_constraint, "right")

            self.constraints.append(expect_L == deriv_constraint.specific_value)
            self.constraints.append(expect_R == deriv_constraint.specific_value)

        return
    
    def add_deriv_inequal_contraints(self, deriv_constraint: DerivConstraint):
        
        # deriv constraints has to implemment both of inital point and final point of segment.
            # but if given waypoint is initial point or finil point of whole trajectory, only 1 constraints has to be applied.
                # initial point of whole trajectory -> only segment coefficient on left side of point
                # final point of whole trajectory -> only segment coefficient on right side of point

        const_waypoint =  deriv_constraint.waypoint_num

        if  const_waypoint == 0:
            expect_R = self.get_deriv_expected_val_one_side(deriv_constraint, "right")
            self.constraints.append(expect_R <= deriv_constraint.specific_value)
        elif const_waypoint == self.n_waypoint -1:
            expect_L = self.get_deriv_expected_val_one_side(deriv_constraint, "left")
            self.constraints.append(expect_L <= deriv_constraint.specific_value)
        else:
            expect_L = self.get_deriv_expected_val_one_side(deriv_constraint, "left")
            expect_R = self.get_deriv_expected_val_one_side(deriv_constraint, "right")

            self.constraints.append(expect_L <= deriv_constraint.specific_value)
            self.constraints.append(expect_R <= deriv_constraint.specific_value)

        return

    def get_deriv_expected_val_one_side(self, deriv_constraint: DerivConstraint, side=None):

        # side means "left" or "right"
        # if constraints are on waypoint except 0th and mth, both side of segments coefficients have to be optimized.
            # But, if for 0th or mth, their constraints has to be applied on right and left side of segment coefficients for each.
        # CRETERIA OF SIDE IS WAYPOINT! Inspect of the waypoint, which coefficient of segments has to be optimizaed?

        const_var = deriv_constraint.which_variable
        # const_t = self.waypoints.waypoints_arrival_time[deriv_contraint.waypoint_num]
        const_order = deriv_constraint.which_deriv
        const_waypoint =  deriv_constraint.waypoint_num

        if  side == "left": # as n_waypoint is m+1
            const_t = self.duration_time[const_waypoint -1] # left segment's time would be duration time of past segment
        elif side == "right":
            const_t = 0 # left segment's time would be 0.
        else:
            raise ValueError("left and right only!")
            
        if const_var == "yaw" and const_order == 1:
            basis = eval_waypoint_yaw_rate_intermsof_coefficient(const_t)
            coeff = self.cyaw

        elif const_var == "x" and const_order == 1:
            basis = eval_waypoint_vel_intermsof_coefficient(const_t)
            coeff = self.cx

        elif const_var == "y" and const_order == 1:
            basis = eval_waypoint_vel_intermsof_coefficient(const_t)
            coeff = self.cy

        elif const_var == "z" and const_order == 1:
            basis = eval_waypoint_vel_intermsof_coefficient(const_t)
            coeff = self.cz

        elif const_var == "x" and const_order == 2:
            basis = eval_waypoint_acc_intermsof_coefficient(const_t)
            coeff = self.cx

        elif const_var == "y" and const_order == 2:
            basis = eval_waypoint_acc_intermsof_coefficient(const_t)
            coeff = self.cy

        elif const_var == "z" and const_order == 2:
            basis = eval_waypoint_acc_intermsof_coefficient(const_t)
            coeff = self.cz

        else:
            raise ValueError(
                f"Unsupported derivative constraint: "
                f"which_variable={const_var}, which_deriv={const_order}"
            )
        
        if side == "left":
            return basis @ coeff[deriv_constraint.waypoint_num -1]
        elif side == "right":
            return basis @ coeff[deriv_constraint.waypoint_num]


    def solve_problem(self):

        for i in range(np.shape(self.cx)[0]):

            delta_t = self.waypoints.waypoints_arrival_time[i+1] - self.waypoints.waypoints_arrival_time[i]
            Q_snap, Q_yaw_acc = get_cost_weight(delta_t)

            Q_snap = 0.5*(Q_snap + Q_snap.T)
            Q_yaw_acc = 0.5*(Q_yaw_acc + Q_yaw_acc.T)
            
            # self.cost += cp.quad_form(self.cx[i, :], Q_snap)
            # self.cost += cp.quad_form(self.cy[i, :], Q_snap)
            # self.cost += cp.quad_form(self.cz[i, :], Q_snap)

            self.cost += cp.quad_form(self.cx[i, :], cp.psd_wrap(Q_snap))
            self.cost += cp.quad_form(self.cy[i, :], cp.psd_wrap(Q_snap))
            self.cost += cp.quad_form(self.cz[i, :], cp.psd_wrap(Q_snap))

            # self.cost += cp.quad_form(self.cyaw[i, :], Q_yaw_acc)

            self.cost += cp.quad_form(self.cyaw[i, :], cp.psd_wrap(Q_yaw_acc))

        self.waypoint_equality_constraints_add()
        self.continuity_contraints_add()

        objective = cp.Minimize(self.cost)
        prob = cp.Problem(objective, self.constraints)
        result = prob.solve()

        return prob, result