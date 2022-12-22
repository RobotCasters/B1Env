import numpy as np

# for readthedocs
try:
    import proxsuite
except:
    print("ProxSuite is not installed")


from B1Env.mpc.utils import get_A, get_b, get_C, get_g, get_H, get_UB


class MPC:
    r"""
    This class solves a MPC problem in the form of

    .. math::
        \begin{align}
            \min_{x}\ &\ \frac{1}{2}x^T\mathbf{H}x + \mathbf{g}^Tx\\
            \mathrm{subject\ to}\ &\ \mathbf{A}x = \mathbf{b}\\
                                  &\ \mathbf{C}x \leq \mathrm{UB}.
        \end{align}

    """

    def __init__(self, horizon_length=16):
        r"""
        The ``__init__`` function sets the problem variables and pre-computes
        the non-changing parts of the MPC problem.

        :param horizon_length: the horizon length of the MPC problem
        :type horizon_length: int
        """
        # set problem variables
        self.horizon_length = horizon_length
        self.n_sw = 2  # number of swing legs
        self.n_st = 2  # number of stance leg
        self.f_min = 1e-3  # minimum support force
        self.f_max = 100.0  # maximum support force
        self.mu = 0.6  # ground friction coefficient

        # define QP dimension
        self.n = 27 * self.horizon_length
        self.n_eq = (15 + 3 * self.n_sw) * self.horizon_length
        self.n_ieq = 6 * self.n_st * self.horizon_length

        # define QP
        self.qp = proxsuite.proxqp.dense.QP(self.n, self.n_eq, self.n_ieq)
        self.qp_initialized = False

        # compute non-changing part of the MPC problem
        self.pre_construct_objective_function()
        self.pre_construct_inequality_constraint()

    def pre_construct_objective_function(self):
        r"""
        Computes the non-changing part in the MPC objective function.
        In this case the quadratic cost matrix ``H`` is not changing.
        """
        self.H = get_H(timesteps=self.horizon_length)

    def construct_objective_function(self, x_ref_mat):
        r"""
        Computes the changing part in the MPC objective function. In this
        case the linear cost matrix ``g`` changes each timestep.

        :param x_ref_mat: the reference motion of the centroidal model
        :type x_ref_mat: ndarray
        """
        self.g = get_g(x_ref_mat, timesteps=self.horizon_length)

    def pre_construct_inequality_constraint(self):
        r"""
        Computes the non-changing part in the MPC inequality constraint.
        In this case the inequality constraint upper bound ``UB`` is
        not changing.
        """
        self.UB = get_UB(
            self.n_st, f_min=self.f_min, f_max=self.f_max, timesteps=self.horizon_length
        )

    def construct_inequality_constraint(self, M_st):
        r"""
        Computes the changing part in the MPC inequality constraint. In this
        case the inequality constraint matrix ``C`` changes each timestep.

        :param M_st: the stance foot selection matrix
        :type M_st: ndarray
        """
        self.C = get_C(self.n_st, M_st, timesteps=self.horizon_length, mu=self.mu)

    def construct_equality_constraint(self, bA, bA_0, bB, x_0, M_sw):
        r"""
        Computes the changing part in the MPC equality constraint. In this
        case both the ``A`` and ``b`` matrix changes each timestep.

        :param bA: the aggregated drift matrix
        :type bA: ndarray
        :param bA_0: the drift matrix at the current state
        :type bA_0: ndarray
        :param bB: the aggregated control matrix
        :type bB: ndarray
        :param x_0: the current centroidal dynamics state
        :type x_0: ndarray
        :param M_sw: the swing foot selection matrix
        :type M_sw: ndarray
        """
        self.A = get_A(bA, bB, M_sw, self.n_sw, timesteps=self.horizon_length)
        self.b = get_b(bA_0, x_0, self.n_sw, timesteps=self.horizon_length)

    def solve(self):
        r"""
        Solves the MPC problem using the computed problem configurations.
        Need to run ``construct_objective_function``, ``construct_inequality_constraint``,
        and ``construct_equality_constraint`` before running this function.
        """
        if not self.qp_initialized:
            self.qp.init(H=self.H, g=self.g, A=self.A, b=self.b, C=self.C, u=self.UB)
            self.qp.settings.eps_abs = 1.0e-6
            self.qp_initialized = True
        else:
            self.qp.update(H=self.H, g=self.g, A=self.A, b=self.b, C=self.C, u=self.UB)

        self.qp.solve()
        self.qp_sol = self.qp.results.x
