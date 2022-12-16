from pdb import set_trace

import numpy as np
from scipy.linalg import block_diag


def get_Q_step():
    r"""
    Computes the step-wise state cost matrix ``Q_step``:

    .. math::
        \mathbf{Q}_{i+1} = \mathrm{blockdiag}\Big(\mathbf{I}_3, \mathbf{0}_{3\times3}, \mathbf{I}_3, \mathbf{0}_{6\times6}\Big)\in\mathbb{R}^{15\times15}

    :return: step-wise state cost matrix ``Q_step``
    :rtype: ndarray
    """

    identity_mat_3 = np.eye(3)
    zeros_mat_3 = np.zeros((3, 3))
    zeros_mat_6 = np.zeros((6, 6))

    Q_step = block_diag(identity_mat_3, zeros_mat_3, identity_mat_3, zeros_mat_6)

    return Q_step


def get_HQ(timesteps=16):
    r"""
    Computes the state cost matrix ``H_Q``:

    .. math::
        \mathbf{H}_\mathbf{Q} = 2\mathrm{blockdiag}\Big(\mathbf{Q}_1, \cdots, \mathbf{Q}_N\Big)\in\mathbb{R}^{15N\times15N}

    :param timesteps: the number of timestep considered within the MPC preview horizon
    :type timesteps: int
    :return: state cost matrix ``H_Q``
    :rtype: ndarray
    """

    Q_step = get_Q_step()
    Q_step_list = [Q_step] * timesteps
    H_Q = block_diag(*Q_step_list)

    return H_Q


def get_HR(timesteps=16):
    r"""
    Computes the control cost matrix ``H_R`` with shape:

    .. math::
        \mathbf{H}_\mathbf{R} = 2\mathrm{blockdiag}\Big(\mathbf{R}_{0}, \cdots, \mathbf{R}_{N-1}\Big)\in\mathbb{R}^{12N\times12N}

    :param timesteps: the number of timestep considered within the MPC preview horizon
    :type timesteps: int
    :return: state cost matrix ``H_R``
    :rtype: ndarray
    """

    R_step = np.eye(12)
    R_step_list = [R_step] * timesteps
    H_R = block_diag(*R_step_list)

    return H_R


def get_H(timesteps=16):
    r"""
    Computes the quadratic cost matrix ``H``:

    .. math::
        \mathbf{H} = \begin{bmatrix}
            \mathbf{H}_\mathbf{Q} & \mathbf{0}_{15N\times12N}\\
            \mathbf{0}_{12N\times15N} & \mathbf{H}_\mathbf{R}
        \end{bmatrix}\in\mathbb{R}^{27N\times27N}

    :param timesteps: the number of timestep considered within the MPC preview horizon
    :type timesteps: int
    :return: quadratic cost matrix ``H``
    :rtype: ndarray
    """

    H_Q = get_HQ(timesteps=timesteps)
    H_R = get_HR(timesteps=timesteps)
    H = block_diag(H_Q, H_R)

    return H


def get_gx(x_ref_mat, timesteps=16):
    r"""
    Computes the linear state cost matrix ``g_x``:

    .. math::
        \mathbf{g}_\mathbf{x} = \begin{bmatrix}
            -2\mathbf{Q}_{1}^T\mathbf{x}_{1}^\mathrm{ref}\\
            \vdots\\
            -2\mathbf{Q}_{N}^T\mathbf{x}_{N}^\mathrm{ref}
        \end{bmatrix}\in\mathbb{R}^{15N\times1}

    by taking ``x_ref_mat`` as its input

    .. math::
        \mathbf{x}^\mathrm{ref} = \begin{bmatrix}
            \mathbf{x}_1^\mathrm{ref}\\
            \vdots\\
            \mathbf{x}_N^\mathrm{ref}
        \end{bmatrix}\in\mathbb{R}^{15N\times1}

    and performing the computation

    .. math::
        \mathbf{g}_\mathbf{x} = -2\mathbf{H}_\mathbf{Q}\mathbf{x}^\mathrm{ref}

    :param x_ref_mat: the reference motion of the centroidal model
    :type x_ref_mat: ndarray
    :param timesteps: the number of timestep considered within the MPC preview horizon
    :type timesteps: int
    :return: linear state cost matrix ``g_x``
    :rtype: ndarray
    """

    H_Q = get_HQ(timesteps=timesteps)
    g_x = -2 * H_Q @ x_ref_mat

    return g_x


def get_g(x_ref_mat, timesteps=16):
    r"""
    Computes the linear cost matrix ``g``:

    .. math::
        \mathbf{g} = \begin{bmatrix}
            \mathbf{g}_\mathbf{x}\\
            \mathbf{g}_\mathbf{f}
        \end{bmatrix}\in\mathbb{R}^{27N\times1}

    by taking ``x_ref_mat`` as its input

    .. math::
        \mathbf{x}^\mathrm{ref} = \begin{bmatrix}
            \mathbf{x}_1^\mathrm{ref}\\
            \vdots\\
            \mathbf{x}_N^\mathrm{ref}
        \end{bmatrix}\in\mathbb{R}^{15N\times1}

    :param x_ref_mat: the reference motion of the centroidal model
    :type x_ref_mat: ndarray
    :param timesteps: the number of timestep considered within the MPC preview horizon
    :type timesteps: int
    :return: linear cost matrix ``g``
    :rtype: ndarray
    """

    g_x = get_gx(x_ref_mat, timesteps=timesteps)
    g_f = np.zeros((12 * timesteps, 1))
    g = np.vstack((g_x, g_f))

    return g


def get_bcf(mu=0.6):
    r"""
    Computes the friction cone coefficient matrix ``bc_f``:

    .. math::
        \bar{\mathbf{c}}_\mathbf{f} = \begin{bmatrix}
            0 & 0 & -1\\
            0 & 0 & 1\\
            -1 & 0 & -\mu\\
            1 & 0 & -\mu\\
            0 & -1 & -\mu\\
            0 & 1 & -\mu
        \end{bmatrix}\in\mathbb{R}^{6\times3}

    :param mu: the friction coefficient ``mu``
    :type mu: float
    :return: friction cone coefficient matrix ``bc_f``
    :rtype: ndarray
    """

    bc_f = np.array(
        [[0, 0, -1], [0, 0, 1], [-1, 0, -mu], [1, 0, -mu], [0, -1, -mu], [0, 1, -mu]]
    )

    return bc_f


def get_bubf(f_min=1e-3, f_max=10.0):
    r"""
    Computes the friction cone constraint upper bound vector ``bub_f``:

    .. math::
        \bar{\mathrm{ub}}_\mathbf{f} = \begin{bmatrix}
            -f_\min\\
            f_\max\\
            0\\
            0\\
            0\\
            0
        \end{bmatrix}

    :param f_min: the minimum support force
    :type f_min: float
    :param f_max: the maximum support force
    :type f_max: float
    :return: friction cone constraint upper bound vector ``bub_f``
    :rtype: ndarray
    """

    bub_f = np.array([[-f_min], [f_max], [0.0], [0.0], [0.0], [0.0]])

    return bub_f


def get_C(n_st, M_st, timesteps=16, mu=0.6):
    r"""
    Computes the inequality constraint matrix ``C``:

    .. math::
        \mathbf{C} = \begin{bmatrix}
            \mathbf{0} & \mathbf{C}_\mathrm{f}
        \end{bmatrix}\in\mathbb{R}^{6n_{st}N\times27N}

    where

    .. math::
        \mathbf{C}_\mathrm{f} = \mathrm{blockdiag}\Big(\mathbf{c}_{\mathbf{f}}\mathcal{M}_{st, 0}, \cdots, \mathbf{c}_{\mathbf{f}}\mathcal{M}_{st, N-1}\Big)\in\mathbb{R}^{6n_{st}N\times12N}

    We compute this by first computing

    .. math::
        \mathbf{c}_{\mathbf{f}} = \mathrm{blockdiag}\Big(\underbrace{\bar{\mathbf{c}}_{\mathbf{f}}, \cdots, \bar{\mathbf{c}}_{\mathbf{f}}}_{n_{st}}\Big)\in\mathbb{R}^{6n_{st}\times3n_{st}}

    and then constructing the matrix

    .. math::
        \texttt{c_f_mat} = \begin{bmatrix}
            \mathbf{c}_{\mathbf{f}} & \cdots & \mathbf{0}\\
            \vdots & \ddots & \vdots\\
            \mathbf{0} & \cdots & \mathbf{c}_{\mathbf{f}}
        \end{bmatrix}\in\mathbb{R}^{6n_{st}N\times3n_{st}N}

    The input ``M_st`` is the matrix

    .. math::
        \texttt{M_st} = \begin{bmatrix}
            \mathcal{M}_{st, 1} & \cdots & \mathbf{0}\\
            \vdots & \ddots & \vdots\\
            \mathbf{0} & \cdots & \mathcal{M}_{st, N}
        \end{bmatrix}\in\mathbb{R}^{3n_{st}N\times12N}

    We can then compute ``C_f`` as

    .. math::
        \mathbf{C}_\mathbf{f} = \texttt{c_f_mat}\texttt{M_st} = \mathrm{blockdiag}\Big(\mathbf{c}_{\mathbf{f}}\mathcal{M}_{st, 0}, \cdots, \mathbf{c}_{\mathbf{f}}\mathcal{M}_{st, N-1}\Big)\in\mathbb{R}^{6n_{st}N\times12N}

    :param n_st: the number of stance feet at each time instance (gait dependent)
    :type n_st: int
    :param M_st: the stance foot selection matrix
    :type M_st: ndarray
    :param timesteps: the number of timestep considered within the MPC preview horizon
    :type timesteps: int
    :param mu: the friction coefficient ``mu``
    :type mu: float
    :return: inequality constraint matrix ``C``
    :rtype: ndarray
    """

    bc_f = get_bcf(mu=mu)
    bc_f_list = [bc_f] * n_st

    c_f = block_diag(*bc_f_list)
    c_f_list = [c_f] * timesteps
    c_f_mat = block_diag(*c_f_list)

    C_f = c_f_mat @ M_st
    Zeros = np.zeros((6 * n_st * timesteps, 15 * timesteps))

    C = np.hstack((Zeros, C_f))

    return C


def get_UB(n_st, f_min=1e-3, f_max=10.0, timesteps=16):
    r"""
    Computes the inequality constraint upper bound ``UB``:

    .. math::
        \mathrm{UB} = \mathrm{UB}_\mathbf{f} = \mathrm{vstack}\Big(\mathrm{ub}_\mathbf{f}, \cdots, \mathrm{ub}_\mathbf{f}\Big)\in\mathbb{R}^{6n_{st}N\times1}

    First, it computes the value of ``bub_f``. Then, it computes the value of ``ub_f`` as

    .. math::
        \mathrm{ub}_\mathbf{f} = \mathrm{vstack}\Big(\bar{\mathrm{ub}}_\mathbf{f}, \cdots, \bar{\mathrm{ub}}_\mathbf{f}\Big)\in\mathbb{R}^{6n_{st}\times1}

    :param n_st: the number of stance feet at each time instance (gait dependent)
    :type n_st: int
    :param f_min: the minimum support force
    :type f_min: float
    :param f_max: the maximum support force
    :type f_max: float
    :param timesteps: the number of timestep considered within the MPC preview horizon
    :type timesteps: int
    :return: inequality constraint upper bound ``UB``
    :rtype: ndarray
    """

    bub_f = get_bubf(f_min=f_min, f_max=f_max)
    bub_f_list = [bub_f] * n_st

    ub_f = np.concatenate(bub_f_list, axis=0)
    ub_f_list = [ub_f] * timesteps

    UB = np.concatenate(ub_f_list, axis=0)

    return UB


def get_b(bA_0, x_0, n_sw, timesteps=16):
    r"""
    Computes the ``b`` matrix in the QP equality constraint. The ``b`` matrix is defined as

    .. math::
        \mathbf{b} = \begin{bmatrix}
            \mathbf{b}_d\\
            \mathbf{b}_f
        \end{bmatrix}\in\mathbb{R}^{(15 + 3n_{sw})N\times1}

    with

    .. math::
        \mathbf{b}_d = \begin{bmatrix}
            \bar{\mathbf{A}}_0\mathbf{x}_0\\
            \mathbf{0}
        \end{bmatrix}\in\mathbb{R}^{15N\times1} \quad \text{&} \quad \mathbf{b}_f = \mathbf{0}\in\mathbb{R}^{3n_{sw}N\times1}.

    :param bA_0: the drift matrix at the current state
    :type bA_0: ndarray
    :param x_0: the current centroidal dynamics state
    :type x_0: ndarray
    :param n_sw: the number of swing feet at each time instance (gait dependent)
    :type n_sw: int
    :param timesteps: the number of timestep considered within the MPC preview horizon
    :type timesteps: int
    :return: the ``b`` matrix in the QP equality constraint
    :rtype: ndarray
    """

    b_f = np.zeros((3 * n_sw * timesteps, 1))
    b_d = np.vstack((bA_0 @ x_0, np.zeros((14 * timesteps, 1))))

    b = np.vstack((b_d, b_f))

    return b


def get_A(bA, bB, M_sw, n_sw, timesteps=16):
    r"""
    Computes the ``A`` matrix in the QP equality constraint. The ``A`` matrix is defined as

    .. math::
        \mathbf{A} = \begin{bmatrix}
            \mathbf{A}_d\\
            \mathbf{A}_f
        \end{bmatrix}\in\mathbb{R}^{(15+3n_{sw})N\times27N}

    with

    .. math::
        \mathbf{A}_d = \begin{bmatrix}
            \bar{\mathbf{A}} & \bar{\mathbf{B}}
        \end{bmatrix}\in\mathbb{R}^{15N\times27N} \quad \text{&} \quad \mathbf{A}_f = \begin{bmatrix}
            \mathbf{0} & \mathbf{M}_\mathrm{sw}
        \end{bmatrix}\in\mathbb{R}^{3n_{sw}N\times27N}.
    
    And the matrix ``M_sw`` is defined as

    .. math::
        \mathbf{M}_\mathrm{sw} = \mathrm{blockdiag}\Big(\mathcal{M}_{sw, 0}, \cdots, \mathcal{M}_{sw, N-1}\Big)\in\mathbb{R}^{3n_{sw}N\times12N}.

    :param bA: the aggregated drift matrix
    :type bA: ndarray
    :param bB: the aggregated control matrix
    :type bB: ndarray
    :param M_sw: the swing foot selection matrix
    :type M_sw: ndarray
    :param n_sw: the number of swing feet at each time instance (gait dependent)
    :type n_sw: int
    :param timesteps: the number of timestep considered within the MPC preview horizon
    :type timesteps: int
    :return: the ``A`` matrix in the QP equality constraint
    :rtype: ndarray
    """

    A_d = np.hstack((bA, bB))
    A_f = np.hstack((np.zeros((3 * n_sw * timesteps, 15 * timesteps)), M_sw))

    A = np.vstack((A_d, A_f))

    return A
