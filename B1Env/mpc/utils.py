import numpy as np

from scipy.linalg import block_diag
from pdb import set_trace


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
