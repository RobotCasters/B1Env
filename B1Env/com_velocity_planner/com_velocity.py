import numpy as np


class CoMVelocityPlanner:
    def __init__(self, config):
        self.config = config

    
    def get_com_new_velocity(self, c_dot, v_cmd):
        r"""
        Computes the center of mass velocity for next timestep ``v_i+1``:

        .. math::
            \mathbf{v}_i = (1-\alpha)\mathbf{v} + \alpha \mathbf{v}_{cmd} \in\mathbb{R}^{3\times1}
        
        :param c_dot: current center of mass velocity
        :type c_dot: ndarray
        :param v_cmd: commanded center of mass velocity
        :type v_cmd: ndarray
        :return: center of mass velocity for the next timestep 
        :rtype: ndarray
        """

        return (1 - self.config.v_alpha)*c_dot + self.config.v_alpha*v_cmd


    def get_com_new_velocity_horizon(self, c_dot, v_cmd_horizon, N):
        r"""
        Computes the center of mass velocity over a time horizon ``v_i_horizon``:

        .. math::
            \mathbf{v}_i^{horizon} =  \begin{bmatrix}
                \mathbf{v}_1\\
                \vdots\\
                \mathbf{v}_N
            \end{bmatrix}\in\mathbb{R}^{3N\times1}
        
        :param c_dot: current center of mass velocity
        :type c_dot: ndarray
        :param v_cmd_horizon: commanded center of mass velocity over a horizon
        :type v_cmd_horizon: ndarray
        :param N: horizon
        :type N: int
        :return: center of mass velocity over a horizon 
        :rtype: ndarray
        """

        com_velocity_horizon = self.get_com_new_velocity(c_dot, v_cmd_horizon[0])
        c_dot_1 = c_dot
        for i in range(N-1):
            com_velocity_horizon = np.append(com_velocity_horizon, self.get_com_new_velocity(c_dot_1, v_cmd_horizon[i+1]))
            c_dot_1 = self.get_com_new_velocity(c_dot_1, v_cmd_horizon[i+1])
        return com_velocity_horizon


    def get_x_ref_mat(self, v_cmd, e_z_ref):
        r"""
        Computes the reference state vector ``x_ref_mat``:
        
        .. math::
            \mathbf{x}_{i+1}^\mathrm{ref} = \begin{bmatrix}
                \mathbf{e}_z^\mathrm{ref}\\
                \mathbf{0}_{3\times1}\\
                \mathbf{v}_{i+1}^\mathrm{ref}\\
                \mathbf{0}_{3\times1}\\
                \mathbf{0}_{3\times1}
            \end{bmatrix}\in\mathbb{R}^{15\times1}

        :param v_cmd: commanded velocity
        :type v_cmd: ndarray
        :param e_z_ref: center of mass position in z direction
        :type e_z_ref: ndarray
        :return: vector with information about center of mass in z direction and commanded velocity 
        :rtype: ndarray
        """

        return np.stack((e_z_ref, np.zeros(3), v_cmd, np.zeros(3), np.zeros(3))).ravel()


    def get_x_ref_mat_horizon(self, v_cmd_horizon, e_z_ref_horizon, N):
        r"""
        Computes the reference state vector over a time horizon ``x_ref_mat_horizon``:

        .. math::
            \mathbf{x}^\mathrm{ref} = \begin{bmatrix}
                \mathbf{x}_1^\mathrm{ref}\\
                \vdots\\
                \mathbf{x}_N^\mathrm{ref}
            \end{bmatrix}\in\mathbb{R}^{15N\times1}
        
        :param v_cmd_horizon: commanded velocity over a horizon
        :type v_cmd_horizon: ndarray
        :param e_z_ref_horizon: center of mass position in z direction over a horizon
        :type e_z_ref_horizon: ndarray
        :param N: horizon
        :type N: int
        :return: center of mass position and commanded velocity over a time horizon
        :rtype: ndarray
        """

        x_ref_mat_horizon = self.get_x_ref_mat(v_cmd_horizon[0], e_z_ref_horizon[0])
        for i in range(N-1):
            x_ref_mat_horizon = np.append(x_ref_mat_horizon, self.get_x_ref_mat(v_cmd_horizon[i+1], e_z_ref_horizon[i+1]))
        return x_ref_mat_horizon
