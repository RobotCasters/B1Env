import numpy as np


class CoMVelocityPlanner:
    def __init__(self, config):
        self.config = config

    
    def get_new_com_velocity(self, v_i, v_cmd):
        r"""
        Computes the center of mass velocity ``v_i+1`` in the global frame by taking ``v_i`` and ``v_cmd`` as inputs:

        .. math::
            \mathbf{v}_{i+1} = (1-\alpha)\mathbf{v}_i + \alpha \mathbf{v}_{cmd} \in\mathbb{R}^{3\times1}
        
        :param v_i: center of mass velocity at time step ``i`` in global frame
        :type v_i: ndarray
        :param v_cmd: commanded center of mass velocity in global frame
        :type v_cmd: ndarray
        :return: center of mass velocity at time step ``i+1`` in global frame
        :rtype: ndarray
        """

        return (1 - self.config.v_alpha)*v_i + self.config.v_alpha*v_cmd


    def get_new_com_velocity_horizon(self, v_i, v_cmd_horizon, N):
        r"""
        Computes the center of mass velocities ``v_i_horizon`` over a time horizon ``N``:

        .. math::
            \mathbf{v}_i^{horizon} =  \begin{bmatrix}
                \mathbf{v}_{i+1}\\
                \vdots\\
                \mathbf{v}_{i+N}
            \end{bmatrix}\in\mathbb{R}^{3N\times1}
        
        :param v_i: center of mass velocity at time step ``i`` in global frame
        :type v_i: ndarray
        :param v_cmd_horizon: commanded center of mass velocities over a horizon ``N``
        :type v_cmd_horizon: ndarray
        :param N: horizon length
        :type N: int
        :return: center of mass velocities over a horizon ``N``
        :rtype: ndarray
        """

        com_velocity_horizon = self.get_new_com_velocity(v_i, v_cmd_horizon[0])
        v_1 = v_i
        for i in range(N-1):
            com_velocity_horizon = np.append(com_velocity_horizon, self.get_new_com_velocity(v_1, v_cmd_horizon[i+1]))
            v_1 = self.get_new_com_velocity(v_1, v_cmd_horizon[i+1])
        return com_velocity_horizon


    def get_x_ref_mat(self, v_i, v_cmd, e_z_ref):
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

        :param v_i: center of mass velocity at time step ``i`` in global frame
        :type v_i: ndarray
        :param v_cmd: commanded center of mass velocity in global frame
        :type v_cmd: ndarray
        :param e_z_ref: center of mass position in z direction in global frame
        :type e_z_ref: ndarray
        :return: vector with information about center of mass in z direction and commanded center of mass velocity in global frame
        :rtype: ndarray
        """

        v_ref = self.get_new_com_velocity(v_i, v_cmd)
        return np.stack((e_z_ref, np.zeros(3), v_ref, np.zeros(3), np.zeros(3))).ravel()


    def get_x_ref_mat_horizon(self, v_i, v_cmd_horizon, e_z_ref_horizon, N):
        r"""
        Computes the reference state vector ``x_ref_mat_horizon`` over a time horizon ``N``:

        .. math::
            \mathbf{x}^\mathrm{ref,horizon} = \begin{bmatrix}
                \mathbf{x}_{i+1}^\mathrm{ref}\\
                \vdots\\
                \mathbf{x}_{i+N}^\mathrm{ref}
            \end{bmatrix}\in\mathbb{R}^{15N\times1}
        
        :param v_i: center of mass velocity at time step ``i`` in global frame
        :type v_i: ndarray
        :param v_cmd_horizon: commanded velocity over a horizon
        :type v_cmd_horizon: ndarray
        :param e_z_ref_horizon: center of mass position in z direction over a horizon ``N`` in global frame
        :type e_z_ref_horizon: ndarray
        :param N: horizon length
        :type N: int
        :return: center of mass positions and commanded velocities over a time horizon ``N`` in global frame
        :rtype: ndarray
        """

        x_ref_mat_horizon = self.get_x_ref_mat(v_i, v_cmd_horizon[0], e_z_ref_horizon[0])
        v_i_1 = self.get_new_com_velocity(v_i,v_cmd_horizon[0])
        for i in range(N-1):
            x_ref_mat_horizon = np.append(x_ref_mat_horizon, self.get_x_ref_mat(v_i_1, v_cmd_horizon[i+1], e_z_ref_horizon[i+1]))
            v_i_1 = self.get_new_com_velocity(v_i_1,v_cmd_horizon[i+1])
        return x_ref_mat_horizon

    def get_new_theta(self, theta_i, R_z, omega_i):
        r"""
        Computes the angles ``theta_i+1`` in body frame:

        .. math::
            \Theta_{i+1} = \Theta_i + \mathrm{R}_z(\psi)\omega_i \Delta t 
        
        :param theta_i: angles at time step ``i`` in body frame
        :type theta_i: ndarray
        :param R_z: rotation matrix to translate global frame to CoM frame
        :type R_z: ndarray
        :param omega_i: angular velocity at time step ``i`` in global frame
        :type omega_i: ndarray
        :return: angles at time step ``i+1`` in body frame
        :rtype: ndarray
        """

        return (R_z @ omega_i)*self.config.dt + theta_i

    def get_I_G(self, R_z, I_B):
        r"""
        Computes the moment of inertia in global frame ``I_G``:

        .. math::
            \mathbf{I}_\mathcal{G} = \mathrm{R}_z(\psi)\mathbf{I}_\mathcal{B}\mathrm{R}_z^T(\psi)
        
        :param R_z: rotation matrix translating global frame to CoM frame
        :type R_z: ndarray
        :param I_B: moment of inertia in body frame
        :type I_B: ndarray
        :return: moment of inertia in global frame
        :rtype: ndarray
        """

        return R_z @ I_B @ R_z.transpose()

    def get_new_omega(self, omega_i, R_z, I_B, r_FL, r_FR, r_HL, r_HR, f_FL, f_FR, f_HL, f_HR):
        r"""
        Computes the angular velocity ``omega_i+1``:

        .. math::
            \omega_{i+1} = \omega_i + \mathbf{I}_\mathcal{G}^{-1}([\mathbf{r}_\mathrm{FL}]_\times \mathbf{f}_\mathrm{FL} + [\mathbf{r}_\mathrm{FR}]_\times \mathbf{f}_\mathrm{FR} + [\mathbf{r}_\mathrm{HL}]_\times \mathbf{f}_\mathrm{HL} + [\mathbf{r}_\mathrm{HR}]_\times \mathbf{f}_\mathrm{HR})
        
        :param omega_i: angular velocity at time step ``i`` in body frame
        :type omega_i: ndarray
        :param R_z: rotation matrix translating global frame to CoM frame
        :type R_z: ndarray
        :param I_B: moment of inertia in body frame
        :type I_B: ndarray
        :param r_FL: vector from CoM to the front left leg
        :type r_FL: ndarray
        :param r_FR: vector from CoM to the front right leg
        :type r_FR: ndarray
        :param r_HL: vector from CoM to the hind left leg
        :type r_HL: ndarray
        :param r_HR: vector from CoM to the hind right leg
        :type r_HR: ndarray
        :param f_FL: ground reaction force at the front left leg
        :type f_FL: ndarray
        :param f_FR: ground reaction force at the front right leg
        :type f_FR: ndarray
        :param f_HL: ground reaction force at the hind left leg
        :type f_HL: ndarray
        :param f_HR: ground reaction force at the hind right leg
        :type f_HR: ndarray
        :return: angular velocity at time step ``i+1`` in body frame
        :rtype: ndarray
        """

        I_G_inv = np.linalg.inv(self.get_I_G(R_z, I_B))
        return omega_i + (I_G_inv @ (self.get_skew_symm_matrix(r_FL)@f_FL + self.get_skew_symm_matrix(r_FR)@f_FR + self.get_skew_symm_matrix(r_HL)@f_HL + self.get_skew_symm_matrix(r_HR)@f_HR)) * self.config.dt

    def get_skew_symm_matrix(self, vector):
        r"""
        Computes the skew symmetric matrix of a given vector ``vector_skew``:

        .. math::
            [\mathbf{vector}]_\times = \begin{bmatrix}
                                        0 & -vector[2] & vector[1]\\
                                        vector[2] & 0 & -vector[0]\\
                                        -vector[1] & vector[0] & 0
                                    \end{bmatrix}
        
        :param vector: generic 3 X 1 vector
        :type vector: ndarray
        :return: skew symmetric matrix
        :rtype: ndarray
        """

        return np.array([
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0]
        ])
        