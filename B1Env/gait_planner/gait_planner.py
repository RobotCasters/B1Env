import numpy as np
from scipy.linalg import block_diag
np.set_printoptions(threshold=np.inf)


class GaitPlanner:
    def __init__(self, config):
        self.config = config


    def phase_index(self, ticks):
        r"""
        Computes which part of the gait cycle the robot is in ``phase_index``:

        .. math::
            if (offset + clock_time) % cycle_length <= stance_duration:
                IN_STANCE_PHASE = True
            else:
                IN_STANCE_PHASE = False
   
        :param ticks: the number of timesteps that have passed since the program started
        :type ticks: int
        :return: the index of gait phase that the robot is in
        :rtype: int
        """

        phase_time = (ticks + self.config.offset) % self.config.phase_length
        phase_sum = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                return i
        assert False


    def subphase_ticks(self, ticks):
        r"""
        Calculates the number of timesteps since the start of the current phase

        :param ticks: the number of timesteps that have passed since the program started
        :type ticks: int
        :return: the number of timesteps since the start of the current phase
        :rtype: int
        """

        phase_time = (ticks + self.config.offset) % self.config.phase_length
        phase_sum = 0
        subphase_ticks = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - phase_sum + self.config.phase_ticks[i]
                return subphase_ticks
        assert False


    def get_Mst_i(self, ticks):
        r"""
        Computes which legs are in stance position at a given timestep by matrix ``M_st,i``:

        The size of $\mathcal{M}_{st,i}$ is $3n_{st} \times 12$, where $n_{st}$ is the number of stance feet. If the front left (FL) and hind right (HR) feet are the stance feet then we have 

        .. math:: 
            \mathcal{M}_{st,i} = \begin{bmatrix}
                \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
                \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3
            \end{bmatrix}\in\mathbb{R}^{6\times12}

        If the front left (FL), front right (FR), and hind right (HR) feet are the stance feet then we have

        .. math:: 
            \mathcal{M}_{st,i} = \begin{bmatrix}
                \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
                \mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
                \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3
            \end{bmatrix}\in\mathbb{R}^{9\times12}
        
        :param ticks: the number of timesteps that have passed since the program started
        :type ticks: int
        :return: the block matrix with the information about the legs that are in stance phase 
        :rtype: ndarray
        """

        return self.config.mst_gait[self.phase_index(ticks)]


    def get_Mst(self, ticks, N):
        r"""
        Computes which legs are in stance position over a time horizon by giving matrix ``M_st``:

        .. math::
            \mathbf{M}_\mathrm{st} = \mathrm{blockdiag}\Big(\mathcal{M}_{st, 0}, \cdots, \mathcal{M}_{st, N-1}\Big)\in\mathbb{R}^{3n_{st}N\times12N}
        
        :param ticks: the number of timesteps that have passed since the program started
        :type ticks: int
        :param N: the time horizon
        :type N: int
        :return: the block matrix for a time horizon with the information about the legs that are in stance phase 
        :rtype: ndarray
        """

        mst_horizon_rem0 = block_diag(*self.config.mst_gait)
        mst_horizon_rem0 = np.roll(mst_horizon_rem0,-12*self.phase_index(ticks),axis=1)
        num_phases_horizon = int(N/self.config.num_phases)
        if N%self.config.num_phases == 0:
            return block_diag(*([mst_horizon_rem0] * num_phases_horizon))
        else:
            mst_horizon = block_diag(*([mst_horizon_rem0] * num_phases_horizon))
            for i in range(N%self.config.num_phases):
                mst_horizon = block_diag(mst_horizon, self.config.mst_gait[(self.phase_index(ticks)+i)%self.config.num_phases])
            return mst_horizon


    def get_Msw_i(self, ticks):
        r"""
        Computes which legs are in swing position at a given timestep by giving matrix ``M_sw,i``:

        The size of $\mathcal{M}_{sw,i}$ is $3n_{sw} \times 12$, where $n_{sw}$ is the number of swing feet. If the front left (FL) and hind right (HR) feet are the swing feet then we have 

        .. math:: 
            \mathcal{M}_{sw,i} = \begin{bmatrix}
                \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
                \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3
            \end{bmatrix}\in\mathbb{R}^{6\times12}

        If the front left (FL), front right (FR), and hind right (HR) feet are the swing feet then we have

        .. math:: 
            \mathcal{M}_{sw,i} = \begin{bmatrix}
                \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
                \mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
                \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3
            \end{bmatrix}\in\mathbb{R}^{9\times12}
        
        :param ticks: the number of timesteps that have passed since the program started
        :type ticks: int
        :return: the block matrix with the information about the legs that are in swing phase 
        :rtype: ndarray
        """

        return self.config.msw_gait[self.phase_index(ticks)]


    def get_Msw(self, ticks, N):
        r"""
        Computes which legs are in swing position for a given time horizon by giving matrix ``M_sw``:

        .. math::
            \mathbf{M}_\mathrm{sw} = \mathrm{blockdiag}\Big(\mathcal{M}_{sw, 0}, \cdots, \mathcal{M}_{sw, N-1}\Big)\in\mathbb{R}^{3n_{sw}N\times12N}
        
        :param ticks: the number of timesteps that have passed since the program started
        :type ticks: int
        :param N: the time horizon
        :type N: int
        :return: the block matrix for a time horizon with the information about the legs that are in swing phase 
        :rtype: ndarray
        """

        msw_horizon_rem0 = block_diag(*self.config.msw_gait)
        msw_horizon_rem0 = np.roll(msw_horizon_rem0,-12*self.phase_index(ticks),axis=1)
        num_phases_horizon = int(N/self.config.num_phases)
        if N%self.config.num_phases == 0:
            return block_diag(*([msw_horizon_rem0] * num_phases_horizon))
        else:
            msw_horizon = block_diag(*([msw_horizon_rem0] * num_phases_horizon))
            for i in range(N%self.config.num_phases):
                msw_horizon = block_diag(msw_horizon, self.config.msw_gait[(self.phase_index(ticks)+i)%self.config.num_phases])
            return msw_horizon