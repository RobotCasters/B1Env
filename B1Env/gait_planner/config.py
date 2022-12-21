import numpy as np


class Configuration:
    def __init__(self):  
        self.dt = 0.01
        
        self.offset = 0
        self.gait_type = "walk"   # walk/trot/gallop

        self.mst_trot_0 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])
        self.mst_trot_1 = np.block([
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))]
        ])

        self.mst_trot = [self.mst_trot_0, self.mst_trot_1]

        self.msw_trot_0 = np.block([
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))]
        ])
        self.msw_trot_1 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])

        self.msw_trot = [self.msw_trot_0, self.msw_trot_1]

        self.mst_walk_0 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])
        self.mst_walk_1 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])
        self.mst_walk_2 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))]
        ])
        self.mst_walk_3 = np.block([
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])

        self.mst_walk = [self.mst_walk_0, self.mst_walk_1, self.mst_walk_2, self.mst_walk_3]

        self.msw_walk_0 = np.block([
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))]
        ])
        self.msw_walk_1 = np.block([
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))]
        ])
        self.msw_walk_2 = np.block([
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])
        self.msw_walk_3 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))]
        ])

        self.msw_walk = [self.msw_walk_0, self.msw_walk_1, self.msw_walk_2, self.mst_walk_3]

        self.mst_gallop_0 = np.block([
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))]
        ])
        self.mst_gallop_1 = np.block([
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))]
        ])
        self.mst_gallop_2 = np.block([
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])
        self.mst_gallop_3 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))]
        ])

        self.mst_gallop = [self.mst_gallop_0, self.mst_gallop_1, self.mst_gallop_2, self.mst_gallop_3]

        self.msw_gallop_0 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])
        self.msw_gallop_1 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])
        self.msw_gallop_2 = np.block([
            [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))]
        ])
        self.msw_gallop_3 = np.block([
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]
        ])

        self.msw_gallop = [self.msw_gallop_0, self.msw_gallop_1, self.msw_gallop_2, self.msw_gallop_3]

        self.gait_frequency = 4

        self.stance_time = self.get_duty_factor/self.gait_frequency

        self.swing_time = (1-self.get_duty_factor)/self.gait_frequency
        self.num_phases = len(self.phase_ticks)
        self.v_alpha = 0.02


    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return int(self.stance_time / self.dt)

    @property
    def phase_ticks(self):
        if self.gait_type == "walk":
            return np.array([self.swing_ticks, self.swing_ticks, self.swing_ticks, self.swing_ticks])
        elif self.gait_type == "trot":
            return np.array([self.stance_ticks, self.swing_ticks])
        elif self.gait_type == "gallop":
            return np.array([self.stance_ticks, self.stance_ticks])

    @property
    def phase_length(self):
        return self.swing_ticks + self.stance_ticks

    @property
    def mst_gait(self):
        if self.gait_type == "walk":
            return self.mst_walk
        elif self.gait_type == "trot":
            return self.mst_trot
        elif self.gait_type == "gallop":
            return self.mst_gallop
    
    @property
    def msw_gait(self):
        if self.gait_type == "walk":
            return self.msw_walk
        elif self.gait_type == "trot":
            return self.msw_trot
        elif self.gait_type == "gallop":
            return self.msw_gallop

    @property
    def get_duty_factor(self):
        if self.gait_type == "walk":
            return 0.75
        elif self.gait_type == "trot":
            return 0.5
        elif self.gait_type == "gallop":
            return 0.25
