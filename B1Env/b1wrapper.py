import numpy as np
import pybullet as p
from bullet_utils.wrapper import PinBulletWrapper

from B1Env import getDataPath
from B1Env.config import B1Config

dt = 1e-3


class B1Robot(PinBulletWrapper):
    """
    PinBulletWrapper: https://github.com/machines-in-motion/bullet_utils/blob/main/src/bullet_utils/wrapper.py
    """

    def __init__(
        self,
        pos=None,
        orn=None,
        useFixedBase=False,
        init_sliders_pose=4
        * [
            0,
        ],
    ):
        # Load the robot
        if pos is None:
            pos = [0.0, 0, 1.0]
        if orn is None:
            orn = p.getQuaternionFromEuler([0, 0, 0])

        self.config = B1Config()

        p.setAdditionalSearchPath(getDataPath())
        self.urdf_path = self.config.urdf_path
        self.robotId = p.loadURDF(
            self.urdf_path,
            pos,
            orn,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=useFixedBase,
        )
        p.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        self.pin_robot = self.config.buildRobotWrapper()

        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)

        for ji in range(num_joints):
            p.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.slider_a = p.addUserDebugParameter("a", 0, 1, init_sliders_pose[0])
        self.slider_b = p.addUserDebugParameter("b", 0, 1, init_sliders_pose[1])
        self.slider_c = p.addUserDebugParameter("c", 0, 1, init_sliders_pose[2])
        self.slider_d = p.addUserDebugParameter("d", 0, 1, init_sliders_pose[3])

        self.base_link_name = "base_link"
        self.end_eff_ids = []
        self.end_effector_names = []
        controlled_joints = []

        for leg in ["FR", "FL", "HR", "HL"]:
            controlled_joints += [
                leg + "_hip_joint",
                leg + "_thigh_joint",
                leg + "_knee_joint",
            ]
            self.end_eff_ids.append(self.pin_robot.model.getFrameId(leg + "_foot"))
            self.end_effector_names.append(leg + "_foot")

        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

        self.fr_index = self.pin_robot.model.getFrameId("FR_ankle_fixed")
        self.fl_index = self.pin_robot.model.getFrameId("FL_ankle_fixed")
        self.hr_index = self.pin_robot.model.getFrameId("HR_ankle_fixed")
        self.hl_index = self.pin_robot.model.getFrameId("HL_ankle_fixed")

        # Creates the wrapper by calling the super.__init__.
        super(B1Robot, self).__init__(
            self.robotId,
            self.pin_robot,
            controlled_joints,
            [
                "FR_ankle_fixed",
                "FL_ankle_fixed",
                "HR_ankle_fixed",
                "HL_ankle_fixed",
            ],
        )

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

    def get_slider_position(self, letter):
        try:
            if letter == "a":
                return p.readUserDebugParameter(self.slider_a)
            if letter == "b":
                return p.readUserDebugParameter(self.slider_b)
            if letter == "c":
                return p.readUserDebugParameter(self.slider_c)
            if letter == "d":
                return p.readUserDebugParameter(self.slider_d)
        except Exception:
            # In case of not using a GUI.
            return 0.0

    def reset_to_initial_state(self) -> None:
        """Reset robot state to the initial configuration (based on B1Config)."""
        q0 = np.matrix(self.config.initial_configuration).T
        dq0 = np.matrix(self.config.initial_velocity).T
        self.reset_state(q0, dq0)
