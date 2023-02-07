from pdb import set_trace

import numpy as np
import pinocchio as pin
import pybullet as p
from bullet_utils.env import BulletEnvWithGround
from bullet_utils.wrapper import PinBulletWrapper
from pinocchio.robot_wrapper import RobotWrapper

from B1Env import getDataPath


class B1Robot(PinBulletWrapper):
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

        package_directory = getDataPath()
        robot_URDF = package_directory + "/robots/b1_raw.urdf"
        urdf_search_path = package_directory + "/robots"
        p.setAdditionalSearchPath(urdf_search_path)

        self.robotId = p.loadURDF(
            robot_URDF,
            pos,
            orn,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=useFixedBase,
        )
        p.getBasePositionAndOrientation(self.robotId)

        self.robot = RobotWrapper.BuildFromURDF(
            robot_URDF, package_directory, pin.JointModelFreeFlyer()
        )


def main():
    # Create a Pybullet simulation environment before any robots!
    env = BulletEnvWithGround()

    robot = B1Robot()

    set_trace()


if __name__ == "__main__":
    main()
