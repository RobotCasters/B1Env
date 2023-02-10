import numpy as np
import pybullet as p
from bullet_utils.env import BulletEnvWithGround

from B1Env.b1_sdk_api import B1SDKAPI
from B1Env.b1wrapper import B1Robot


def main():
    """Reads the robot state when the robot is hung in the air"""

    # Create a Pybullet simulation environment before any robots
    env = BulletEnvWithGround()

    # Remove debug sliders
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Create a robot instance. This adds the robot to the simulator as well.
    robot = B1Robot(useFixedBase=False)

    # Add the robot to the env to update the internal structure of the robot
    # at every simulation steps.
    env.add_robot(robot)

    # Create connection api to the real robot
    robot_api = B1SDKAPI()

    com_pos = np.array([0.0, 0.0, 1.0])

    # Read B1 state and show in simulator
    while True:
        _quat, _omega, _q, _dq = robot_api.get_state()

        # add base position and orientation info
        q = np.concatenate((com_pos, _quat, _q))
        dq = np.concatenate((np.zeros(3), _omega, _dq))

        # update pinocchio and pybullet state
        robot.update_pinocchio(q, dq)
        robot.reset_state(q, dq)

        # compute torque for gravity compensation of the legs
        _gravity = robot.pin_robot.gravity(q)
        tau = _gravity[6:]

        # set desired joint position and velocity to zero
        q_cmd = np.zeros(12)
        dq_cmd = np.zeros(12)

        # send joint command
        robot_api.send_joint_command(q_cmd, dq_cmd, tau)

        # Step the simulator.
        env.step(sleep=True)


if __name__ == "__main__":
    main()
