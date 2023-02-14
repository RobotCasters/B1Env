import numpy as np
import pinocchio as pin
import pybullet as p
from bullet_utils.env import BulletEnvWithGround

from B1Env.b1_sdk_api import B1SDKAPI
from B1Env.b1wrapper import B1Robot
from B1Env.config import B1Config


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

    # Retrieve nominal configuration
    q0 = np.matrix(B1Config.initial_configuration).T
    dq0 = np.matrix(B1Config.initial_velocity).T
    com_pos = np.array([0.0, 0.0, 1.0])

    # Leg ground reaction force
    leg_gc_force = -np.array([[0.0], [0.0], [B1Config.mass * 9.81 / 4]])

    kp = 10.0
    kd = 1.0

    count = 0

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
        tau = (
            _gravity[6:]
            + kp * (np.asarray(q0)[7:, 0] - q[7:])
            + kd * (np.asarray(dq0)[6:, 0] - dq[6:])
        )

        for idx in [robot.fr_index, robot.fl_index, robot.hr_index, robot.hl_index]:
            _J = robot.pin_robot.getFrameJacobian(
                idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )[:3, 6:]
            _tau_gc = _J.T @ leg_gc_force
            tau = tau + _tau_gc[:, 0]

        # set desired joint position and velocity to zero
        q_cmd = np.zeros(12)
        dq_cmd = np.zeros(12)

        # send joint command
        robot_api.send_joint_command(q_cmd, dq_cmd, tau)

        # Step the simulator.
        env.step(sleep=False)
        count += 1
        print(count)

        if count >= 20000:
            kp = 60.0
            kd = 6.0


if __name__ == "__main__":
    main()
