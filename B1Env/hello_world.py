import numpy as np
import pybullet as p
from bullet_utils.env import BulletEnvWithGround

from B1Env.b1wrapper import B1Robot
from B1Env.config import B1Config


def main():
    # ! Create a Pybullet simulation environment before any robots !
    env = BulletEnvWithGround()

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Create a robot instance. This adds the robot to the simulator as well.
    robot = B1Robot(useFixedBase=True)

    # Add the robot to the env to update the internal structure of the robot
    # ate every simulation steps.
    env.add_robot(robot)

    # Some control.
    tau = np.zeros(robot.nb_dof)

    # Reset the robot to some initial state.
    q0 = np.matrix(B1Config.initial_configuration).T
    dq0 = np.matrix(B1Config.initial_velocity).T
    robot.reset_state(q0, dq0)

    # Run the simulator for 2000 steps
    while True:
        robot.send_joint_command(tau)

        # Step the simulator.
        env.step(sleep=True)  # You can sleep here if you want to slow down the replay

    # Read the final state and forces after the stepping.
    q, dq = robot.get_state()
    active_eff, forces = robot.get_force()
    print("q", q)
    print("dq", dq)
    print("active eff", active_eff)
    print("forces", forces)


if __name__ == "__main__":
    main()
