import sys

sys.path.append("/home/franka-ws/Documents/unitree_legged_sdk-3.8.3/lib/python/amd64/")
import numpy as np
import robot_interface as sdk


class B1SDKAPI:
    def __init__(self):
        LOWLEVEL = 0xFF
        self.udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)

        self.cmd = sdk.LowCmd()
        self.state = sdk.LowState()
        self.udp.InitCmdData(self.cmd)
        self.safe = sdk.Safety(sdk.LeggedType.B1)

        self._init_joint_command()

    def get_state(self):
        self.udp.Recv()
        self.udp.GetRecv(self.state)

        _quat = self.state.imu.quaternion
        omega = self.state.imu.gyroscope

        base_orientation = np.array([_quat[1], _quat[2], _quat[3], _quat[0]])
        q = np.array([motor.q for motor in self.state.motorState[:12]])
        dq = np.array([motor.dq for motor in self.state.motorState[:12]])

        return base_orientation, omega, q, dq

    def _init_joint_command(self):
        for motor_id in range(12):
            self.cmd.motorCmd[motor_id].Kp = 20.0
            self.cmd.motorCmd[motor_id].Kd = 2.0

    def send_joint_command(self, q_cmd, dq_cmd, tau_cmd):
        for motor_id in range(12):
            self.cmd.motorCmd[motor_id].q = q_cmd[motor_id]
            self.cmd.motorCmd[motor_id].dq = dq_cmd[motor_id]
            self.cmd.motorCmd[motor_id].tau = tau_cmd[motor_id]

        self.safe.PositionLimit(self.cmd)
        self.udp.SetSend(self.cmd)
        self.udp.Send()
