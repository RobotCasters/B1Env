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

    def get_state(self, map_to_pin=True):
        self.udp.Recv()
        self.udp.GetRecv(self.state)

        _quat = self.state.imu.quaternion
        omega = self.state.imu.gyroscope

        base_orientation = np.array([_quat[1], _quat[2], _quat[3], _quat[0]])
        _q = np.array([motor.q for motor in self.state.motorState[:12]])
        _dq = np.array([motor.dq for motor in self.state.motorState[:12]])

        if map_to_pin:
            # reorder the joints from [FL, FR, HL, HR] to [FR, FL, HR, HL]
            q = self.map_joint_idx(_q)
            dq = self.map_joint_idx(_dq)
        else:
            q = _q
            dq = _dq

        return base_orientation, omega, q, dq

    def _init_joint_command(self):
        for motor_id in range(12):
            self.cmd.motorCmd[motor_id].Kp = 0.0
            self.cmd.motorCmd[motor_id].Kd = 0.0

    def send_joint_command(self, _q_cmd, _dq_cmd, _tau_cmd, map_from_pin=True):
        if map_from_pin:
            q_cmd = self.map_joint_idx(_q_cmd)
            dq_cmd = self.map_joint_idx(_dq_cmd)
            tau_cmd = self.map_joint_idx(_tau_cmd)
        else:
            q_cmd, dq_cmd, tau_cmd = _q_cmd, _dq_cmd, _tau_cmd

        for motor_id in range(12):
            self.cmd.motorCmd[motor_id].q = q_cmd[motor_id]
            self.cmd.motorCmd[motor_id].dq = dq_cmd[motor_id]
            self.cmd.motorCmd[motor_id].tau = tau_cmd[motor_id]

        self.safe.PositionLimit(self.cmd)
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def map_joint_idx(self, pin_array):
        return np.array(
            [
                pin_array[3],
                pin_array[4],
                pin_array[5],
                pin_array[0],
                pin_array[1],
                pin_array[2],
                pin_array[9],
                pin_array[10],
                pin_array[11],
                pin_array[6],
                pin_array[7],
                pin_array[8],
            ]
        )
