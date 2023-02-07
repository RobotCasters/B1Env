from math import pi

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import zero

from B1Env import getDataPath


class B1Config(object):
    robot_family = "unitree"
    robot_name = "b1"

    # ========== BEGIN MAYBE UNUSED VARIABLES ==========

    # PID gains
    kp = 5.0
    kd = 0.1
    ki = 0.0

    # The Kt constant of the motor [Nm/A]: tau = I * Kt.
    motor_torque_constant = 0.025

    # Control time period.
    control_period = 0.001
    dt = control_period

    # MaxCurrent = 12 # Ampers
    max_current = 2

    # Maximum torques.
    max_torque = motor_torque_constant * max_current

    # Maximum control one can send, here the control is the current.
    max_control = max_current

    # ctrl_manager_current_to_control_gain I am not sure what it does so 1.0.
    ctrl_manager_current_to_control_gain = 1.0

    max_qref = pi
    base_p_com = [0.0, 0.0, -0.02]

    # =========== END MAYBE UNUSED VARIABLES ===========

    base_link_name = "trunk"

    meshes_path = getDataPath()
    # dgm_yaml_path = resources.dgm_yaml_path
    urdf_path = meshes_path + "/robots/b1_raw.urdf"
    # ctrl_path = resources.imp_ctrl_yaml_path

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045

    # The motor gear ratio.
    motor_gear_ratio = 9.0

    # pinocchio model.
    pin_robot_wrapper = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path, pin.JointModelFreeFlyer()
    )
    pin_robot_wrapper.model.rotorInertia[6:] = motor_inertia
    pin_robot_wrapper.model.rotorGearRatio[6:] = motor_gear_ratio
    pin_robot = pin_robot_wrapper

    robot_model = pin_robot_wrapper.model
    mass = np.sum([i.mass for i in robot_model.inertias])
    base_name = robot_model.frames[2].name

    # End effectors informations
    shoulder_ids = []
    end_eff_ids = []
    shoulder_names = []
    end_effector_names = []
    for leg in ["FL", "FR", "HL", "HR"]:
        shoulder_ids.append(robot_model.getFrameId(leg + "_hip_joint"))
        shoulder_names.append(leg + "_hip_joint")
        end_eff_ids.append(robot_model.getFrameId(leg + "_foot"))
        end_effector_names.append(leg + "_foot")

    nb_ee = len(end_effector_names)
    hl_index = robot_model.getFrameId("HL_ankle_fixed")
    hr_index = robot_model.getFrameId("HR_ankle_fixed")
    fl_index = robot_model.getFrameId("FL_ankle_fixed")
    fr_index = robot_model.getFrameId("FR_ankle_fixed")

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv - 6

    joint_names = [
        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_knee_joint",
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_knee_joint",
        "HL_hip_joint",
        "HL_thigh_joint",
        "HL_knee_joint",
        "HR_hip_joint",
        "HR_thigh_joint",
        "HR_knee_joint",
    ]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(12))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] + 4 * [0.0, 0.9, -1.8]
    initial_velocity = int((8 + 4 + 6) / 2) * [0.0, 0.0]

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)

    rot_base_to_imu = np.identity(3)
    r_base_to_imu = np.zeros(3)

    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path, pin.JointModelFreeFlyer()
        )
        robot.model.rotorInertia[6:] = cls.motor_inertia
        robot.model.rotorGearRatio[6:] = cls.motor_gear_ratio
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names
