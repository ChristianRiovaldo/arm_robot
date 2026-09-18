import numpy as np
try:
    from robot_model import RobotArm
    POINTING_LIBS_AVAILABLE = True
except ImportError:
    POINTING_LIBS_AVAILABLE = False

class PointingIKHandler:
    def __init__(self):
        self.enabled = POINTING_LIBS_AVAILABLE
        if self.enabled:
            self.robot = RobotArm()
            self.q_current = np.zeros(5)
        else:
            self.robot = None
            self.q_current = None

        self.last_gripper_deg = 0

    def compute_pointing_target(self, index_tip_landmark, fingers):
        result = {
            "target_xyz": None,
            "ee_pos": None,
            "joint_deg": None,
            "servo": None,
            "ik_success": False,
            "reason": "OK",
        }

        if not self.enabled:
            result["reason"] = "robot_model / spatialmath belum terinstall"
            return result

        other_4_fingers = [fingers[0], fingers[2], fingers[3], fingers[4]]

        if other_4_fingers == [0, 0, 0, 0]:
            gripper_deg = 0
        elif other_4_fingers == [1, 1, 1, 1]:
            gripper_deg = 180
        else:
            gripper_deg = self.last_gripper_deg

        self.last_gripper_deg = gripper_deg

        X = (index_tip_landmark.x - 0.5) * 0.50
        Y = 0.10 + (0.5 - index_tip_landmark.y) * 0.50
        Z = float(np.clip(0.18 - (index_tip_landmark.z * 0.50), 0.05, 0.30))

        result["target_xyz"] = (X, Y, Z)

        success, q_rad, servo_deg, ee_pos = self.robot.inverse_kinematics_3d(
            X, Y, Z, q_initial=self.q_current
        )

        if success:
            self.q_current = q_rad
            result["joint_deg"] = np.round(np.degrees(q_rad)).astype(int).tolist()

            full_servos = np.zeros(6, dtype=int)
            full_servos[0:5] = np.round(servo_deg).astype(int)
            full_servos[5] = int(gripper_deg)

            result["servo"] = full_servos.tolist()
            result["ee_pos"] = ee_pos
            result["ik_success"] = True
        else:
            result["reason"] = "IK gagal (target telunjuk di luar jangkauan)"

        return result
    
    def reset(self):
        if self.enabled:
            self.q_current = np.zeros(5)
            self.last_gripper_deg = 0