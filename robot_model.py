import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import numpy as np
import roboticstoolbox as rtb
import servo_config as cfg
from spatialmath import SE3

DH_PARAMS = [
    (0.000000,  np.pi / 2,  0.127026,  np.pi / 2),   # Joint 1 : Base yaw
    (0.120000,  0.0,        0.000000,  0.0),         # Joint 2 : Shoulder
    (0.089750, -np.pi / 2,  0.003376,  0.0),         # Joint 3 : Elbow
    (0.014200, -np.pi / 2,  0.032500,  0.0),         # Joint 4 : Wrist
    (0.000000,  0.0,        0.000000,  0.0),         # Joint 5 : Wrist 2
]

TOOL = SE3(-0.105, 0.0, -0.0075) * SE3.Rz(np.pi)
JOINT_LIMITS_RAD = [tuple(np.radians(cfg.joint_limits_deg(i))) for i in range(cfg.NUM_DOF)]

class RobotModel(rtb.DHRobot):
    def __init__(self):
        links = [
            rtb.RevoluteDH(a=a, alpha=alpha, d=d, offset=offset, qlim=list(JOINT_LIMITS_RAD[i]))
            for i, (a, alpha, d, offset) in enumerate(DH_PARAMS)
        ]
        super().__init__(links, name="RobotModel_Tabel", tool=TOOL)

    def forward_kinematics(self, q_radians):
        T = self.fkine(q_radians)
        return T.t

    def home_q(self):
        q = np.zeros(self.n)
        for i, (lo, hi) in enumerate(JOINT_LIMITS_RAD):
            q[i] = min(max(0.0, lo), hi)
        return q

    def q_to_servo(self, q_rad):
        q_deg = np.degrees(q_rad)
        return np.array([cfg.q_to_servo_deg(i, q_deg[i]) for i in range(self.n)], dtype=float)

    def inverse_kinematics_3d(self, target_x, target_y, target_z, q_initial=None):
        if q_initial is None:
            q_initial = self.home_q()

        target_pose = SE3(target_x, target_y, target_z)

        for q0 in (q_initial, self.home_q()):
            sol = self.ikine_LM(target_pose, q0=q0, mask=[1, 1, 1, 0, 0, 0],
                                ilimit=50, slimit=1, joint_limits=True)
            if sol.success:
                q_rad = np.asarray(sol.q, dtype=float)
                servo_deg = self.q_to_servo(q_rad)
                ee_pos = tuple(self.forward_kinematics(q_rad))
                return True, q_rad, servo_deg, ee_pos

        return False, q_initial, None, None

if __name__ == "__main__":
    arm = RobotModel()
    print("=== MODEL ROBOT (DH diturunkan dari tabel transformasi) ===")
    print(arm)

    q0 = np.zeros(arm.n)
    p0 = arm.forward_kinematics(q0)
    print(f"\nEE saat semua joint = 0 (pose nol tabel): "
          f"X={p0[0]:.4f}, Y={p0[1]:.4f}, Z={p0[2]:.4f} m")

    tx, ty, tz = 0.05, 0.15, 0.15
    print(f"\n[TEST IK 3D] Target: X={tx}m, Y={ty}m, Z={tz}m")
    success, q_rad, servos, ee_pos = arm.inverse_kinematics_3d(tx, ty, tz)
    if success:
        print("IK berhasil!")
        print(f"Sudut joint (rad)   : {np.round(q_rad, 3)}")
        print(f"Sudut servo (0-180) : {np.round(servos, 1)}")
        print(f"Posisi FK (m)       : X={ee_pos[0]:.3f}, Y={ee_pos[1]:.3f}, Z={ee_pos[2]:.3f}")
    else:
        print("Target di luar jangkauan robot!")