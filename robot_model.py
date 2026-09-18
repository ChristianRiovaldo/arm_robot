import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

class RobotArm(rtb.DHRobot):
    def __init__(self):
        # Konversi presisi dari data tabel transformasi fisik
        d1 = 0.05675 + 0.070276  # = 0.127026 m (Tinggi Base ke Shoulder)
        a1 = 0.01392             # m (Offset horizontal Joint 1)
        
        a2 = 0.12                # m (Panjang Link 2 / Upper Arm)
        
        a3 = 0.08975             # m (Panjang Link 3 / Forearm)
        d3 = -0.010544           # m (Offset Sumbu Z Link 3)
        
        d4 = 0.0325              # m (Offset Tinggi Wrist Pitch)
        a4 = 0.0                 # m
        
        a5 = 0.105               # m (Panjang End-Effector / Gripper)
        d5 = -0.0075             # m (Offset Z End-Effector)

        links = [
            # Link 1: Base -> Link 1 & 2
            rtb.RevoluteDH(a=a1, alpha=np.pi/2, d=d1, offset=0, qlim=[-np.pi, np.pi]),
            
            # Link 2: Link 2 -> Link 3 (Shoulder)
            rtb.RevoluteDH(a=a2, alpha=0, d=0, offset=-np.pi/2, qlim=[-np.pi/2, np.pi/2]),
            
            # Link 3: Link 3 -> Link 4 (Elbow)
            rtb.RevoluteDH(a=a3, alpha=0, d=d3, offset=0, qlim=[-np.pi/2, np.pi/2]),
            
            # Link 4: Link 4 -> Link 5 (Wrist Pitch)
            rtb.RevoluteDH(a=a4, alpha=-np.pi/2, d=d4, offset=0, qlim=[-np.pi, np.pi]),
            
            # Link 5: Link 5 -> Link EE (Wrist Roll / Gripper)
            rtb.RevoluteDH(a=a5, alpha=0, d=d5, offset=0, qlim=[-np.pi, np.pi]),
        ]
        
        super().__init__(links, name="RobotArm_RealPhysical")

    def forward_kinematics(self, q_radians):
        T = self.fkine(q_radians)
        return T.t

    def inverse_kinematics_3d(self, target_x, target_y, target_z, q_initial=None):
        if q_initial is None:
            q_initial = np.zeros(5)

        target_pose = SE3(target_x, target_y, target_z)

        # Gunakan mask 3D (X, Y, Z) saja
        sol = self.ikine_LM(target_pose, q0=q_initial, mask=[1, 1, 1, 0, 0, 0])

        if sol.success:
            q_rad = sol.q
            q_deg = np.degrees(q_rad)

            # Map sudut ke Servo 0 - 180 derajat
            servo_deg = np.zeros(5)
            servo_deg[0] = np.clip(q_deg[0] + 90, 0, 180)  # Base
            servo_deg[1] = np.clip(q_deg[1] + 90, 0, 180)  # Shoulder
            servo_deg[2] = np.clip(q_deg[2] + 90, 0, 180)  # Elbow
            servo_deg[3] = np.clip(q_deg[3] + 90, 0, 180)  # Wrist Pitch
            servo_deg[4] = np.clip(q_deg[4] + 90, 0, 180)  # Wrist Roll

            ee_pos = tuple(self.forward_kinematics(q_rad))
            return True, q_rad, servo_deg, ee_pos

        return False, q_initial, None, None


if __name__ == "__main__":
    arm = RobotArm()
    print("=== MODEL ROBOT DENGAN PARAMETER ASLI CAD ===")
    print(arm)

    # Uji IK dengan target jangkauan akurat (X=0.05m, Y=0.20m, Z=0.15m)
    tx, ty, tz = 0.05, 0.20, 0.15
    print(f"\n[TEST IK 3D] Target Koordinat: X={tx}m, Y={ty}m, Z={tz}m")

    success, q_rad, servos, ee_pos = arm.inverse_kinematics_3d(tx, ty, tz)

    if success:
        print("IK Berhasil Ditemukan!")
        print(f"Sudut Joint (Radian) : {np.round(q_rad, 3)}")
        print(f"Sudut Servo (0-180°) : {np.round(servos, 1)}")
        print(f"Posisi Aktual FK (m) : X={ee_pos[0]:.3f}, Y={ee_pos[1]:.3f}, Z={ee_pos[2]:.3f}")
    else:
        print("Target di luar jangkauan robot!")