import numpy as np

import servo_config as cfg

try:
    from robot_model import RobotModel
    POINTING_LIBS_AVAILABLE = True
except ImportError as e:
    print(f"[pointing_ik] Mode pointing nonaktif, gagal import robot_model: {e}")
    POINTING_LIBS_AVAILABLE = False

WORKSPACE_CENTER = (0.00, 0.105, 0.27)  # X, Y, Z tengah kotak workspace
WORKSPACE_HALF = (0.10, 0.055, 0.04)

CAM_EDGE_MARGIN = 0.10
CAM_DEPTH_RANGE = 0.20

GRIPPER = cfg.GRIPPER_INDEX
GRIPPER_CLOSED_DEG = cfg.binary_target(GRIPPER, is_open=False)
GRIPPER_OPEN_DEG = cfg.binary_target(GRIPPER, is_open=True)

class PointingIKHandler:
    def __init__(self):
        self.enabled = POINTING_LIBS_AVAILABLE
        if self.enabled:
            self.robot = RobotModel()
            self.q_current = self.robot.home_q()
        else:
            self.robot = None
            self.q_current = None

        self.last_gripper_deg = GRIPPER_CLOSED_DEG

    @staticmethod
    def camera_to_target(x_norm, y_norm, z_rel):
        span = 0.5 - CAM_EDGE_MARGIN
        u = float(np.clip((x_norm - 0.5) / span, -1.0, 1.0)) 
        v = float(np.clip((0.5 - y_norm) / span, -1.0, 1.0))
        w = float(np.clip(-z_rel / CAM_DEPTH_RANGE, -1.0, 1.0)) 

        cx, cy, cz = WORKSPACE_CENTER
        hx, hy, hz = WORKSPACE_HALF
        return cx + u * hx, cy + v * hy, cz + w * hz

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
            gripper_deg = GRIPPER_CLOSED_DEG
        elif other_4_fingers == [1, 1, 1, 1]:
            gripper_deg = GRIPPER_OPEN_DEG
        else:
            gripper_deg = self.last_gripper_deg

        self.last_gripper_deg = gripper_deg

        X, Y, Z = self.camera_to_target(index_tip_landmark.x, index_tip_landmark.y, index_tip_landmark.z)
        result["target_xyz"] = (X, Y, Z)

        success, q_rad, servo_deg, ee_pos = self.robot.inverse_kinematics_3d(X, Y, Z, q_initial=self.q_current)

        if success:
            self.q_current = q_rad
            result["joint_deg"] = np.round(np.degrees(q_rad)).astype(int).tolist()

            full_servos = [cfg.clamp_servo(i, servo_deg[i]) for i in range(cfg.NUM_DOF)]
            full_servos.append(cfg.clamp_servo(GRIPPER, gripper_deg))

            result["servo"] = full_servos
            result["ee_pos"] = ee_pos
            result["ik_success"] = True
        else:
            result["reason"] = "IK gagal (target telunjuk di luar jangkauan)"

        return result

    def reset(self):
        if self.enabled:
            self.q_current = self.robot.home_q()
            self.last_gripper_deg = GRIPPER_CLOSED_DEG


if __name__ == "__main__":
    from types import SimpleNamespace

    handler = PointingIKHandler()
    if not handler.enabled:
        raise SystemExit("robot_model tidak bisa di-import.")

    ok = total = out_of_limit = 0
    worst_err = 0.0
    for x in np.linspace(0.0, 1.0, 11):
        ys = np.linspace(0.0, 1.0, 11)
        ys = ys if int(round(x * 10)) % 2 == 0 else ys[::-1]
        for y in ys:
            for z in (-0.25, -0.10, 0.0, 0.10):
                for fingers in ([1, 1, 0, 0, 0], [1, 1, 1, 1, 1], [0, 1, 0, 0, 0]):
                    tip = SimpleNamespace(x=float(x), y=float(y), z=float(z))
                    res = handler.compute_pointing_target(tip, fingers)
                    total += 1
                    if res["ik_success"]:
                        ok += 1
                        err = np.linalg.norm(np.array(res["ee_pos"]) - np.array(res["target_xyz"]))
                        worst_err = max(worst_err, err)
                        for i, s in enumerate(res["servo"]):
                            if s < cfg.SERVO_MIN_DEG[i] or s > cfg.SERVO_MAX_DEG[i]:
                                out_of_limit += 1

    print(f"IK sukses: {ok}/{total} ({100 * ok / total:.1f}%)")
    print(f"Error posisi EE terburuk: {worst_err * 1000:.3f} mm")
    print(f"Sudut servo di luar batas: {out_of_limit}")