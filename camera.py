import cv2
import mediapipe as mp

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    import numpy as np
    POINTING_LIBS_AVAILABLE = True
except ImportError:
    POINTING_LIBS_AVAILABLE = False

COLOR_WORKSPACE_IDLE = (52, 45, 42)
COLOR_WORKSPACE_ACTIVE = (220, 157, 0)
COLOR_LABEL_TEXT = (255, 255, 255)
COLOR_ACCENT = (48, 100, 242)

LEFT_HAND_MODE_MAP = {
    (0, 1, 0, 0, 0): "DOF1",
    (0, 1, 1, 0, 0): "DOF2",
    (0, 1, 1, 1, 0): "DOF3",
    (0, 1, 1, 1, 1): "DOF4",
    (1, 1, 1, 1, 1): "DOF5",
    (1, 0, 0, 0, 0): "GRIPPER",
}
DOF_ORDER = ["DOF1", "DOF2", "DOF3", "DOF4", "DOF5", "GRIPPER"]

class Camera:
    def __init__(self, max_num_hands=2,
                 min_detection_confidence=0.7,
                 min_tracking_confidence=0.7):

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )

        self.pointing_enabled = POINTING_LIBS_AVAILABLE
        if self.pointing_enabled:
            self.robot = rtb.models.DH.Puma560()
            self.q_current = self.robot.qz
        else:
            self.robot = None
            self.q_current = None

    def get_finger_states(self, hand_landmarks, hand_label):
        fingers = []
        if hand_label == "Right":
            thumb_open = (
                hand_landmarks.landmark[4].x <
                hand_landmarks.landmark[3].x
            )
        else:
            thumb_open = (
                hand_landmarks.landmark[4].x >
                hand_landmarks.landmark[3].x
            )

        fingers.append(1 if thumb_open else 0)
        fingers.append(1 if hand_landmarks.landmark[8].y < hand_landmarks.landmark[6].y else 0)
        fingers.append(1 if hand_landmarks.landmark[12].y < hand_landmarks.landmark[10].y else 0)
        fingers.append(1 if hand_landmarks.landmark[16].y < hand_landmarks.landmark[14].y else 0)
        fingers.append(1 if hand_landmarks.landmark[20].y < hand_landmarks.landmark[18].y else 0)

        return fingers

    def get_gripper_state(self, hand_landmarks):
        thumb_tip = hand_landmarks.landmark[4]
        index_tip = hand_landmarks.landmark[8]
        wrist = hand_landmarks.landmark[0]
        middle_mcp = hand_landmarks.landmark[9]

        pinch_distance = (
            (thumb_tip.x - index_tip.x) ** 2 +
            (thumb_tip.y - index_tip.y) ** 2
        ) ** 0.5

        palm_size = (
            (wrist.x - middle_mcp.x) ** 2 +
            (wrist.y - middle_mcp.y) ** 2
        ) ** 0.5

        if palm_size < 1e-6:
            palm_size = 1e-6

        normalized_distance = pinch_distance / palm_size
        gripper_state = "CLOSE" if normalized_distance < 0.5 else "OPEN"
        angle = 0 if gripper_state == "CLOSE" else 180

        return gripper_state, angle, normalized_distance

    def process(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        h, w, _ = frame.shape

        margin_x = int(w * 0.01)
        top_y = int(h * 0.03)
        bottom_y = int(h * 0.99)
        mid_gap = int(w * 0.01)

        workspaces = {
            "left": {
                "x1": margin_x, "y1": top_y,
                "x2": int(w / 2) - mid_gap, "y2": bottom_y,
                "color": COLOR_WORKSPACE_IDLE, "label": "WORKSPACE KIRI"
            },
            "right": {
                "x1": int(w / 2) + mid_gap, "y1": top_y,
                "x2": w - margin_x, "y2": bottom_y,
                "color": COLOR_WORKSPACE_IDLE, "label": "WORKSPACE KANAN"
            },
        }

        detected = {"left": None, "right": None}

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(
                    results.multi_hand_landmarks,
                    results.multi_handedness):

                hand_label = handedness.classification[0].label

                wrist = hand_landmarks.landmark[0]
                wrist_x = int(wrist.x * w)
                wrist_y = int(wrist.y * h)

                target_key = None
                for key, ws in workspaces.items():
                    if (ws["x1"] <= wrist_x <= ws["x2"] and
                            ws["y1"] <= wrist_y <= ws["y2"]):
                        target_key = key
                        break

                if target_key is None or detected[target_key] is not None:
                    continue

                fingers = self.get_finger_states(hand_landmarks, hand_label)
                gripper_state, gripper_angle, normalized_distance = self.get_gripper_state(hand_landmarks)

                detected[target_key] = {
                    "fingers": fingers,
                    "gripper_state": gripper_state,
                    "gripper_angle": gripper_angle,
                    "normalized_distance": normalized_distance,
                    "hand_label": hand_label,
                }

                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )
                cv2.circle(frame, (wrist_x, wrist_y), 10, COLOR_ACCENT, -1, cv2.LINE_AA)

        for key, ws in workspaces.items():
            ws["color"] = COLOR_WORKSPACE_ACTIVE if detected[key] is not None else COLOR_WORKSPACE_IDLE

            label_h = 26
            cv2.rectangle(frame, (ws["x1"], ws["y1"]),
                           (ws["x2"], ws["y2"]), ws["color"], 3)
            cv2.rectangle(frame,
                           (ws["x1"], ws["y1"]),
                           (ws["x2"], ws["y1"] + label_h),
                           ws["color"], -1)
            cv2.putText(frame, ws["label"],
                        (ws["x1"] + 8, ws["y1"] + label_h - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_LABEL_TEXT, 2,
                        cv2.LINE_AA)

        return detected

    @staticmethod
    def get_robot_command(detected):
        left = detected.get("left")
        right = detected.get("right")
        result = {
            "active": False,
            "target": None,
            "angle": None,
            "reason": "Menunggu 2 tangan",
        }

        if left is None or right is None:
            missing = []
            if left is None:
                missing.append("KIRI")
            if right is None:
                missing.append("KANAN")
            result["reason"] = "Belum terdeteksi: " + ", ".join(missing)
            return result

        mode = LEFT_HAND_MODE_MAP.get(tuple(left["fingers"]))

        if mode is None:
            result["reason"] = "Pola jari kiri tidak dikenali"
            return result

        result["target"] = mode

        if mode == "GRIPPER":
            result["active"] = True
            result["angle"] = right["gripper_angle"]
            result["reason"] = "OK"
            return result

        right_fingers = right["fingers"]

        if right_fingers == [0, 0, 0, 0, 0]:
            result["active"] = True
            result["angle"] = 0
            result["reason"] = "OK"
        elif right_fingers == [1, 1, 1, 1, 1]:
            result["active"] = True
            result["angle"] = 180
            result["reason"] = "OK"
        else:
            result["reason"] = (
                "Tangan kanan harus MENGGENGGAM (0°) atau TERBUKA (180°)"
            )

        return result

    def process_pointing(self, frame):
        result = {
            "detected": False,
            "target_xyz": None,
            "ee_pos": None,
            "joint_deg": None,
            "servo": None,
            "ik_success": False,
            "reason": "Menunggu tangan",
        }

        if not self.pointing_enabled:
            result["reason"] = "roboticstoolbox / spatialmath belum terinstall"
            return result

        h, w, _ = frame.shape

        margin_x = int(w * 0.01)
        ws_x1, ws_y1 = margin_x, int(h * 0.03)
        ws_x2, ws_y2 = w - margin_x, int(h * 0.99)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]

            self.mp_draw.draw_landmarks(
                frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
            )

            wrist = hand_landmarks.landmark[0]
            wrist_x = int(wrist.x * w)
            wrist_y = int(wrist.y * h)
            cv2.circle(frame, (wrist_x, wrist_y), 10, COLOR_ACCENT, -1, cv2.LINE_AA)

            result["detected"] = True
            result["reason"] = "OK"

            X = (wrist.x - 0.5) * 0.60
            Y = (0.5 - wrist.y) * 0.60
            Z = 0.40
            result["target_xyz"] = (X, Y, Z)

            target_pose = SE3(X, Y, Z)
            sol = self.robot.ikine_LM(target_pose, q0=self.q_current)

            if sol.success:
                self.q_current = sol.q
                joint_deg = np.degrees(sol.q)
                result["joint_deg"] = joint_deg

                servo = np.zeros(6)
                servo[0] = np.clip(joint_deg[0] + 160, 0, 180)
                servo[1] = np.clip(joint_deg[1] + 110, 0, 180)
                servo[2] = np.clip(joint_deg[2] + 135, 0, 180)
                servo[3] = np.clip(joint_deg[3] + 266, 0, 180)
                servo[4] = np.clip(joint_deg[4] + 100, 0, 180)
                servo[5] = np.clip(joint_deg[5] + 266, 0, 180)
                result["servo"] = servo

                T = self.robot.fkine(sol.q)
                result["ee_pos"] = tuple(T.t)
                result["ik_success"] = True
            else:
                result["reason"] = "IK gagal (target di luar jangkauan)"
        else:
            result["reason"] = "Tangan tidak terdeteksi"

        color = COLOR_WORKSPACE_ACTIVE if result["detected"] else COLOR_WORKSPACE_IDLE
        label_h = 26
        cv2.rectangle(frame, (ws_x1, ws_y1), (ws_x2, ws_y2), color, 3)
        cv2.rectangle(frame, (ws_x1, ws_y1), (ws_x2, ws_y1 + label_h), color, -1)
        cv2.putText(frame, "WORKSPACE POINTING",
                    (ws_x1 + 8, ws_y1 + label_h - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_LABEL_TEXT, 2,
                    cv2.LINE_AA)

        return result

    def reset_pointing(self):
        if self.pointing_enabled:
            self.q_current = self.robot.qz

    def close(self):
        self.hands.close()