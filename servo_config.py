SERVO_MIN_DEG = [0,   90,  45,   0,   100, 90]
SERVO_MAX_DEG = [180, 180, 150, 180, 180, 180]

NUM_DOF = 5
GRIPPER_INDEX = 5

SERVO_ZERO_DEG = [90.0, 90.0, 90.0, 90.0, 90.0]
SERVO_DIR = [1, 1, 1, 1, 1]

def clamp_servo(index, deg):
    """Batasi sudut servo ke [MIN, MAX] miliknya. Return int."""
    return int(round(min(max(deg, SERVO_MIN_DEG[index]), SERVO_MAX_DEG[index])))

def q_to_servo_deg(index, q_deg):
    """Sudut joint model (derajat) -> sudut servo (derajat, sudah dibatasi)."""
    return clamp_servo(index, SERVO_ZERO_DEG[index] + SERVO_DIR[index] * q_deg)

def joint_limits_deg(index):
    """Batas sudut joint model (derajat) yang setara dengan batas servo."""
    a = (SERVO_MIN_DEG[index] - SERVO_ZERO_DEG[index]) * SERVO_DIR[index]
    b = (SERVO_MAX_DEG[index] - SERVO_ZERO_DEG[index]) * SERVO_DIR[index]
    return (min(a, b), max(a, b))

def binary_target(index, is_open):
    """Perintah 'terbuka' -> batas maksimum servo, 'menggenggam' -> batas minimum."""
    return SERVO_MAX_DEG[index] if is_open else SERVO_MIN_DEG[index]