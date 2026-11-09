import argparse
import importlib
import sys

REQUIRED = [
    ("cv2", "opencv-python"),
    ("numpy", "numpy"),
    ("PIL", "pillow"),
    ("mediapipe", "mediapipe"),
]

OPTIONAL = [
    ("roboticstoolbox", "roboticstoolbox-python", "mode Pointing (IK) nonaktif"),
    ("spatialmath", "spatialmath-python", "mode Pointing (IK) nonaktif"),
    ("serial", "pyserial", "tombol 'Kirim ke Servo' tidak bisa dipakai"),
]

def _has_module(name):
    try:
        importlib.import_module(name)
        return True
    except ImportError:
        return False

def check_dependencies():
    missing = [(m, p) for m, p in REQUIRED if not _has_module(m)]
    if missing:
        print("ERROR: modul wajib belum terpasang:")
        for m, p in missing:
            print(f"  - {m:<16} -> pip install {p}")
        return False

    import mediapipe as mp
    if not hasattr(mp, "solutions"):
        print("ERROR: versi mediapipe yang terpasang tidak punya mp.solutions "
              "(dipakai camera.py).")
        print("       Pasang versi yang masih menyertakannya, mis.:  "
              "pip install mediapipe==0.10.14")
        return False

    for m, p, effect in OPTIONAL:
        if not _has_module(m):
            print(f"PERINGATAN: {m} belum terpasang -> {effect}  (pip install {p})")
    return True

def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("pyserial belum terpasang:  pip install pyserial")
        return 1
    ports = list(lp.comports())
    if not ports:
        print("Tidak ada port serial yang terdeteksi. Cek kabel USB Arduino.")
        return 0
    print("Port serial yang terdeteksi:")
    for p in sorted(ports, key=lambda x: x.device):
        print(f"  {p.device:<14} {p.description}")
    return 0

def parse_args(argv):
    ap = argparse.ArgumentParser(description="Sistem kontrol lengan robot: kamera + MediaPipe + IK + Arduino/PCA9685.")
    ap.add_argument("--port", help="port serial Arduino (/dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, help="baud rate serial (harus sama dengan servo_control.ino)")
    ap.add_argument("--camera", type=int, help="index kamera untuk cv2.VideoCapture")
    ap.add_argument("--width", type=int, help="lebar capture kamera")
    ap.add_argument("--height", type=int, help="tinggi capture kamera")
    ap.add_argument("--list-ports", action="store_true", help="tampilkan port serial yang terdeteksi lalu keluar")
    return ap.parse_args(argv)

def main(argv=None):
    args = parse_args(argv)

    if args.list_ports:
        return list_ports()

    if not check_dependencies():
        return 1

    try:
        import gui_app
    except ImportError as e:
        print(f"ERROR: gagal memuat gui_app.py ({e}).")
        print("Pastikan camera.py, pointing_ik.py, robot_model.py, servo_config.py "
              "dan serial_com.py berada di folder yang sama dengan main.py.")
        return 1

    if args.port is not None:
        gui_app.SERIAL_PORT = args.port
    if args.baud is not None:
        gui_app.SERIAL_BAUD = args.baud
    if args.camera is not None:
        gui_app.CAMERA_INDEX = args.camera
    if args.width is not None:
        gui_app.CAM_CAPTURE_W = args.width
    if args.height is not None:
        gui_app.CAM_CAPTURE_H = args.height

    print("=== ARM ROBOT CONTROLLER ===")
    print(f"Kamera : index {gui_app.CAMERA_INDEX} ({gui_app.CAM_CAPTURE_W}x{gui_app.CAM_CAPTURE_H})")
    print(f"Serial : {gui_app.SERIAL_PORT} @ {gui_app.SERIAL_BAUD} (aktif setelah tombol 'Kirim ke Servo' ditekan)")

    if _has_module("serial"):
        from serial.tools import list_ports as lp
        found = [p.device for p in lp.comports()]
        if gui_app.SERIAL_PORT not in found:
            shown = ", ".join(found) if found else "tidak ada"
            print(f"CATATAN: {gui_app.SERIAL_PORT} tidak ada di daftar port terdeteksi ({shown}). "
                  f"Pakai --port untuk memilih port lain.")

    app = gui_app.GuiApp()
    app.protocol("WM_DELETE_WINDOW", app.destroy)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        app.destroy()
    return 0

if __name__ == "__main__":
    sys.exit(main())