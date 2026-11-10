import argparse
import importlib
import sys
import time

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

MAX_CAMERA_INDEX = 10

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

def _quiet_opencv():
    import cv2
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except AttributeError:
        pass

def camera_backends():
    import cv2
    result = [("default", cv2.CAP_ANY)]
    if sys.platform.startswith("linux"):
        result.append(("V4L2", cv2.CAP_V4L2))
    elif sys.platform.startswith("win"):
        result.append(("DSHOW", cv2.CAP_DSHOW))
    return result

def probe_camera(index, backend):
    import cv2
    cap = cv2.VideoCapture(index, backend)
    opened = bool(cap.isOpened())
    frame_ok, size = False, None
    if opened:
        for _ in range(10):
            ok, frame = cap.read()
            if ok and frame is not None:
                frame_ok = True
                size = (frame.shape[1], frame.shape[0])
                break
            time.sleep(0.05)
    cap.release()
    return opened, frame_ok, size

def probe_index(index):
    for name, backend in camera_backends():
        opened, frame_ok, size = probe_camera(index, backend)
        if opened:
            return {"index": index, "backend_name": name, "backend": backend,
                    "frame_ok": frame_ok, "size": size}
    return None

def scan_cameras(skip=None):
    found = []
    for i in range(MAX_CAMERA_INDEX):
        if i == skip:
            continue
        hit = probe_index(i)
        if hit:
            found.append(hit)
    return found

def camera_diagnostics():
    print("Kemungkinan penyebab:")
    if sys.platform.startswith("linux"):
        import glob
        import os
        devs = sorted(glob.glob("/dev/video*"))
        if not devs:
            print("  - Tidak ada /dev/video* : kamera tidak terdeteksi sistem.")
            print("    Cek kabel/port USB, lalu `lsusb` dan `v4l2-ctl --list-devices`.")
            print("    Di WSL / mesin virtual / container, webcam harus di-passthrough dulu.")
            return
        denied = [d for d in devs if not os.access(d, os.R_OK | os.W_OK)]
        print("  Perangkat kamera ditemukan: " + ", ".join(devs))
        if denied:
            try:
                import grp
                groups = sorted({grp.getgrgid(g).gr_name for g in os.getgroups()})
            except (ImportError, KeyError):
                groups = []
            print("  - TIDAK ADA IZIN akses ke: " + ", ".join(denied))
            print(f"    Grup kamu sekarang: {', '.join(groups) or '?'}")
            print("    Perbaikan: sudo usermod -aG video $USER   lalu logout/login (atau reboot).")
        print("  - Kamera sedang dipakai program lain (browser, Zoom, Cheese, proses Python lama).")
        print("    Cek dengan: sudo fuser -v /dev/video0   lalu tutup program tersebut.")
        print("  - Index salah: satu kamera sering punya 2 node (video0 = gambar, video1 = metadata).")
    else:
        print("  - Index salah, atau kamera sedang dipakai program lain.")
        print("  - Izin kamera di pengaturan privasi sistem belum diberikan.")


def list_cameras():
    _quiet_opencv()
    print("Memeriksa kamera (index 0 s/d %d)..." % (MAX_CAMERA_INDEX - 1))
    found = scan_cameras()
    if not found:
        print("Tidak ada kamera yang bisa dibuka.")
        camera_diagnostics()
        return 1
    for h in found:
        info = f"{h['size'][0]}x{h['size'][1]}" if h["size"] else "tidak ada frame"
        print(f"  index {h['index']}: BISA dibuka (backend {h['backend_name']}, {info})")
    print(f"Pakai:  python main.py --camera {found[0]['index']}")
    return 0


def preflight_camera(gui_app, explicit):
    index = gui_app.CAMERA_INDEX
    hit = probe_index(index)
    if hit:
        gui_app.CAMERA_BACKEND = hit["backend"]
        info = f"{hit['size'][0]}x{hit['size'][1]}" if hit["size"] else "belum ada frame"
        print(f"Kamera index {index} OK (backend {hit['backend_name']}, {info})")
        return True

    print(f"ERROR: kamera index {index} tidak bisa dibuka.")
    others = [h for h in scan_cameras(skip=index) if h["frame_ok"]]
    if others and not explicit:
        h = others[0]
        gui_app.CAMERA_INDEX = h["index"]
        gui_app.CAMERA_BACKEND = h["backend"]
        print(f"       Index lain yang bisa dibuka: {[o['index'] for o in others]}.")
        print(f"       Memakai index {h['index']} (backend {h['backend_name']}). "
              f"Untuk memilih sendiri: --camera N")
        return True
    if others:
        print(f"       Index lain yang bisa dibuka: {[o['index'] for o in others]} "
              f"(pakai --camera N).")
    camera_diagnostics()
    return False

def parse_args(argv):
    ap = argparse.ArgumentParser(description="Sistem kontrol lengan robot: kamera + MediaPipe + IK + Arduino/PCA9685.")
    ap.add_argument("--port", help="port serial Arduino (mis. COM5 atau /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, help="baud rate serial (harus sama dengan servo_control.ino)")
    ap.add_argument("--camera", type=int, help="index kamera untuk cv2.VideoCapture")
    ap.add_argument("--width", type=int, help="lebar capture kamera")
    ap.add_argument("--height", type=int, help="tinggi capture kamera")
    ap.add_argument("--list-ports", action="store_true", help="tampilkan port serial yang terdeteksi lalu keluar")
    ap.add_argument("--list-cameras", action="store_true", help="cek kamera yang bisa dibuka + diagnosis lalu keluar")
    ap.add_argument("--skip-camera-check", action="store_true", help="lewati pemeriksaan kamera sebelum GUI dibuka")
    return ap.parse_args(argv)

def main(argv=None):
    args = parse_args(argv)

    if args.list_ports:
        return list_ports()

    if not check_dependencies():
        return 1

    if args.list_cameras:
        return list_cameras()

    try:
        import gui_app
    except ImportError as e:
        print(f"ERROR: gagal memuat gui_app.py ({e}).")
        print("       Pastikan camera.py, pointing_ik.py, robot_model.py, servo_config.py "
              "dan servo_link.py berada di folder yang sama dengan main.py.")
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

    if not args.skip_camera_check:
        _quiet_opencv()
        if not preflight_camera(gui_app, explicit=args.camera is not None):
            print("Jalankan  python main.py --list-cameras  untuk memeriksa ulang, atau "
                  "--skip-camera-check untuk tetap membuka GUI.")
            return 1

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