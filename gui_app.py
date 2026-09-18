import time
import tkinter as tk
import cv2
from PIL import Image, ImageTk
from camera import Camera

CAMERA_INDEX = 0
CAM_CAPTURE_W = 1300
CAM_CAPTURE_H = 810
SIDEBAR_W = 280
HEADER_H = 52

BG_HEADER = "#001D4A"
BG_SIDEBAR = "#006992"
BG_CARD = "#EAF8BF"
BG_BTN = "#EAF8BF"
BG_BTN_HOVER = "#ECA400"
BG_BTN_ACTIVE = "#001D4A"

FG_TITLE = "#EAF8BF"
FG_TEXT = "#001D4A"

COL_ACCENT = "#EAF8BF"
COL_GREEN = "#0FBA00"
COL_YELLOW = "#D1DC00"
COL_RED = "#D60000"

FONT_FAMILY = "Arial"

class StatCard(tk.Frame):
    def __init__(self, parent, title, initial_value="-"):
        super().__init__(parent, bg=BG_CARD, bd=0, highlightthickness=0)
        
        self.strip = tk.Frame(self, bg=COL_ACCENT, width=4)
        self.strip.pack(side=tk.LEFT, fill=tk.Y)
        
        self.text_container = tk.Frame(self, bg=BG_CARD)
        self.text_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=7)
        
        self.title_label = tk.Label(
            self.text_container, text=title, fg=FG_TEXT, bg=BG_CARD,
            font=(FONT_FAMILY, 8), anchor="w"
        )
        self.title_label.pack(fill=tk.X)
        
        self.value_label = tk.Label(
            self.text_container, text=initial_value, fg=FG_TEXT, bg=BG_CARD,
            font=(FONT_FAMILY, 9), anchor="w", justify=tk.LEFT, wraplength=SIDEBAR_W - 40
        )
        self.value_label.pack(fill=tk.X, pady=(2, 0))

    def set_value(self, text, color=None):
        self.value_label.config(text=text)
        if color:
            self.value_label.config(fg=color)
        else:
            self.value_label.config(fg=FG_TEXT)

class GuiApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Arm Robot Controller")
        self.geometry("1280x720")
        self.minsize(900, 560)
        self.configure()

        self.is_fullscreen = False
        self.mode = "manual"

        self.detector = Camera()
        # self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_CAPTURE_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_CAPTURE_H)

        self.prev_time = 0
        self.current_image_tk = None

        self._build_ui()
        self._build_shortcuts()
        self._set_mode("manual")

        if not self.cap.isOpened():
            self.fps_card.set_value("Kamera tidak terbuka", COL_RED)
        
        self._update_frame()

    def _build_ui(self):
        self.header = tk.Frame(self, bg=BG_HEADER, height=HEADER_H, bd=0, highlightthickness=0)
        self.header.pack(side=tk.TOP, fill=tk.X)
        self.header.pack_propagate(False)
        
        header_border = tk.Frame(self, bg=COL_ACCENT, height=1)
        header_border.pack(side=tk.TOP, fill=tk.X)

        title = tk.Label(self.header, text="ARM ROBOT CONTROLLER", fg=FG_TITLE, bg=BG_HEADER, font=(FONT_FAMILY, 13, "bold"))
        title.pack(side=tk.LEFT, padx=18)
        
        self.header_mode_label = tk.Label(self.header, text="MODE: MANUAL", fg=COL_ACCENT, bg=BG_HEADER, font=(FONT_FAMILY, 9))
        self.header_mode_label.pack(side=tk.RIGHT)

        body = tk.Frame(self)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.sidebar = tk.Frame(body, bg=BG_SIDEBAR, width=SIDEBAR_W, bd=0, highlightthickness=0)
        self.sidebar.pack(side=tk.RIGHT, fill=tk.Y)
        self.sidebar.pack_propagate(False)

        self.video_container = tk.Frame(body, bg="black")
        self.video_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.video_label = tk.Label(self.video_container, bg="black")
        self.video_label.pack(fill=tk.BOTH, expand=True)

        self._build_sidebar_contents()

    def _build_sidebar_contents(self):
        self._section_title(self.sidebar, "MODE KONTROL").pack(fill=tk.X, pady=(0, 4))

        mode_row = tk.Frame(self.sidebar, bg=BG_SIDEBAR)
        mode_row.pack(fill=tk.X, pady=(0, 10))

        self.btn_mode_manual = self._make_toggle_button(mode_row, "Manual", lambda: self._set_mode("manual"))
        self.btn_mode_manual.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))

        self.btn_mode_pointing = self._make_toggle_button(mode_row, "Pointing", lambda: self._set_mode("pointing"))
        self.btn_mode_pointing.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))

        self._section_title(self.sidebar, "STATUS").pack(fill=tk.X, pady=(0, 4))
        self.fps_card = StatCard(self.sidebar, "FPS")
        self.fps_card.pack(fill=tk.X, pady=(0, 6))

        self.manual_section = tk.Frame(self.sidebar, bg=BG_SIDEBAR)
        self.manual_section.pack(fill=tk.X)

        self.target_card = StatCard(self.manual_section, "TARGET DOF", "Kiri : -\nKanan: -")
        self.target_card.pack(fill=tk.X, pady=(0, 4))
        
        self.command_card = StatCard(self.manual_section, "COMMAND", "MENUNGGU 2 TANGAN")
        self.command_card.pack(fill=tk.X, pady=(0, 4))

        self.pointing_section = tk.Frame(self.sidebar, bg=BG_SIDEBAR)
        self.pointing_section.pack_forget()

        self.pt_target_card = StatCard(self.pointing_section, "TARGET (X, Y, Z)")
        self.pt_target_card.pack(fill=tk.X, pady=(0, 4))
        
        self.pt_ee_card = StatCard(self.pointing_section, "END EFFECTOR")
        self.pt_ee_card.pack(fill=tk.X, pady=(0, 4))
        
        self.pt_servo_card = StatCard(self.pointing_section, "SERVO (1-6)")
        self.pt_servo_card.pack(fill=tk.X, pady=(0, 4))
        
        self.pt_status_card = StatCard(self.pointing_section, "STATUS IK")
        self.pt_status_card.pack(fill=tk.X, pady=(0, 4))

        footer_container = tk.Frame(self.sidebar, bg=BG_SIDEBAR)
        footer_container.pack(side=tk.BOTTOM, fill=tk.X)

        hint = tk.Label(footer_container, text="Fullscreen (F11/Fn+F11)  |  Minimize (Esc)", fg=FG_TITLE, bg=BG_SIDEBAR, font=(FONT_FAMILY, 9))
        hint.pack(fill=tk.X, pady=(0, 10))

        btn_fullscreen = self._make_flat_button(footer_container, "Fullscreen", self._toggle_fullscreen)
        btn_fullscreen.pack(fill=tk.X, pady=(0, 4))

        btn_minimize = self._make_flat_button(footer_container, "Minimize", self._on_minimize)
        btn_minimize.pack(fill=tk.X, pady=(0, 4))

        btn_exit = self._make_flat_button(footer_container, "Exit", self.destroy, hover_color=COL_RED)
        btn_exit.pack(fill=tk.X, pady=(0, 4))

        for child in self.sidebar.winfo_children():
            if child != footer_container:
                child.pack_configure(padx=16)

    def _section_title(self, parent, text):
        wrap = tk.Frame(parent, bg=parent["bg"])
        label = tk.Label(
            wrap, text=text, fg=FG_TITLE, bg=parent["bg"],
            font=(FONT_FAMILY, 11, "bold"), anchor="w"
        )
        label.pack(fill=tk.X, pady=(8, 4))
        line = tk.Frame(wrap, bg=COL_ACCENT, height=1)
        line.pack(fill=tk.X)
        return wrap

    def _make_flat_button(self, parent, text, command, hover_color=None):
        color = hover_color or COL_ACCENT
        btn = tk.Button(
            parent, text=text, command=command, bg=BG_BTN, fg=FG_TEXT,
            activebackground=BG_BTN_HOVER, activeforeground=color,
            font=(FONT_FAMILY, 9), bd=0, relief=tk.FLAT, 
            padx=0, pady=9, 
            cursor="hand2"
        )
        btn.bind("<Enter>", lambda e: btn.config(bg=BG_BTN_HOVER, fg=color))
        btn.bind("<Leave>", lambda e: btn.config(bg=BG_BTN, fg=FG_TEXT))
        return btn

    def _make_toggle_button(self, parent, text, command):
        return tk.Button(
            parent, text=text, command=command, font=(FONT_FAMILY, 9), 
            bd=0, relief=tk.FLAT, 
            padx=0, pady=9, 
            cursor="hand2"
        )

    def _style_toggle_button(self, btn, active):
        if active:
            btn.config(bg=BG_BTN_ACTIVE, fg=BG_BTN, activebackground=BG_BTN_HOVER, activeforeground=BG_BTN, font=(FONT_FAMILY, 9, "bold"))
            btn.unbind("<Enter>")
            btn.unbind("<Leave>")
        else:
            btn.config(bg=BG_BTN, fg=FG_TEXT, activebackground=BG_BTN_HOVER, activeforeground=BG_BTN, font=(FONT_FAMILY, 9))
            btn.bind("<Enter>", lambda e: btn.config(bg=BG_BTN_HOVER, fg=BG_BTN, font=(FONT_FAMILY, 9, "bold")))
            btn.bind("<Leave>", lambda e: btn.config(bg=BG_BTN, fg=FG_TEXT, font=(FONT_FAMILY, 9)))

    def _highlight_mode_button(self, active_btn):
        for btn in (self.btn_mode_manual, self.btn_mode_pointing):
            self._style_toggle_button(btn, active=(btn == active_btn))

    def _build_shortcuts(self):
        self.bind("<F11>", lambda e: self._toggle_fullscreen())
        self.bind("<Escape>", lambda e: self._exit_fullscreen())

    def _set_mode(self, mode):
        self.mode = mode
        if mode == "manual":
            self.pointing_section.pack_forget()
            self.manual_section.pack(fill=tk.X, after=self.fps_card)
            self._highlight_mode_button(self.btn_mode_manual)
            self.header_mode_label.config(text="MODE: MANUAL")
        else:
            self.manual_section.pack_forget()
            self.pointing_section.pack(fill=tk.X, after=self.fps_card)
            self._highlight_mode_button(self.btn_mode_pointing)
            self.header_mode_label.config(text="MODE: POINTING")

            if not self.detector.pointing_enabled:
                self.pt_status_card.set_value("roboticstoolbox belum terinstall", COL_RED)

            self.detector.reset_pointing()
            
        for child in self.sidebar.winfo_children():
            try:
                if child.winfo_class() in ("Frame", "Label") and child.pack_info().get("side") != "bottom":
                    child.pack_configure(padx=16)
            except tk.TclError:
                continue

    def _toggle_fullscreen(self):
        self.is_fullscreen = not self.is_fullscreen
        self.attributes("-fullscreen", self.is_fullscreen)

    def _exit_fullscreen(self):
        if self.is_fullscreen:
            self.is_fullscreen = False
            self.attributes("-fullscreen", False)

    def _on_minimize(self):
        if self.is_fullscreen:
            self.is_fullscreen = False
            self.attributes("-fullscreen", False)
        self.iconify()

    def _update_frame(self):
        success, frame = self.cap.read()
        if not success:
            self.after(15, self._update_frame)
            return

        frame = cv2.flip(frame, 1)
        current_time = time.time()
        fps = 1 / (current_time - self.prev_time) if self.prev_time else 0
        self.prev_time = current_time
        self.fps_card.set_value(f"{int(fps)}")

        if self.mode == "manual":
            self._run_manual_mode(frame)
        else:
            self._run_pointing_mode(frame)

        self._render_video(frame)        
        self.after(15, self._update_frame)

    def _run_manual_mode(self, frame):
        detected = self.detector.process(frame)
        cmd = Camera.get_robot_command(detected)
        target_text = cmd["target"] if cmd["target"] else "-"
        angle_text = f"{cmd['angle']}\u00b0" if cmd["angle"] is not None else "-"
        self.target_card.set_value(f"Kiri : {target_text}\nKanan: {angle_text}")

        if cmd["active"]:
            self.command_card.set_value(f"ACTIVE\n{cmd['target']} -> {cmd['angle']} deg", COL_GREEN)
        elif cmd["target"] is not None:
            self.command_card.set_value(f"BELUM VALID\n{cmd['target']}: kanan harus\nFIST/OPEN PALM", COL_YELLOW)
        else:
            self.command_card.set_value(cmd["reason"], FG_TEXT)

    def _run_pointing_mode(self, frame):
        pt = self.detector.process_pointing(frame)

        if pt["target_xyz"] is not None:
            x, y, z = pt["target_xyz"]
            self.pt_target_card.set_value(f"X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}")
        else:
            self.pt_target_card.set_value("-")

        if pt["ee_pos"] is not None:
            ex, ey, ez = pt["ee_pos"]
            self.pt_ee_card.set_value(f"X:{ex:.3f}, Y:{ey:.3f}, Z:{ez:.3f}")
        else:
            self.pt_ee_card.set_value("-")

        if pt["servo"] is not None:
            servo = pt["servo"]
            lines = "\n".join(f"S{i + 1}: {servo[i]:5.1f} deg" for i in range(len(servo)))
            self.pt_servo_card.set_value(lines)
        else:
            self.pt_servo_card.set_value("-")

        color = COL_GREEN if pt["ik_success"] else FG_TEXT
        self.pt_status_card.set_value(pt["reason"], color)

    def _render_video(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(frame_rgb)

        lbl_w = self.video_label.winfo_width()
        lbl_h = self.video_label.winfo_height()
        
        if lbl_w > 1 and lbl_h > 1:
            img_w, img_h = img_pil.size
            ratio = min(lbl_w / img_w, lbl_h / img_h)
            new_w = int(img_w * ratio)
            new_h = int(img_h * ratio)
            img_pil = img_pil.resize((new_w, new_h), Image.Resampling.LANCZOS)

        image_tk = ImageTk.PhotoImage(img_pil)
        self.current_image_tk = image_tk  
        self.video_label.config(image=image_tk)

    def destroy(self):
        if self.cap.isOpened():
            self.cap.release()
        self.detector.close()
        super().destroy()

if __name__ == "__main__":
    app = GuiApp()
    app.protocol("WM_DELETE_WINDOW", app.destroy)
    app.mainloop()