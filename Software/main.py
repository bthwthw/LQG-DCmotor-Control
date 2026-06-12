import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from collections import deque

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class LQGMotorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("USB CDC LQG DC Motor Controller")
        self.root.geometry("1180x780")
        self.root.minsize(1080, 700)

        self.ser = None
        self.connected = False
        self.reader_thread = None
        self.port_map = {}

        self.max_points = 150
        self.time_data = deque(maxlen=self.max_points)
        self.sp_data = deque(maxlen=self.max_points)
        self.rpm_data = deque(maxlen=self.max_points)

        self.current_rpm = 0
        self.current_pwm = 0
        self.current_xhat = 0
        self.current_mode = 0
        self.current_dir = 0

        self.start_time = time.time()

        self.create_gui()
        self.root.after(100, self.update_plot)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def create_gui(self):
        left = tk.Frame(self.root)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = tk.Frame(self.root, width=390)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        # ================= COM PORT =================
        tk.Label(right, text="USB CDC COM PORT", font=("Arial", 12, "bold")).pack(pady=(8, 3))

        self.cbx_port = ttk.Combobox(right, width=35, state="readonly")
        self.cbx_port.pack(fill=tk.X, padx=15, pady=2)

        tk.Label(right, text="BAUDRATE", font=("Arial", 10, "bold")).pack(pady=(4, 2))

        self.cbx_baud = ttk.Combobox(right, values=["9600", "115200"], state="readonly")
        self.cbx_baud.set("9600")
        self.cbx_baud.pack(fill=tk.X, padx=15, pady=2)

        port_btn_frame = tk.Frame(right)
        port_btn_frame.pack(fill=tk.X, padx=15, pady=3)

        tk.Button(port_btn_frame, text="Refresh", command=self.refresh_ports).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3)
        )

        self.btn_connect = tk.Button(port_btn_frame, text="Connect", command=self.connect_usb_cdc)
        self.btn_connect.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0))

        self.btn_disconnect = tk.Button(
            right,
            text="Disconnect",
            state=tk.DISABLED,
            command=self.disconnect_usb_cdc
        )
        self.btn_disconnect.pack(fill=tk.X, padx=15, pady=3)

        self.lbl_status = tk.Label(
            right,
            text="USB CDC : Disconnected",
            font=("Arial", 10),
            fg="red",
            wraplength=350,
            justify="left"
        )
        self.lbl_status.pack(anchor="w", padx=15, pady=(3, 5))

        ttk.Separator(right).pack(fill=tk.X, padx=10, pady=4)

        # ================= SETPOINT / PWM =================
        tk.Label(right, text="SETPOINT / PWM", font=("Arial", 12, "bold")).pack(pady=(4, 3))

        self.ent_setpoint = tk.Entry(right, font=("Arial", 12))
        self.ent_setpoint.insert(0, "150")
        self.ent_setpoint.pack(fill=tk.X, padx=15)

        self.lbl_input_note = tk.Label(
            right,
            text="LQG: nhập RPM mục tiêu, max 250 RPM",
            font=("Arial", 9)
        )
        self.lbl_input_note.pack(pady=(2, 4))

        # ================= DIRECTION =================
        tk.Label(right, text="DIRECTION", font=("Arial", 12, "bold")).pack(pady=(4, 3))

        self.dir_var = tk.IntVar(value=1)

        dir_frame = tk.Frame(right)
        dir_frame.pack(fill=tk.X, padx=15)

        tk.Radiobutton(dir_frame, text="Forward", variable=self.dir_var, value=1).pack(side=tk.LEFT)
        tk.Radiobutton(dir_frame, text="Reverse", variable=self.dir_var, value=-1).pack(side=tk.LEFT)
        tk.Radiobutton(dir_frame, text="Stop", variable=self.dir_var, value=0).pack(side=tk.LEFT)

        # ================= MODE =================
        tk.Label(right, text="CONTROL MODE", font=("Arial", 12, "bold")).pack(pady=(6, 3))

        self.mode_var = tk.IntVar(value=1)

        mode_frame = tk.Frame(right)
        mode_frame.pack(fill=tk.X, padx=15)

        tk.Radiobutton(
            mode_frame,
            text="Manual PWM",
            variable=self.mode_var,
            value=0,
            command=self.update_mode_ui
        ).pack(side=tk.LEFT)

        tk.Radiobutton(
            mode_frame,
            text="LQG Control",
            variable=self.mode_var,
            value=1,
            command=self.update_mode_ui
        ).pack(side=tk.LEFT)

        tk.Button(
            right,
            text="APPLY MOTOR",
            font=("Arial", 11, "bold"),
            command=self.send_command
        ).pack(fill=tk.X, padx=15, pady=7)

        ttk.Separator(right).pack(fill=tk.X, padx=10, pady=4)

        # ================= LQG TUNING =================
        tk.Label(right, text="LQG ONLINE TUNING", font=("Arial", 12, "bold")).pack(pady=(2, 3))

        self.kx_var = tk.DoubleVar(value=4.5)
        self.ki_var = tk.DoubleVar(value=0.8)
        self.l_var = tk.DoubleVar(value=0.30)
        self.kr_var = tk.DoubleVar(value=1.00)
        self.alpha_var = tk.DoubleVar(value=0.10)
        self.rpm_ramp_var = tk.DoubleVar(value=3.0)
        self.pwm_ramp_var = tk.DoubleVar(value=12.0)
        self.pwm_min_var = tk.DoubleVar(value=80.0)
        self.max_step_var = tk.DoubleVar(value=12.0)

        self.create_tuning_row(right, "Kx", self.kx_var, 0.0, 20.0, 0.1)
        self.create_tuning_row(right, "Ki", self.ki_var, 0.0, 5.0, 0.05)
        self.create_tuning_row(right, "L", self.l_var, 0.0, 1.0, 0.01)
        self.create_tuning_row(right, "Kr", self.kr_var, 0.0, 2.0, 0.05)
        self.create_tuning_row(right, "Alpha", self.alpha_var, 0.01, 1.0, 0.01)
        self.create_tuning_row(right, "RPM Ramp", self.rpm_ramp_var, 0.5, 20.0, 0.5)
        self.create_tuning_row(right, "PWM Ramp", self.pwm_ramp_var, 1.0, 50.0, 1.0)
        self.create_tuning_row(right, "PWM Min", self.pwm_min_var, 0.0, 300.0, 5.0)
        self.create_tuning_row(right, "Max Step", self.max_step_var, 1.0, 50.0, 1.0)

        preset_frame = tk.Frame(right)
        preset_frame.pack(fill=tk.X, padx=15, pady=3)

        tk.Button(
            preset_frame,
            text="Smooth",
            command=lambda: self.apply_preset(3.5, 0.4, 0.20, 0.90, 0.08, 2.0, 8.0, 80, 8.0)
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        tk.Button(
            preset_frame,
            text="Normal",
            command=lambda: self.apply_preset(4.5, 0.8, 0.30, 1.00, 0.10, 3.0, 12.0, 80, 12.0)
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        tk.Button(
            preset_frame,
            text="Fast",
            command=lambda: self.apply_preset(6.5, 1.3, 0.45, 1.10, 0.15, 5.0, 18.0, 90, 18.0)
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        tk.Button(
            right,
            text="SEND TUNE",
            font=("Arial", 10, "bold"),
            command=self.send_tune
        ).pack(fill=tk.X, padx=15, pady=4)

        tk.Label(
            right,
            text="Rung nhiều: giảm Ki, L, Max Step. Lên chậm: tăng Kr hoặc Ki. Motor ì: tăng PWM Min.",
            font=("Arial", 8),
            wraplength=350,
            justify="left"
        ).pack(anchor="w", padx=15, pady=(0, 4))

        ttk.Separator(right).pack(fill=tk.X, padx=10, pady=4)

        # ================= MONITOR =================
        self.lbl_rpm = tk.Label(right, text="RPM : 0", font=("Arial", 12))
        self.lbl_rpm.pack(anchor="w", padx=15, pady=1)

        self.lbl_pwm = tk.Label(right, text="PWM : 0", font=("Arial", 12))
        self.lbl_pwm.pack(anchor="w", padx=15, pady=1)

        self.lbl_xhat = tk.Label(right, text="x̂ : 0", font=("Arial", 12))
        self.lbl_xhat.pack(anchor="w", padx=15, pady=1)

        self.lbl_mode = tk.Label(right, text="Mode : ---", font=("Arial", 12))
        self.lbl_mode.pack(anchor="w", padx=15, pady=1)

        self.lbl_dir = tk.Label(right, text="Direction : ---", font=("Arial", 12))
        self.lbl_dir.pack(anchor="w", padx=15, pady=1)

        # ================= GRAPH =================
        self.fig = Figure(figsize=(6, 5))
        self.ax = self.fig.add_subplot(111)

        self.ax.set_title("Motor Speed Response")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("RPM")
        self.ax.grid(True)

        self.line_sp, = self.ax.plot([], [], "--", label="RPM Setpoint (LQG only)")
        self.line_rpm, = self.ax.plot([], [], label="Actual RPM")
        self.ax.legend()

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.refresh_ports()
        self.update_mode_ui()

    # ==========================================================
    # TUNING UI
    # ==========================================================
    def create_tuning_row(self, parent, name, var, min_val, max_val, resolution):
        frame = tk.Frame(parent)
        frame.pack(fill=tk.X, padx=15, pady=1)

        tk.Label(frame, text=f"{name}:", width=8, font=("Arial", 9)).pack(side=tk.LEFT)

        scale = tk.Scale(
            frame,
            from_=min_val,
            to=max_val,
            resolution=resolution,
            orient=tk.HORIZONTAL,
            variable=var,
            showvalue=False,
            length=190
        )
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True)

        entry = tk.Entry(frame, width=7)
        entry.pack(side=tk.LEFT, padx=(5, 0))

        def update_entry(*args):
            entry.delete(0, tk.END)
            entry.insert(0, f"{var.get():.3g}")

        def update_scale(event=None):
            try:
                value = float(entry.get())
                if value < min_val:
                    value = min_val
                if value > max_val:
                    value = max_val
                var.set(value)
            except:
                update_entry()

        var.trace_add("write", update_entry)
        entry.bind("<Return>", update_scale)
        entry.bind("<FocusOut>", update_scale)
        update_entry()

    def apply_preset(self, kx, ki, l, kr, alpha, rpm_ramp, pwm_ramp, pwm_min, max_step):
        self.kx_var.set(kx)
        self.ki_var.set(ki)
        self.l_var.set(l)
        self.kr_var.set(kr)
        self.alpha_var.set(alpha)
        self.rpm_ramp_var.set(rpm_ramp)
        self.pwm_ramp_var.set(pwm_ramp)
        self.pwm_min_var.set(pwm_min)
        self.max_step_var.set(max_step)

    # ==========================================================
    # UI MODE
    # ==========================================================
    def update_mode_ui(self):
        mode = self.mode_var.get()

        if mode == 1:
            self.lbl_input_note.config(text="LQG: nhập RPM mục tiêu, max 250 RPM")
            try:
                value = float(self.ent_setpoint.get())
                if value > 250:
                    self.ent_setpoint.delete(0, tk.END)
                    self.ent_setpoint.insert(0, "250")
            except:
                pass
        else:
            self.lbl_input_note.config(text="Manual: nhập PWM 0-999, Actual RPM đọc từ encoder")
            try:
                value = float(self.ent_setpoint.get())
                if value > 999:
                    self.ent_setpoint.delete(0, tk.END)
                    self.ent_setpoint.insert(0, "999")
            except:
                pass

    # ==========================================================
    # COM FUNCTIONS
    # ==========================================================
    def refresh_ports(self):
        self.port_map.clear()
        ports = list(serial.tools.list_ports.comports())
        display_ports = []

        for p in ports:
            name = f"{p.device} - {p.description}"
            display_ports.append(name)
            self.port_map[name] = p.device

        self.cbx_port["values"] = display_ports

        if display_ports:
            self.cbx_port.current(0)
        else:
            self.cbx_port.set("")

    def get_selected_port(self):
        selected = self.cbx_port.get()
        if not selected:
            return ""
        return self.port_map.get(selected, selected.split(" ")[0])

    def connect_usb_cdc(self):
        port = self.get_selected_port()

        if not port:
            messagebox.showwarning("Warning", "Please select a COM port.")
            return

        try:
            baud = int(self.cbx_baud.get())

            if self.ser:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                except:
                    pass

            time.sleep(0.3)

            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = baud
            self.ser.timeout = 0.1
            self.ser.write_timeout = 0.5

            self.ser.open()
            self.ser.setDTR(True)
            self.ser.setRTS(True)

            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except:
                pass

            self.connected = True
            self.start_time = time.time()

            self.time_data.clear()
            self.sp_data.clear()
            self.rpm_data.clear()

            self.btn_connect.config(state=tk.DISABLED)
            self.btn_disconnect.config(state=tk.NORMAL)

            self.lbl_status.config(
                text=f"USB CDC : Connected ({port}, {baud})",
                fg="green"
            )

            self.reader_thread = threading.Thread(
                target=self.receive_task,
                daemon=True
            )
            self.reader_thread.start()

        except serial.SerialException as e:
            messagebox.showerror(
                "Connection Error",
                f"Cannot open COM port.\n\n"
                f"Error:\n{e}\n\n"
                "Hãy đóng Hercules/Serial Monitor nếu đang mở, "
                "hoặc rút cáp USB STM32 ra rồi cắm lại."
            )
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect_usb_cdc(self):
        self.connected = False
        time.sleep(0.2)

        try:
            if self.ser:
                if self.ser.is_open:
                    self.ser.close()
        except:
            pass

        self.ser = None
        self.btn_connect.config(state=tk.NORMAL)
        self.btn_disconnect.config(state=tk.DISABLED)
        self.lbl_status.config(text="USB CDC : Disconnected", fg="red")

    # ==========================================================
    # SEND COMMANDS
    # ==========================================================
    def send_command(self):
        if not self.connected or not self.ser or not self.ser.is_open:
            messagebox.showwarning("Warning", "USB CDC is not connected.")
            return

        try:
            value = float(self.ent_setpoint.get())
            direction = self.dir_var.get()
            mode = self.mode_var.get()

            if value < 0:
                value = 0.0

            if mode == 1:
                if value > 250:
                    value = 250.0
                    self.ent_setpoint.delete(0, tk.END)
                    self.ent_setpoint.insert(0, "250")
            else:
                if value > 999:
                    value = 999.0
                    self.ent_setpoint.delete(0, tk.END)
                    self.ent_setpoint.insert(0, "999")

            cmd = f"SET,{value},{direction},{mode}\r\n"
            self.ser.write(cmd.encode("ascii"))

        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    def send_tune(self):
        if not self.connected or not self.ser or not self.ser.is_open:
            messagebox.showwarning("Warning", "USB CDC is not connected.")
            return

        try:
            kx = self.kx_var.get()
            ki = self.ki_var.get()
            l = self.l_var.get()
            kr = self.kr_var.get()
            alpha = self.alpha_var.get()
            rpm_ramp = self.rpm_ramp_var.get()
            pwm_ramp = self.pwm_ramp_var.get()
            pwm_min = int(self.pwm_min_var.get())
            max_step = self.max_step_var.get()

            cmd = (
                f"TUNE,{kx},{ki},{l},{kr},{alpha},"
                f"{rpm_ramp},{pwm_ramp},{pwm_min},{max_step}\r\n"
            )

            self.ser.write(cmd.encode("ascii"))

            messagebox.showinfo(
                "Tuning",
                "Sent tuning:\n"
                f"Kx={kx:.3g}, Ki={ki:.3g}, L={l:.3g}, Kr={kr:.3g}\n"
                f"Alpha={alpha:.3g}, RPM Ramp={rpm_ramp:.3g}, PWM Ramp={pwm_ramp:.3g}\n"
                f"PWM Min={pwm_min}, Max Step={max_step:.3g}"
            )

        except Exception as e:
            messagebox.showerror("Tune Error", str(e))

    # ==========================================================
    # RECEIVE
    # ==========================================================
    def receive_task(self):
        while self.connected:
            try:
                if not self.ser or not self.ser.is_open:
                    break

                line = self.ser.readline()
                if not line:
                    continue

                text = line.decode(errors="ignore").strip()
                if not text:
                    continue

                if text.startswith("OK") or text.startswith("ERR"):
                    continue

                fields = text.split(",")

                if len(fields) != 6:
                    continue

                sp = float(fields[0])
                rpm = float(fields[1])
                pwm = float(fields[2])
                xhat = float(fields[3])
                mode = int(float(fields[4]))
                direction = int(float(fields[5]))

                t = time.time() - self.start_time

                self.time_data.append(t)

                if mode == 1:
                    self.sp_data.append(sp)
                else:
                    self.sp_data.append(float("nan"))

                self.rpm_data.append(rpm)

                self.current_rpm = rpm
                self.current_pwm = pwm
                self.current_xhat = xhat
                self.current_mode = mode
                self.current_dir = direction

            except:
                pass

    # ==========================================================
    # UPDATE GRAPH
    # ==========================================================
    def update_plot(self):
        self.lbl_rpm.config(text=f"RPM : {int(self.current_rpm)}")
        self.lbl_pwm.config(text=f"PWM : {int(self.current_pwm)}")
        self.lbl_xhat.config(text=f"x̂ : {int(self.current_xhat)}")

        if self.current_mode == 1:
            self.lbl_mode.config(text="Mode : LQG")
        else:
            self.lbl_mode.config(text="Mode : Manual")

        if self.current_dir == 1:
            self.lbl_dir.config(text="Direction : Forward")
        elif self.current_dir == -1:
            self.lbl_dir.config(text="Direction : Reverse")
        else:
            self.lbl_dir.config(text="Direction : Stop")

        if len(self.time_data) > 1:
            self.line_sp.set_data(list(self.time_data), list(self.sp_data))
            self.line_rpm.set_data(list(self.time_data), list(self.rpm_data))

            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()

        self.root.after(100, self.update_plot)

    def on_close(self):
        self.disconnect_usb_cdc()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = LQGMotorGUI(root)
    root.mainloop()