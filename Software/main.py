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
        self.root.title("LQG DC Motor Controller")
        self.root.geometry("1000x650")

        # ================= UART =================

        self.ser = None
        self.connected = False

        # ================= DATA =================

        self.max_points = 150

        self.time_data = deque(maxlen=self.max_points)
        self.sp_data = deque(maxlen=self.max_points)
        self.rpm_data = deque(maxlen=self.max_points)

        self.current_rpm = 0
        self.current_pwm = 0
        self.current_xhat = 0

        self.start_time = time.time()

        self.create_gui()

        self.root.after(100, self.update_plot)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ==========================================================
    # GUI
    # ==========================================================

    def create_gui(self):

        left = tk.Frame(self.root)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = tk.Frame(self.root, width=280)
        right.pack(side=tk.RIGHT, fill=tk.Y)

        right.pack_propagate(False)

        # ------------------------------------------------------
        # SETPOINT
        # ------------------------------------------------------

        tk.Label(
            right,
            text="SETPOINT RPM",
            font=("Arial", 12, "bold")
        ).pack(pady=(15, 5))

        self.ent_setpoint = tk.Entry(
            right,
            font=("Arial", 12)
        )

        self.ent_setpoint.insert(0, "400")
        self.ent_setpoint.pack(fill=tk.X, padx=15)

        # ------------------------------------------------------
        # DIRECTION
        # ------------------------------------------------------

        tk.Label(
            right,
            text="DIRECTION",
            font=("Arial", 12, "bold")
        ).pack(pady=(15, 5))

        self.dir_var = tk.IntVar(value=1)

        tk.Radiobutton(
            right,
            text="Forward",
            variable=self.dir_var,
            value=1
        ).pack(anchor="w", padx=20)

        tk.Radiobutton(
            right,
            text="Reverse",
            variable=self.dir_var,
            value=-1
        ).pack(anchor="w", padx=20)

        tk.Radiobutton(
            right,
            text="Stop",
            variable=self.dir_var,
            value=0
        ).pack(anchor="w", padx=20)

        # ------------------------------------------------------
        # MODE
        # ------------------------------------------------------

        tk.Label(
            right,
            text="CONTROL MODE",
            font=("Arial", 12, "bold")
        ).pack(pady=(15, 5))

        self.mode_var = tk.IntVar(value=1)

        tk.Radiobutton(
            right,
            text="Manual PWM",
            variable=self.mode_var,
            value=0
        ).pack(anchor="w", padx=20)

        tk.Radiobutton(
            right,
            text="LQG Control",
            variable=self.mode_var,
            value=1
        ).pack(anchor="w", padx=20)

        # ------------------------------------------------------
        # SEND
        # ------------------------------------------------------

        tk.Button(
            right,
            text="APPLY",
            font=("Arial", 11, "bold"),
            command=self.send_command
        ).pack(
            fill=tk.X,
            padx=15,
            pady=15
        )

        ttk.Separator(right).pack(
            fill=tk.X,
            padx=10,
            pady=10
        )

        # ------------------------------------------------------
        # MONITOR
        # ------------------------------------------------------

        self.lbl_rpm = tk.Label(
            right,
            text="RPM : 0",
            font=("Arial", 12)
        )

        self.lbl_rpm.pack(anchor="w", padx=15, pady=3)

        self.lbl_pwm = tk.Label(
            right,
            text="PWM : 0",
            font=("Arial", 12)
        )

        self.lbl_pwm.pack(anchor="w", padx=15, pady=3)

        self.lbl_xhat = tk.Label(
            right,
            text="x̂ : 0",
            font=("Arial", 12)
        )

        self.lbl_xhat.pack(anchor="w", padx=15, pady=3)

        self.lbl_mode = tk.Label(
            right,
            text="Mode : ---",
            font=("Arial", 12)
        )

        self.lbl_mode.pack(anchor="w", padx=15, pady=3)

        self.lbl_dir = tk.Label(
            right,
            text="Direction : ---",
            font=("Arial", 12)
        )

        self.lbl_dir.pack(anchor="w", padx=15, pady=3)

        ttk.Separator(right).pack(
            fill=tk.X,
            padx=10,
            pady=10
        )

        # ------------------------------------------------------
        # COM PORT
        # ------------------------------------------------------

        tk.Label(
            right,
            text="COM PORT",
            font=("Arial", 12, "bold")
        ).pack()

        self.cbx_port = ttk.Combobox(right)

        self.refresh_ports()

        self.cbx_port.pack(
            fill=tk.X,
            padx=15,
            pady=5
        )

        tk.Button(
            right,
            text="Refresh",
            command=self.refresh_ports
        ).pack(
            fill=tk.X,
            padx=15,
            pady=2
        )

        self.btn_connect = tk.Button(
            right,
            text="Connect",
            command=self.connect_uart
        )

        self.btn_connect.pack(
            fill=tk.X,
            padx=15,
            pady=2
        )

        self.btn_disconnect = tk.Button(
            right,
            text="Disconnect",
            state=tk.DISABLED,
            command=self.disconnect_uart
        )

        self.btn_disconnect.pack(
            fill=tk.X,
            padx=15,
            pady=2
        )

        # ------------------------------------------------------
        # GRAPH
        # ------------------------------------------------------

        self.fig = Figure(figsize=(6, 5))

        self.ax = self.fig.add_subplot(111)

        self.ax.set_title("Motor Speed Response")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("RPM")
        self.ax.grid(True)

        self.line_sp, = self.ax.plot(
            [],
            [],
            '--',
            label="Setpoint"
        )

        self.line_rpm, = self.ax.plot(
            [],
            [],
            label="Actual RPM"
        )

        self.ax.legend()

        self.canvas = FigureCanvasTkAgg(
            self.fig,
            master=left
        )

        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH,
            expand=True
        )

    # ==========================================================
    # COM
    # ==========================================================

    def refresh_ports(self):

        ports = [
            p.device
            for p in serial.tools.list_ports.comports()
        ]

        self.cbx_port["values"] = ports

        if ports:
            self.cbx_port.current(0)

    def connect_uart(self):

        port = self.cbx_port.get()

        if not port:
            return

        try:

            self.ser = serial.Serial(
                port,
                115200,
                timeout=0.1
            )

            self.connected = True

            self.btn_connect.config(state=tk.DISABLED)
            self.btn_disconnect.config(state=tk.NORMAL)

            threading.Thread(
                target=self.receive_task,
                daemon=True
            ).start()

        except Exception as e:

            messagebox.showerror(
                "Error",
                str(e)
            )

    def disconnect_uart(self):

        self.connected = False

        if self.ser:
            self.ser.close()

        self.btn_connect.config(state=tk.NORMAL)
        self.btn_disconnect.config(state=tk.DISABLED)

    # ==========================================================
    # SEND COMMAND
    # ==========================================================

    def send_command(self):

        if not self.connected:
            return

        try:

            sp = float(
                self.ent_setpoint.get()
            )

            direction = self.dir_var.get()
            mode = self.mode_var.get()

            cmd = (
                f"SET,{sp},{direction},{mode}\r\n"
            )

            self.ser.write(
                cmd.encode()
            )

        except:
            pass

    # ==========================================================
    # RECEIVE
    # ==========================================================

    def receive_task(self):

        while self.connected:

            try:

                line = self.ser.readline()

                if not line:
                    continue

                text = line.decode(
                    errors="ignore"
                ).strip()

                fields = text.split(',')

                if len(fields) != 6:
                    continue

                sp = float(fields[0])
                rpm = float(fields[1])
                pwm = float(fields[2])
                xhat = float(fields[3])

                mode = int(fields[4])
                direction = int(fields[5])

                t = time.time() - self.start_time

                self.time_data.append(t)
                self.sp_data.append(sp)
                self.rpm_data.append(rpm)

                self.current_rpm = rpm
                self.current_pwm = pwm
                self.current_xhat = xhat

                if mode == 1:
                    self.lbl_mode.config(
                        text="Mode : LQG"
                    )
                else:
                    self.lbl_mode.config(
                        text="Mode : Manual"
                    )

                if direction == 1:
                    self.lbl_dir.config(
                        text="Direction : Forward"
                    )

                elif direction == -1:
                    self.lbl_dir.config(
                        text="Direction : Reverse"
                    )

                else:
                    self.lbl_dir.config(
                        text="Direction : Stop"
                    )

            except:
                pass

    # ==========================================================
    # UPDATE GRAPH
    # ==========================================================

    def update_plot(self):

        self.lbl_rpm.config(
            text=f"RPM : {int(self.current_rpm)}"
        )

        self.lbl_pwm.config(
            text=f"PWM : {int(self.current_pwm)}"
        )

        self.lbl_xhat.config(
            text=f"x̂ : {int(self.current_xhat)}"
        )

        if len(self.time_data) > 1:

            self.line_sp.set_data(
                list(self.time_data),
                list(self.sp_data)
            )

            self.line_rpm.set_data(
                list(self.time_data),
                list(self.rpm_data)
            )

            self.ax.relim()
            self.ax.autoscale_view()

            self.canvas.draw_idle()

        self.root.after(
            100,
            self.update_plot
        )

    # ==========================================================
    # CLOSE
    # ==========================================================

    def on_close(self):

        self.disconnect_uart()

        self.root.destroy()


if __name__ == "__main__":

    root = tk.Tk()

    app = LQGMotorGUI(root)

    root.mainloop()