import tkinter as tk
from tkinter import ttk, messagebox
import requests
import json

# ---------------- 配置默认 URL 和参数 ----------------
DEFAULT_DETECT_URL   = "http://172.26.0.205:5000/detect"
DEFAULT_MOVE_URL     = "http://172.26.0.212:5000/control/plan_joint_path"
DEFAULT_RECOVER_URL  = "http://172.26.0.212:5000/recover"
DEFAULT_GRIPPER_URL  = "http://172.26.0.212:5000/control/go_to_gripper_state"
DEFAULT_GRASP_URL    = "http://172.26.0.212:5000/control/gripper_grasp"
DEFAULT_MOVEL_URL    = "http://172.26.0.212:5000/control/plan_cartesian_path"
DEFAULT_FORCE_URL    = "http://172.26.0.212:5000/force"
DEFAULT_STATE_URL    = "http://172.26.0.212:5000/state"

# 预设位姿
CAMERA_POSE  = (0.6, -0.27, 0.4)   # 拍照位
KITTING_POSE = (0.488,  0.2878, 0.2)   # kitting 位
KITTING_ABOVE_POSE = (
    KITTING_POSE[0],
    KITTING_POSE[1],
    KITTING_POSE[2] + 0.1
)  # 比 kitting 高 0.1 的位姿

Z_PICK = 0.1   # 物件上方 z


class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Screw Picker GUI")

        # 保存检测到的物件
        self.objects = []
        self.current_index = 0

        # URL 变量
        self.detect_url_var  = tk.StringVar(value=DEFAULT_DETECT_URL)
        self.move_url_var    = tk.StringVar(value=DEFAULT_MOVE_URL)
        self.recover_url_var = tk.StringVar(value=DEFAULT_RECOVER_URL)
        self.gripper_url_var = tk.StringVar(value=DEFAULT_GRIPPER_URL)
        self.grasp_url_var   = tk.StringVar(value=DEFAULT_GRASP_URL)
        self.grasp_force_var = tk.StringVar(value="20")
        self.grasp_width_var = tk.StringVar(value="0.0")
        self.movel_url_var   = tk.StringVar(value=DEFAULT_MOVEL_URL)
        self.step_var        = tk.StringVar(value="0.05")
        self.force_url_var   = tk.StringVar(value=DEFAULT_FORCE_URL)
        self.force_display_var = tk.StringVar(value="Force: -- N")
        self.state_url_var   = tk.StringVar(value=DEFAULT_STATE_URL)

        self._build_widgets()
        self._poll_force()   # 启动实时力读数轮询

    def _build_widgets(self):
        # ----- URL 设置区域 -----
        url_frame = ttk.LabelFrame(self.root, text="API URLs")
        url_frame.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")

        ttk.Label(url_frame, text="Detect URL:").grid(row=0, column=0, sticky="e")
        ttk.Entry(url_frame, textvariable=self.detect_url_var, width=55).grid(row=0, column=1, sticky="w")

        ttk.Label(url_frame, text="Move URL:").grid(row=1, column=0, sticky="e")
        ttk.Entry(url_frame, textvariable=self.move_url_var, width=55).grid(row=1, column=1, sticky="w")

        ttk.Label(url_frame, text="Recover URL:").grid(row=2, column=0, sticky="e")
        ttk.Entry(url_frame, textvariable=self.recover_url_var, width=55).grid(row=2, column=1, sticky="w")

        ttk.Label(url_frame, text="Gripper URL:").grid(row=3, column=0, sticky="e")
        ttk.Entry(url_frame, textvariable=self.gripper_url_var, width=55).grid(row=3, column=1, sticky="w")

        ttk.Label(url_frame, text="State URL:").grid(row=4, column=0, sticky="e")
        ttk.Entry(url_frame, textvariable=self.state_url_var, width=55).grid(row=4, column=1, sticky="w")

        ttk.Label(url_frame, text="Grasp URL:").grid(row=5, column=0, sticky="e")
        ttk.Entry(url_frame, textvariable=self.grasp_url_var, width=55).grid(row=5, column=1, sticky="w")

        ttk.Label(url_frame, text="MoveL URL:").grid(row=6, column=0, sticky="e")
        ttk.Entry(url_frame, textvariable=self.movel_url_var, width=55).grid(row=6, column=1, sticky="w")

        ttk.Label(url_frame, text="Force URL:").grid(row=7, column=0, sticky="e")
        ttk.Entry(url_frame, textvariable=self.force_url_var, width=55).grid(row=7, column=1, sticky="w")

        # ----- 控制按钮区域 -----
        btn_frame = ttk.LabelFrame(self.root, text="Controls")
        btn_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")

        # 预设位姿
        ttk.Button(
            btn_frame,
            text="Go to CAMERA pose",
            command=self.go_to_camera_pose
        ).grid(row=0, column=0, columnspan=2, pady=3, sticky="ew")

        ttk.Button(
            btn_frame,
            text="Go to KITTING pose",
            command=self.go_to_kitting_pose
        ).grid(row=1, column=0, columnspan=2, pady=3, sticky="ew")
        ttk.Button(
            btn_frame,
            text="Go to ABOVE-KITTING (+0.1 z)",
            command=self.go_to_kitting_above_pose
        ).grid(row=2, column=0, columnspan=2, pady=3, sticky="ew")
        # 检测与移动
        ttk.Button(
            btn_frame,
            text="Detect objects",
            command=self.detect_objects
        ).grid(row=3, column=0, columnspan=2, pady=3, sticky="ew")

        ttk.Button(
            btn_frame,
            text="Move to selected (z=0.1)",
            command=self.move_to_selected
        ).grid(row=4, column=0, pady=3, sticky="ew")

        ttk.Button(
            btn_frame,
            text="Move to next (z=0.1)",
            command=self.move_to_next
        ).grid(row=4, column=1, pady=3, sticky="ew")

        # 夹爪控制
        ttk.Button(
            btn_frame,
            text="Open gripper (0.10)",
            command=self.gripper_open
        ).grid(row=5, column=0, pady=3, sticky="ew")

        ttk.Button(
            btn_frame,
            text="Close gripper (0.01)",
            command=self.gripper_close
        ).grid(row=5, column=1, pady=3, sticky="ew")

        # 恢复 & 退出
        ttk.Button(
            btn_frame,
            text="Quit",
            command=self.root.quit
        ).grid(row=6, column=0, columnspan=3, pady=3, sticky="ew")

        # ===== Manual Control 区域: MoveL 点动 + grasp / recovery / state =====
        manual_frame = ttk.LabelFrame(self.root, text="Manual Control (MoveL)")
        manual_frame.grid(row=1, column=1, rowspan=3, padx=10, pady=5, sticky="nsew")

        # 步距
        step_row = ttk.Frame(manual_frame)
        step_row.grid(row=0, column=0, pady=(4, 6), sticky="w")
        ttk.Label(step_row, text="Step (m):").grid(row=0, column=0, sticky="e")
        ttk.Entry(step_row, textvariable=self.step_var, width=7).grid(row=0, column=1, sticky="w", padx=(4, 0))

        # 方向点动: 每个方向一个箭头, 相对当前位姿走 MoveL
        jog = ttk.Frame(manual_frame)
        jog.grid(row=1, column=0, pady=4)
        ttk.Button(jog, text="+Y ↑", width=6, command=lambda: self.jog("y", +1)).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(jog, text="−X ←", width=6, command=lambda: self.jog("x", -1)).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(jog, text="+X →", width=6, command=lambda: self.jog("x", +1)).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(jog, text="−Y ↓", width=6, command=lambda: self.jog("y", -1)).grid(row=2, column=1, padx=2, pady=2)
        ttk.Button(jog, text="+Z ⇑", width=6, command=lambda: self.jog("z", +1)).grid(row=0, column=3, padx=(14, 2), pady=2)
        ttk.Button(jog, text="−Z ⇓", width=6, command=lambda: self.jog("z", -1)).grid(row=2, column=3, padx=(14, 2), pady=2)

        ttk.Separator(manual_frame, orient="horizontal").grid(row=2, column=0, sticky="ew", pady=6)

        # 力控抓取 (从 Controls 移过来)
        grasp_row = ttk.Frame(manual_frame)
        grasp_row.grid(row=3, column=0, pady=3, sticky="w")
        ttk.Label(grasp_row, text="Force (N):").grid(row=0, column=0, sticky="e")
        ttk.Entry(grasp_row, textvariable=self.grasp_force_var, width=6).grid(row=0, column=1, padx=(0, 8))
        ttk.Label(grasp_row, text="Width (m):").grid(row=0, column=2, sticky="e")
        ttk.Entry(grasp_row, textvariable=self.grasp_width_var, width=6).grid(row=0, column=3, padx=(0, 8))
        ttk.Button(grasp_row, text="Grasp (force)", command=self.grasp_force).grid(row=0, column=4)

        # Recovery + State (从 Controls 移过来)
        rs_row = ttk.Frame(manual_frame)
        rs_row.grid(row=4, column=0, pady=3, sticky="w")
        ttk.Button(rs_row, text="Recovery", command=self.recover).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(rs_row, text="Get State", command=self.get_robot_state).grid(row=0, column=1)

        # 实时力读数 + 归零. tare 是纯软件(只记基线), 不动夹爪/机械臂, 不会掉物体.
        force_row = ttk.Frame(manual_frame)
        force_row.grid(row=5, column=0, pady=(8, 3), sticky="w")
        ttk.Label(force_row, textvariable=self.force_display_var,
                  font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Button(force_row, text="Tare (归零)", command=self.tare).grid(row=1, column=0, pady=(3, 0), padx=(0, 6))
        ttk.Button(force_row, text="Untare", command=self.untare).grid(row=1, column=1, pady=(3, 0))

        # ----- 物件列表 -----
        list_frame = ttk.LabelFrame(self.root, text="Detected objects")
        list_frame.grid(row=2, column=0, padx=10, pady=5, sticky="nsew")

        self.obj_listbox = tk.Listbox(list_frame, width=60, height=10)
        self.obj_listbox.grid(row=0, column=0, sticky="nsew")

        scroll = ttk.Scrollbar(list_frame, orient="vertical", command=self.obj_listbox.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.obj_listbox.config(yscrollcommand=scroll.set)

        # ----- 日志输出 -----
        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.grid(row=3, column=0, padx=10, pady=5, sticky="nsew")

        self.log_text = tk.Text(log_frame, width=80, height=10, state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")

        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.config(yscrollcommand=log_scroll.set)

        # 自适应布局
        for i in range(4):
            self.root.rowconfigure(i, weight=0)
        self.root.rowconfigure(3, weight=1)
        self.root.columnconfigure(0, weight=1)

    # ----------------- 辅助函数 -----------------
    def log(self, msg):
        self.log_text.config(state="normal")
        self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def http_get(self, url, params=None, timeout=20):
        self.log(f"HTTP GET: {url}  params={params}")
        resp = requests.get(url, params=params, timeout=timeout)
        self.log(f"Response status: {resp.status_code}")
        self.log(f"Body: {resp.text}")
        resp.raise_for_status()
        return resp

    def move_to(self, x, y, z):
        url = self.move_url_var.get().strip()
        try:
            return self.http_get(url, params={"x": x, "y": y, "z": z})
        except Exception as e:
            self.log(f"[ERROR] Move failed: {e}")
            messagebox.showerror("Move error", str(e))

    # ----------------- 位姿按钮 -----------------
    def go_to_camera_pose(self):
        x, y, z = CAMERA_POSE
        self.log(f"Going to CAMERA pose {CAMERA_POSE}...")
        self.move_to(x, y, z)

    def go_to_kitting_pose(self):
        x, y, z = KITTING_POSE
        self.log(f"Going to KITTING pose {KITTING_POSE}...")
        self.move_to(x, y, z)
    def go_to_kitting_above_pose(self):
        x, y, z = KITTING_ABOVE_POSE
        self.log(f"Going to ABOVE-KITTING pose {KITTING_ABOVE_POSE}...")
        self.move_to(x, y, z)

        # ----------------- 检测 & 移动 -----------------
    def detect_objects(self):
        url = self.detect_url_var.get().strip()
        try:
            resp = self.http_get(url, timeout=20)
            data = resp.json()
        except json.JSONDecodeError as e:
            self.log(f"[ERROR] JSON parse error: {e}")
            messagebox.showerror("JSON error", f"Cannot parse JSON: {e}")
            return
        except Exception as e:
            self.log(f"[ERROR] Detect failed: {e}")
            messagebox.showerror("Detect error", str(e))
            return

        if not isinstance(data, list):
            self.log("[ERROR] Detect API did not return a list.")
            messagebox.showerror("Data error", "Detect API did not return a list.")
            return

        self.objects = data
        self.current_index = 0
        self.obj_listbox.delete(0, "end")

        for i, obj in enumerate(self.objects, start=1):
            name = obj.get("name", "object")
            x = obj.get("x")
            y = obj.get("y")
            self.obj_listbox.insert("end", f"{i}. {name}  x={x:.3f}, y={y:.3f}")

        self.log(f"Detected {len(self.objects)} objects.")

    def move_to_selected(self):
        if not self.objects:
            messagebox.showinfo("Info", "No objects. Please detect first.")
            return

        sel = self.obj_listbox.curselection()
        if not sel:
            messagebox.showinfo("Info", "No selection. Please select an object in the list.")
            return

        idx = sel[0]
        obj = self.objects[idx]
        x = float(obj["x"])
        y = float(obj["y"])
        name = obj.get("name", "object")

        self.log(f"Moving to selected object #{idx+1}: {name}")
        self.move_to(x, y, Z_PICK)

        self.current_index = idx

    def move_to_next(self):
        if not self.objects:
            messagebox.showinfo("Info", "No objects. Please detect first.")
            return

        idx = self.current_index
        obj = self.objects[idx]
        x = float(obj["x"])
        y = float(obj["y"])
        name = obj.get("name", "object")

        self.log(f"Moving to next object #{idx+1}: {name}")
        self.obj_listbox.selection_clear(0, "end")
        self.obj_listbox.selection_set(idx)
        self.obj_listbox.see(idx)

        self.move_to(x, y, Z_PICK)

        self.current_index = (self.current_index + 1) % len(self.objects)

    # ----------------- 夹爪控制 -----------------
    def control_gripper(self, width):
        url = self.gripper_url_var.get().strip()
        self.log(f"Setting gripper width = {width}")
        try:
            self.http_get(url, params={"width": width}, timeout=20)
        except Exception as e:
            self.log(f"[ERROR] Gripper failed: {e}")
            messagebox.showerror("Gripper error", str(e))

    def gripper_open(self):
        # 张开夹爪是唯一会让物体"突然掉下来"的动作 -> 先确认, 防误点.
        if not messagebox.askyesno(
                "确认张开夹爪",
                "确定张开夹爪吗?\n如果此刻正夹着物体, 物体会掉下来!"):
            self.log("Open gripper cancelled.")
            return
        self.control_gripper(0.08)

    def gripper_close(self):
        self.control_gripper(0.005)

    def grasp_force(self):
        """力控抓取: /control/gripper_grasp (franka_gripper/grasp action)."""
        url = self.grasp_url_var.get().strip()
        try:
            force = float(self.grasp_force_var.get())
            width = float(self.grasp_width_var.get())
        except ValueError:
            messagebox.showerror("Input error", "Force / Width must be numbers")
            return
        self.log(f"Force grasp: width={width} m, force={force} N")
        try:
            # eps_out 放宽,未知宽度也能正确判定成功; max_retries=0 防搬运中复位重试扰动抓握
            self.http_get(url, params={
                "width": width, "force": force, "speed": 0.05,
                "eps_out": 0.08, "max_retries": 0,
            }, timeout=20)
        except Exception as e:
            self.log(f"[ERROR] Grasp failed: {e}")
            messagebox.showerror("Grasp error", str(e))

    # ----------------- 手动点动 (MoveL) -----------------
    def jog(self, axis, sign):
        """MoveL 点动: 读当前位姿, 沿 axis 走 sign*step 米, 调 plan_cartesian_path (直线)."""
        try:
            step = float(self.step_var.get())
        except ValueError:
            messagebox.showerror("Input error", "Step must be a number")
            return
        try:
            st = self.http_get(self.state_url_var.get().strip(), timeout=5).json()
            pos = st["position"]
            x, y, z = float(pos["x"]), float(pos["y"]), float(pos["z"])
        except Exception as e:
            self.log(f"[ERROR] read state for jog: {e}")
            messagebox.showerror("State error", str(e))
            return
        d = sign * step
        if axis == "x":
            x += d
        elif axis == "y":
            y += d
        elif axis == "z":
            z += d
        self.log(f"MoveL jog {axis}{'+' if sign > 0 else '-'} {step} m -> ({x:.3f}, {y:.3f}, {z:.3f})")
        url = self.movel_url_var.get().strip()
        try:
            self.http_get(url, params={"x": x, "y": y, "z": z}, timeout=30)
        except Exception as e:
            self.log(f"[ERROR] MoveL failed: {e}")
            messagebox.showerror("MoveL error", str(e))

    # ----------------- 力读数 & 归零 -----------------
    def _poll_force(self):
        """每 0.5s 读一次 /force 更新显示. 纯读取, 出错静默(不刷屏 log)."""
        try:
            r = requests.get(self.force_url_var.get().strip(), timeout=1.5)
            if r.status_code == 200:
                d = r.json()
                self.force_display_var.set(
                    "Force: net %.2f N / raw %.2f N  (%s)" % (
                        d.get("magnitude", 0.0), d.get("raw_magnitude", 0.0),
                        d.get("baseline_source", "?")))
            else:
                self.force_display_var.set("Force: (unavailable %d)" % r.status_code)
        except Exception:
            self.force_display_var.set("Force: (server?)")
        finally:
            self.root.after(500, self._poll_force)

    def tare(self):
        """力/力矩归零. 纯软件: 只把当前 F_ext 记为基线, 不动夹爪/机械臂, 不会掉物体."""
        base = self.force_url_var.get().strip()
        try:
            self.http_get(base + "/tare", timeout=5)
            self.log("Tared: force/torque baseline zeroed (no gripper/arm motion).")
        except Exception as e:
            self.log(f"[ERROR] Tare failed: {e}")
            messagebox.showerror("Tare error", str(e))

    def untare(self):
        """清除基线, 恢复显示原始 F_ext. 同样纯软件, 不会掉物体."""
        base = self.force_url_var.get().strip()
        try:
            self.http_get(base + "/untare", timeout=5)
            self.log("Untared: baseline cleared.")
        except Exception as e:
            self.log(f"[ERROR] Untare failed: {e}")
            messagebox.showerror("Untare error", str(e))

    # ----------------- Recovery -----------------
    def recover(self):
        url = self.recover_url_var.get().strip()
        self.log("Calling Recovery API...")
        try:
            self.http_get(url, timeout=20)
        except Exception as e:
            self.log(f"[ERROR] Recovery failed: {e}")
            messagebox.showerror("Recovery error", str(e))

    # ----------------- State -----------------
    def get_robot_state(self):
        url = self.state_url_var.get().strip()
        self.log("Getting Robot State...")
        try:
            resp = self.http_get(url, timeout=5)
            data = resp.json()
            # Format the output nicer
            pos = data.get("position", {})
            ori = data.get("orientation", {})
            gripper = data.get("gripper", -1)
            
            msg = (
                f"State:\n"
                f"  Position: x={pos.get('x',0):.3f}, y={pos.get('y',0):.3f}, z={pos.get('z',0):.3f}\n"
                f"  Orientation: x={ori.get('x',0):.3f}, y={ori.get('y',0):.3f}, z={ori.get('z',0):.3f}, w={ori.get('w',0):.3f}\n"
                f"  Gripper: {'OPEN' if gripper==1 else 'CLOSED' if gripper==0 else 'UNKNOWN'}"
            )
            self.log(msg)
            messagebox.showinfo("Robot State", msg)
            
        except Exception as e:
            self.log(f"[ERROR] Get State failed: {e}")
            messagebox.showerror("State error", str(e))

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()
