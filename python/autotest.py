import tkinter as tk
from tkinter import ttk, messagebox
import requests
import json

# ---------------- 配置默认 URL 和参数 ----------------
DEFAULT_DETECT_URL   = "http://192.168.0.205:5000/detect"
DEFAULT_MOVE_URL     = "http://192.168.0.102:5000/control/plan_joint_path"
DEFAULT_RECOVER_URL  = "http://192.168.0.102:5000/recover"
DEFAULT_GRIPPER_URL  = "http://192.168.0.102:5000/control/go_to_gripper_state"

# 预设位姿
CAMERA_POSE  = (0.6, -0.2, 0.4)   # 拍照位
KITTING_POSE = (0.588,  0.2378, 0.2)   # kitting 位
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

        self._build_widgets()

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

        # ----- 控制按钮区域 -----
        btn_frame = ttk.LabelFrame(self.root, text="Controls")
        btn_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")

        # 预设位姿
        ttk.Button(
            btn_frame,
            text="Go to CAMERA pose  (0.6, -0.2, 0.4)",
            command=self.go_to_camera_pose
        ).grid(row=0, column=0, columnspan=2, pady=3, sticky="ew")

        ttk.Button(
            btn_frame,
            text="Go to KITTING pose (0.588, 0.2378, 0.2)",
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
            text="Recovery",
            command=self.recover
        ).grid(row=6, column=0, pady=3, sticky="ew")

        ttk.Button(
            btn_frame,
            text="Quit",
            command=self.root.quit
        ).grid(row=6, column=1, pady=3, sticky="ew")

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
        self.control_gripper(0.08)

    def gripper_close(self):
        self.control_gripper(0.01)

    # ----------------- Recovery -----------------
    def recover(self):
        url = self.recover_url_var.get().strip()
        self.log("Calling Recovery API...")
        try:
            self.http_get(url, timeout=20)
        except Exception as e:
            self.log(f"[ERROR] Recovery failed: {e}")
            messagebox.showerror("Recovery error", str(e))


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()
