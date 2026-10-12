# 本机器人的工具、坐标系与碰撞模型

最后更新：2026-10-12（对应提交 `ab76eec` 及之后）

本文给所有使用这台 Franka Panda 的人：直接写脚本调 REST API 的、网页 `/ui` 的预设、pipeFit、boxstack 等。这台机器人的末端**不是**标准的 Panda + Franka Hand。2026-10-12 起，MoveIt、TF 和 API 都按实际工具建模，**TCP 的位置因此变了**，请先读第 4 节。

---

## 1. 和标准配置的差异

| 项目 | 标准 Panda + Franka Hand | 本机 | 数据来源 |
|---|---|---|---|
| 法兰与手爪之间 | 直接连接 | **20 mm 转接件** | 实测 |
| 转接件直径 | - | 80 mm | **估计**，待实测 |
| 手指 | Franka 标准手指（橡胶指尖） | 自制手指：弯折铝板 + 3D 打印指尖块 + 泡棉垫 | 照片 |
| 腕部相机 | 无 | Intel RealSense D435，3D 打印支架，装在手爪 -x 面 | 照片 + pipeFit 手眼标定 |

### 自制手指尺寸（单根手指，水平方向为手指张合方向）

| 尺寸 | 数值 | 来源 |
|---|---|---|
| 抓取面（泡棉表面）到金属外侧 | 15 mm | 实测 |
| 泡棉厚度（泡棉比打印块突出的量） | 2 mm | 实测（手绘图） |
| 打印块 + 泡棉高度 | 11 mm | 实测 |
| 打印块下方楔形指尖 | 高 20 mm，厚度从 12 mm 收窄到 2 mm | 实测（手绘图） |
| 手爪白色外壳下沿到指尖最下端 | 67 mm | 实测 |
| 全开时两块泡棉间距 | 80 mm | 实测 |
| 手指宽度（垂直于张合方向） | 25 mm | **估计** |

全开间距 80 mm 说明泡棉抓取面就在标准手指内侧面的位置：**夹爪报告的宽度 = 两块泡棉之间的实际间隙**，完全闭合时两块泡棉刚好贴住。

---

## 2. 坐标系

所有长度单位为 m，沿各自坐标系的 z 轴（法兰朝外、指向手指）。

| 坐标系 | 定义 | 相对法兰 `panda_link8` |
|---|---|---|
| `panda_link8` | 法兰面 | 0 |
| `panda_hand` | 手爪底座 | 绕 z 转 -45°，z **+0.02**（转接件） |
| `panda_hand_tcp` | **API 的 TCP** | `panda_hand` 再沿 z +0.1034，即法兰 **+0.1234** |
| pipeFit 名义 TCP | pipeFit 的 TCP 定义，也是手眼标定所在的坐标系 | 绕 z 转 -45°，z +0.1034（**比 API 的 TCP 高 20 mm**） |

相对 API 的 TCP：

- 泡棉中心约在 TCP **下方 4 mm**（沿手指方向）；
- 指尖最下端约在 TCP **下方 29 mm**；
- 相机光心（手眼标定）约在 TCP 的 (-0.031, -0.032, -0.055)。名义 TCP 下是 (-0.031, -0.032, -0.035)。

可以随时用 API 核对：`GET /robot/tf?from=panda_link8&to=panda_hand_tcp` 应返回位置 [0, 0, 0.1234]、yaw -45°；`GET /scene/info` 里的 `flange_to_tcp_m` 应为 0.1234。

---

## 3. 碰撞模型

MoveIt 用下面这些几何判断碰撞，`/plan/*` 和真实运动用的是同一套。

**写在 URDF 里**（`~/catkin_ws/src/franka_ros` 的 franka_description，补丁见 [`franka_description_local.patch`](franka_description_local.patch)）：
- 手爪整体下移 20 mm（`panda.urdf.xacro` 的 `hand_xyz` 参数，默认 `0 0 0.02`）；
- 每根手指的指尖换成实测轮廓：泡棉 + 打印块 + 金属（15 × 11 mm）、楔形指尖（两层台阶近似，只会偏大）、打印块上方的金属竖臂。碰撞报告里手指的名字仍是 `panda_leftfinger` / `panda_rightfinger`，客户端写的 ACM 条目照常有效。

**API 启动时挂到手上**（`python/restful.py`，不能通过 API 删除）：
- `wrist_camera`：D435 本体用 Intel 官方碰撞 box（`realsense2_description`，25.05 × 90 × 25 mm）每边加 2 mm；支架用 [`python/models/panda_realsense_D4xx.stl`](../python/models/panda_realsense_D4xx.stl)（来自 mvp_grasp）的凸包。位置以法兰为基准，按 pipeFit 手眼标定摆放。
- `flange_adapter`：转接件，半径 0.04（估计）、长 0.02 的圆柱。

可用环境变量调整：`FRANKA_WRIST_CAMERA_BOX="sx,sy,sz,cx,cy,cz"`（名义 TCP 系）、`FRANKA_WRIST_MOUNT=0`、`FRANKA_WRIST_CAMERA=0`、`FRANKA_FLANGE_ADAPTER="半径,长度"` 或 `=0`。

**已知局限**
- 网格碰撞只检查表面：完全落在支架凸包内部的物体不会被报告（实际不会发生）。
- 相机 USB 线等走线没有建模。
- 转接件直径和手指宽度是估计值。

---

## 4. 对使用者的影响（重要）

改动前 MoveIt 以为手在实际位置上方 20 mm。改动后：

1. **同一个 TCP 目标点，手会比以前高 20 mm。** 以前按经验调好的抓取高度（例如网页 `/ui` 的预设、检测抓取用的 z = 0.1、`positions.txt` 里的点位、`scan_auto.py` 和 boxstack 里写死的位置），现在实际会**高 2 cm**。这是安全的方向，但可能抓不到，需要重新调。
2. **`/state` 和 `/joints` 报告的 TCP 位置**，在同一关节角下比以前**低 20 mm**（现在是真实的）。
3. **pipeFit**：
   - pipeFit 的 TCP（法兰 + 0.1034）和 API 的 TCP 相差 20 mm。抓取位姿换算：`T_base_tcpAPI = T_base_tcpPipeFit · Trans(0, 0, 0.02)`；
   - 手眼标定：`T_tcpAPI_cam = Trans(0, 0, -0.02) · T_tcpPipeFit_cam`；
   - 发给 `/plan/dry_run`、`/plan/ik` 的位姿目标按 API 的 TCP 给出；
   - 场景物体的位姿（base 系）不受影响。
4. **夹爪宽度**：报告的宽度就是泡棉之间的间隙，不需要换算。
5. **碰撞更严格**：手指更长、多了相机和支架，以前能规划通过的贴近料箱或桌面的动作，现在可能被判为碰撞。这才是真实情况。

---

## 5. 还没处理的事

- **Franka Desk 的末端设置**仍是标准手爪：`F_T_EE` z = 0.1034、质量 0.73 kg。需要在 Desk 里改成 z = 0.1234，质量填手爪 + 转接件 + 相机 + 支架的实际总和。质量不对会影响重力补偿、外力估计（`/force` 读数）和碰撞反射的灵敏度。
- 实测转接件直径、手指宽度。
- 如果相机或支架拆装过，需要重新做手眼标定。

---

## 6. 复现与验证

在新机器上恢复，或升级 franka_ros 之后：

```bash
cd ~/catkin_ws/src/franka_ros
git apply ~/Documents/FrankaServer/docs/franka_description_local.patch
# 然后重启 franka_control.launch 和 restful.py
```

验证（不运动）：
- `GET /robot/tf?from=panda_link8&to=panda_hand_tcp` → [0, 0, 0.1234]，yaw -45°；
- `GET /scene/info` → `flange_to_tcp_m` 为 0.1234，`builtin_attached` 包含 `wrist_camera` 和 `flange_adapter`；
- `POST /plan/check_state {"use_current": true}` → `valid: true`；
- `POST /plan/check_state {"use_current": true, "gripper_width_m": 0}` → `valid: true`（泡棉刚好贴住）。

仿真测试：`tests/sim/run_all.sh`（见 [`tests/sim/README.md`](../tests/sim/README.md)），2026-10-12 结果为 50/50。真机只读验证同日完成，12 项中 11 项通过。未通过的 1 项是测试阈值过严：IK 解落在相邻的冗余构型上，TCP 误差 0.000 mm。全程机械臂没有运动。

更细的 URDF 改动说明见 [`franka_description_local.md`](franka_description_local.md)。
