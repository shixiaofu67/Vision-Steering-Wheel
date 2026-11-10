# VisionWheel — 基于 OpenCV + ArUco 的视觉方向盘控制系统

用普通 USB 摄像头识别贴在方向盘上的 ArUco 码，实时解算方向盘转角，再通过键盘 PWM 输出映射为游戏中的转向输入。

单目视觉方案，无需编码器、无需改线，一个摄像头 + 一张 6cm 的 ArUco 码即可工作。

---

## 效果

```
方向盘转动  →  摄像头拍到 ArUco 码  →  PnP 解算姿态  →  与基准姿态求转角
            →  One Euro 滤波  →  死区滞回  →  摇杆值 [-1, 1]  →  键盘 PWM 输出
```

实时叠加显示：当前角度、摇杆值、相机距离、重投影误差、帧率。

---

## 原理

| 步骤 | 做法 | 目的 |
|------|------|------|
| 1. 检测 | `cv2.aruco.ArucoDetector` + `CORNER_REFINE_SUBPIX` | 得到 ArUco 码 4 个亚像素角点 |
| 2. 位姿 | `cv2.solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE)` | 正方形码位姿有两组解，全部返回 |
| 3. 选解 | 重投影误差 + 前后帧姿态连贯性（`rot_dist`）双重判据 | 避免翻转解导致角度跳变 |
| 4. 转角 | `cos θ = (trace(Raᵀ·Rb) - 1) / 2` 的等价二维投影公式 | 求当前姿态相对回正基准的转角 |
| 5. 滤波 | One Euro Filter | 静止时不抖，快速转动时不拖 |
| 6. 死区 | 4° 进入 / 6° 退出滞回 | 防止零位附近抖动导致按键闪烁 |
| 7. 输出 | 线性映射到 [-1, 1]，由独立线程做 PWM 键盘输出 | 模拟摇杆量，而非开关量 |

关键技术点：

- **双解筛选**：`solvePnPGeneric` 对平面正方形会返回两个解，仅取误差最小解在斜视角下会频繁翻转，因此加入"与上一帧姿态旋转距离最小"的连贯性判据。
- **滤波与延迟的平衡**：One Euro 的 `mincut` / `beta` 决定静止抖动与快速响应之间的取舍，参数集中在文件头部。
- **独立 PWM 线程**：键盘输出放在独立线程（`PulseKey`），周期 40ms，占空比等于摇杆值；把按键逻辑从视觉主循环中解耦，避免显示卡顿影响按键时序。

---

## 目录结构

```
VisionWheel/
├─ aruco_chess/            # 入门示例
│   ├─ aruco_detect.py     #   最小 ArUco 检测 + 画框
│   ├─ step_asset.py       #   生成 ArUco 码 / 棋盘格图片
│   ├─ fbl.py              #   逐步练习脚本
│   ├─ aruco_id0..png      #   可直接打印的 ArUco 码（id=0）
│   └─ chessboard.png      #   可直接打印的棋盘格标定板
├─ calibrate/
│   └─ step_chess_cli.py   # 相机标定：拍照 → 计算内参 → 保存 camera.npz
├─ position_pose/
│   └─ pnp_pose.py         # PnP 位姿解算示例（显示 marker 距离）
├─ cal_angle_keyboard/     # 主程序
│   ├─ main.py             #   完整流程：检测 → 角度 → 键盘输出
│   ├─ angle_detect.py     #   仅角度检测与可视化（不含键盘输出）
│   └─ key_mapper.py       #   PWM 键盘输出线程 PulseKey
└─ README.md
```

---

## 快速开始

### 1. 安装依赖

```bash
pip install -r requirements.txt
```

### 2. 相机标定（生成 `camera.npz`）

```bash
cd calibrate
python step_chess_cli.py            # 弹窗后按空格拍照，建议 15~25 张，q 退出
python step_chess_cli.py calibrate  # 计算内参
```

- 打印 `aruco_chess/chessboard.png`，贴在硬板上，保持平整
- 拍摄时让棋盘以不同角度、不同位置占满画面，避免反光和运动模糊
- **有效图 ≥ 10 张** 才能完成标定，最终要求 `RMS < 0.5`
- 把 `SQUARE_M` 改成你实际打印的黑格边长（米）

产物 `calibrate/camera.npz` 需放到 `cal_angle_keyboard/` 能访问到的位置。

### 3. 打印 ArUco 码

打印 `aruco_chess/aruco_id0..png`，贴在方向盘中心，**方向保持正对摄像头**。
用尺子量出黑框外边缘边长，填入 `main.py` 的 `MARKER_SIZE`（单位：米）。

### 4. 修改路径与参数

`main.py` 与 `position_pose/pnp_pose.py` 中的 `camera.npz` 目前是绝对路径，需改为你自己的路径，例如：

```python
import os
CALIB = os.path.join(os.path.dirname(__file__), 'camera.npz')
data = np.load(CALIB)
```

并按需调整：

- `MARKER_SIZE`：实测 ArUco 黑框边长（米）
- `SIGN`：转向反向时改为 `-1.0`
- `DEAD` / `SAT`：死区角度与最大饱和角度

### 5. 运行

```bash
cd cal_angle_keyboard
python main.py
```

| 按键 | 功能 |
|------|------|
| `C` | 校准：以当前方向盘位置为 0° 基准（保持静止，采集 5 帧） |
| `R` | 清除基准 |
| `Q` / `ESC` | 退出 |

---

## 参数说明

| 参数 | 默认 | 说明 |
|------|------|------|
| `MARKER_SIZE` | 0.06 | ArUco 码黑框边长（米），必须实测 |
| `MARKER_ID` | 0 | 使用的 ArUco 码编号 |
| `DEAD` | 4 | 死区，转角小于该值不输出（度） |
| `SAT` | 90 | 饱和角，超过该值输出满量程（度） |
| `DEAD_EXIT` | 6 | 死区退出阈值，形成滞回（度） |
| `SIGN` | 1.0 | 方向，反向用 `-1.0` |
| `LOST_HOLD` | 0.20 | 丢码后保持输出的时间（秒） |
| `ERR_MAX` | 8 | 重投影误差上限（像素），超过视为无效帧 |
| `OE_MINCUT` | 1.0 | One Euro 最小截止频率，越大越跟手 |
| `OE_BETA` | 0.03 | 速度系数，越大快速转动时延迟越小 |
| `CALIB_N` | 5 | 校准采样帧数 |
| `CALIB_SPREAD` | 2.0 | 校准允许的最大帧间抖动（度） |
| `PERIOD`（key_mapper） | 0.04 | PWM 周期（秒） |
| `MIN_DUTY`（key_mapper） | 0.22 | 最小按下占空比 |
| `FULL`（key_mapper） | 0.85 | 摇杆值超过该值改为长按 |

---

## 常见问题

**Q：帧率只有 30fps，怎么提高？**
多数摄像头在 1280×720 下默认使用 YUY2 格式，USB 带宽只够 30fps。需先设置 MJPG：

```python
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))   # 必须最先设
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
```

若实测读帧仍为 30fps，说明相机硬件不支持该组合。注意 60fps 下曝光时间减半，需保证光照充足。

**Q：绿框时有时无、检测不稳？**
- 去掉 `GaussianBlur`：ArUcoDetector 内部已有自适应阈值，前置模糊会糊掉 marker 黑白边界
- 放宽 `params.minMarkerPerimeterRate`（默认 0.03 → 0.01）：marker 画面占比小时默认参数会在内部直接过滤

**Q：按键没反应？**
`keyboard` 库需要管理员权限才能 hook 物理按键；若有游戏反作弊拦截，只能让 OpenCV 窗口保持前台，用 `cv2.waitKey` 通道。

**Q：角度方向反了？**
把 `SIGN` 改为 `-1.0`。

---

## 已知限制

- 单目方案依赖标定精度，更换摄像头、分辨率或对焦后需重新标定
- ArUco 码被遮挡或大角度倾斜时会丢失，超过 `LOST_HOLD` 后输出回正
- 强光、反光、运动模糊会明显降低检测稳定性
- 基于视觉的角度估计存在约 1~2° 的抖动，已由 One Euro 滤波抑制

---

## 免责声明

本项目通过模拟键盘输入实现控制，**仅限单人/离线场景与学习研究使用**。在联机游戏中使用可能违反游戏服务条款或触发反作弊，由此产生的任何后果由使用者自行承担。

---

## License

[MIT](LICENSE)
