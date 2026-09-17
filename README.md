# 高速崎岖地面轮腿机器人

MuJoCo 里的轮腿原型：从 Upkie 改出柔顺髋座、菱形四连杆、分级崎岖路面，再用 PID 把 3 倍轮距机体开到约 100 km/h。点开图片可看对应视频。

[![100 km/h Perlin 地形](docs/media/high_speed_100kmh.png)](docs/media/high_speed_100kmh.mp4)

## 已实现

### 菱形四连杆与套筒横杆

左右腿改成前后对称的四连杆，中间是双级伸缩弹性横杆。主动缩短/伸长后切断伺服，机构靠弹簧回中。全伸长时套筒仍保持至少 26 mm 搭接。

[![四连杆三种姿态](docs/media/four_bar_states.png)](docs/media/four_bar.mp4)

![套筒嵌套与最小插入量](docs/media/sleeve.png)

### 分级崎岖路面

按 Unitree 的做法：高度场 + 横向圆木/石块。easy / medium / hard / extreme 四档，前面有平地起飞段。

![四级地形与障碍](docs/media/terrain_levels.png)

### 平地速度 PID

速度 PI 外环、俯仰 PD 内环，目标倾角带限幅。2.0 m/s（7.2 km/h）可完成加速、巡航、制动。轮毂力矩 ±6 N·m。

[![速度跟踪曲线](docs/media/flat_balance_tracking.png)](docs/media/flat_balance.mp4)

### 地形联合控制

射线预瞄地面、左右腿独立伸缩、车身高度/横滚反馈。`medium` 地形上 1.5 m/s 通过约 13.7 m。标称可靠速度 10 m/s（36 km/h）。

[![高度与姿态指标](docs/media/joint_metrics.png)](docs/media/joint_control.mp4)

[36 km/h 长赛道](docs/media/high_speed_36kmh.mp4)

### 100 km/h（3 倍轮距）

轮距 682 mm、甲板 656 mm，是上一版 4 倍轮距的 3/4。高速赛道只剩 Unitree [AddPerlinHeighField](https://github.com/unitreerobotics/unitree_mujoco/blob/main/terrain_tool/readme_zh.md#6addperlinheighfield) 高度场，不再摆圆柱/椭球装饰。3 层 Perlin，沿前进方向 smooth 16 m、横向 32 m，`height_scale` 0.24 m；侧光加阴影，地面用纯色泥土，不铺高对比棋盘。64 s 峰值 **28.41 m/s（102.3 km/h）**，行驶 1081 m，最大俯仰 5.1°、横滚 6.5°。36 km/h 邻域仍在起飞垫上稳定。

连杆仍用 190 mm：站立菱形已有约 174 mm 压缩行程。只加长连杆、不加长横杆，行程会变短。

## 本地运行

需要 Python 3.12 和 [uv](https://docs.astral.sh/uv/)。Linux 无界面渲染再安装 `libosmesa6`。

```bash
uv sync --frozen
uv run python -m mujoco.viewer --mjcf models/upkie/high_speed/scene.xml
```

重新生成演示：

```bash
uv run python scripts/render_four_bar_demo.py --output-directory docs/media
uv run python scripts/render_high_speed_demo.py --output-directory docs/media
```

高速场景在 `models/upkie/high_speed/`，中速四连杆在 `models/upkie/four_bar/`。基线 Upkie 原样放在 `models/upkie/upstream/`，许可见 `models/upkie/UPSTREAM.md`。
