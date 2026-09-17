# 高速崎岖地面轮腿机器人

MuJoCo 里的轮腿原型：从 Upkie 改出柔顺髋座、菱形四连杆、分级崎岖路面，再用 PID 把 3 倍轮距机体开到约 100 km/h。点开图片可看对应视频。

![Unitree 同款 Perlin 褶皱](docs/media/unitree_perlin.png)

[![机器人站在 Perlin 小块上](docs/media/unitree_perlin_robot.png)](docs/media/unitree_perlin_orbit.mp4)

## 已实现

### Unitree 同款 Perlin 高度场

按 [unitree_mujoco/terrain_tool](https://github.com/unitreerobotics/unitree_mujoco) 里 `AddPerlinHeighField` 的演示调用铺：`size=[2.0, 1.5]`、128 px 图、`smooth=100`、6 层、`height_scale=0.2`。网格加到 384 px，同样的褶皱更细。灰色无纹理，搁在默认蓝棋盘上，侧光把纸皱打出来。

这是一块 2 m × 1.5 m 的毯子，不是 400 m 灰面：相机贴着拍，20 cm 起伏才能看清楚。3 倍轮距机体在这种密褶皱上站得住，但不能用同一套 100 km/h PID 巡航。

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

轮距 682 mm、甲板 656 mm。在棋盘起飞垫上 PID 能拉到 **100 km/h** 并巡航满 64 s。Unitree 演示那种 1.56 m 基波长 / 6 层褶皱，3 倍轮距下没法像缓坡那样在皱面上巡航。36 km/h 邻域仍在垫上稳定。

[![100 km/h 起飞垫](docs/media/high_speed_100kmh.png)](docs/media/high_speed_100kmh.mp4)

连杆仍用 190 mm：站立菱形已有约 174 mm 压缩行程。只加长连杆、不加长横杆，行程会变短。

## 本地运行

需要 Python 3.12 和 [uv](https://docs.astral.sh/uv/)。Linux 无界面渲染再安装 `libosmesa6`。

```bash
uv sync --frozen
uv run python -m mujoco.viewer --mjcf models/upkie/high_speed/scene.xml
```

重新生成演示：

```bash
uv run python scripts/generate_high_speed_track.py
uv run python scripts/render_unitree_perlin.py --output-directory docs/media
uv run python scripts/render_four_bar_demo.py --output-directory docs/media
uv run python scripts/render_high_speed_demo.py --output-directory docs/media
```

高速场景在 `models/upkie/high_speed/`，中速四连杆在 `models/upkie/four_bar/`。基线 Upkie 原样放在 `models/upkie/upstream/`，许可见 `models/upkie/UPSTREAM.md`。
