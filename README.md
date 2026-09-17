# 高速崎岖地面轮腿机器人 MuJoCo 平台

本项目用于研究高速运行条件下的轮腿机器人机械结构、崎岖地形适应与控制。当前已建立可复现环境，并导入未经修改的 Upkie 基线模型。

## 环境

- Python 3.12
- MuJoCo 3.13.0
- uv 锁定 Python 依赖
- Linux 无显示环境使用 OSMesa 渲染；有 GPU 的环境后续可切换到 EGL

MuJoCo 的 Python wheel 已包含仿真库，无需另行下载 MuJoCo 二进制包。Ubuntu/Debian 的无界面渲染需要以下系统库：

```bash
sudo apt-get update
sudo apt-get install -y libegl1 libgl1 libglfw3 libosmesa6
```

安装项目依赖并运行验证：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv sync --frozen
uv run python scripts/verify_mujoco.py
uv run python scripts/verify_upkie_baseline.py
uv run python scripts/verify_flexible_hip.py
uv run python scripts/verify_four_bar.py
uv run python scripts/verify_terrains.py
uv run python scripts/verify_flat_control.py
uv run python scripts/verify_joint_terrain_control.py
uv run python scripts/verify_high_speed.py
uv run pytest
```

验证脚本会加载 `models/environment_smoke.xml`，让刚体自由落体并与地面接触，然后通过 OSMesa 生成一帧无界面图像。它同时检查数值稳定性、接触检测和渲染输出。

生成四连杆主动伸缩和被动回弹的图片/视频：

```bash
uv run python scripts/render_four_bar_demo.py
uv run python scripts/render_terrain_comparison.py
uv run python scripts/render_sleeve_safety.py
uv run python scripts/render_flat_control_demo.py
uv run python scripts/render_joint_terrain_demo.py
uv run python scripts/render_high_speed_demo.py
```

联合控制视频使用宽幅斜侧跟随镜头和高对比度地形材质，画面会同时保留机器人、前方凸起和已通过的路面。复现柔顺参数搜索：

```bash
uv run python scripts/tune_compliance.py
uv run python scripts/tune_high_speed_damping.py
```

若本机有图形桌面，可打开交互查看器：

```bash
uv run python -m mujoco.viewer --mjcf models/upkie/four_bar/scene.xml
```

## 原型基线

选定的原型是约 5.46 kg 的双轮腿 [Upkie](https://github.com/upkie/upkie)。项目固定引入了 [MjLab Upkie](https://github.com/MarcDcls/mjlab_upkie) 的 Apache-2.0 MJCF 变体；来源版本、许可和模型性质见 `models/upkie/UPSTREAM.md`。`models/upkie/upstream/` 保持不修改，后续机械改型将存放在独立目录中。

## 分阶段计划

1. MuJoCo 环境与自检（已完成）
2. 筛选并集成 Upkie 原型基线（已完成）
3. 建立在前后 x、垂向 z 两个方向具有线性回中力的柔顺髋座（已完成）
4. 将轮腿改为带主动伸缩、被动弹性横杆的四连杆机构（已完成）
5. 构建参数化崎岖路面与分级测试场景（已完成）
6. 建立控制器、参数辨识和高速运行调参/评估流程（进行中：高速速度包线与降速策略完成）

当前平地控制器采用速度 PI 外环和机身俯仰 PD 内环，目标倾角带限幅和变化率限制。确定性批量仿真得到的初始增益可使机器人以 2.0 m/s（7.2 km/h）目标速度完成加速、巡航、制动，并从 8° 初始俯仰偏差恢复。轮毂执行器已改为 ±6 N·m 直接力矩电机；这是高速改装参数，不代表已验证连续热能力。

联合控制增加地形射线预瞄、独立左右腿长、车身高度/横滚反馈以及差速偏航控制。240 mm 直径车轮与双级伸缩横杆配合后，在 `medium` 地形以 1.5 m/s 目标速度通过 13.7 m；重心高度标准差约 10.9 mm，最大偏差约 39.3 mm。横杆全行程可将腿高从 348 mm 降至约 169 mm（48.7%），并在最大伸长时保持至少 26 mm 套筒搭接。

柔顺参数采用固定随机种子的 52 组候选进行有界多目标搜索。部署的工程整值为髋座 x 方向 `3000 N/m, 100 N·s/m`、z 方向 `7000 N/m, 180 N·s/m`、整根横杆等效 `800 N/m, 50 N·s/m`。其综合得分距连续参数数学最优点约 0.08%，但更容易实现。相对上一组参数，垂向加速度 RMS 下降约 45%，95 分位下降约 73%，横杆 95 分位作用力下降约 41%，重心高度标准差仅由 10.94 mm 变为 11.02 mm。

80 km/h 在平地缓加速测试中可达到 21.44 m/s 的末段均速，但在 ±30 mm、RMS 10.7 mm 的长波起伏路面上约 1.0 s 后横滚超过 15°，因此没有作为可用速度。按失败后回退策略，27 组高速阻尼组合中只有 `140 / 220 / 60 N·s/m`（髋 x / 髋 z / 横杆等效）在 9.7、10.0、10.3 m/s 三点全部稳定。当前可靠标称速度为 10.0 m/s（36 km/h），12 s 行驶 120.4 m，重心高度标准差 2.26 mm。

该高速结果使用速度相关半主动阻尼：3 m/s 以下保留崎岖地形参数，8 m/s 以上切换到高速阻尼。下一步需扩大高阻尼速度带、研究横滚共振，并评估轮毂电机和可调阻尼器的连续功率、峰值载荷及热限制。
