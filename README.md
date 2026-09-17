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
uv run pytest
```

验证脚本会加载 `models/environment_smoke.xml`，让刚体自由落体并与地面接触，然后通过 OSMesa 生成一帧无界面图像。它同时检查数值稳定性、接触检测和渲染输出。

生成四连杆主动伸缩和被动回弹的图片/视频：

```bash
uv run python scripts/render_four_bar_demo.py
uv run python scripts/render_terrain_comparison.py
uv run python scripts/render_sleeve_safety.py
uv run python scripts/render_flat_control_demo.py
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
6. 建立控制器、参数辨识和高速运行调参/评估流程（进行中：平地初调完成）

当前平地控制器采用速度 PI 外环和机身俯仰 PD 内环，目标倾角带限幅和变化率限制。确定性批量仿真得到的初始增益可使机器人以 2.0 m/s（7.2 km/h）目标速度完成加速、巡航、制动，并从 8° 初始俯仰偏差恢复。轮毂执行器已改为 ±6 N·m 直接力矩电机；这是高速改装参数，不代表已验证连续热能力。

机械参数和控制增益不会直接凭经验固定。下一步会把平地控制器迁移到分级崎岖路面，加入横杆行程控制和冲击指标，再联合调整平衡、速度与腿长控制参数。
