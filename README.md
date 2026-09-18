# 高速轮腿机器人与崎岖路面

市面上常见的轮腿机器人（以 [Upkie](https://github.com/upkie/upkie) 这类串髋–串膝–轮毂构型为代表）在平地可以平衡和行走，但**无法在高速下通过崎岖路段**。后续仿真实验将用来证明这一点。

侧翻的直接原因是：车轮碰到 bump 时，车身**无法持续提供向下的正压力**。轮端一跳，法向力断掉，横滚/俯仰失去约束，机体就翻。

可能的原因有两条，需要分开验证：

1. **机械**：关节柔性不够，扰动不能被被动吸收，机构本身有缺陷。
2. **控制**：即使用了更柔的腿，高速下控制器也来不及把正压力补回去。

本仓库目前只保留对比用的机械模型和地形，控制与实验后续再加。

## 保留的模型

- **原型机** `models/prototype/`：官方 [Upkie](https://github.com/upkie/upkie)（描述来自 [upkie_description](https://github.com/upkie/upkie_description)）。串髋串膝、轮毂驱动，没有柔性髋座、没有四连杆、没有伸缩杆。
- **三倍轮距机** `models/wide_car/`：在原型构型上改出的对比样机。包含被动柔性髋关节（x–z 滑移）、菱形四连杆、两级套筒伸缩杆，轮距为官方髋距的三倍（±341 mm）。

地形只保留一种：[Unitree `AddPerlinHeighField`](https://github.com/unitreerobotics/unitree_mujoco) 同款 Perlin 高度场。48 m × 4 m，基波长 1.5625 m，6 层，起伏 0.20 m。前 6 m 平地，6–10 m 过渡，之后全幅皱面。

## 环境

Python 3.12，[uv](https://docs.astral.sh/uv/)，MuJoCo 3.13。无界面渲染需要 `libosmesa6`。

```bash
uv sync --frozen
uv run python scripts/generate_perlin.py
uv run python -m mujoco.viewer --mjcf models/prototype/scene.xml
uv run python -m mujoco.viewer --mjcf models/wide_car/scene.xml
uv run pytest
```

---

# High-speed wheel-legged robots on rough terrain

Typical wheel-legged robots on the market — the serial hip–knee–wheel layout of [Upkie](https://github.com/upkie/upkie) is the usual example — can balance and walk on flat ground, but **cannot traverse rough terrain at high speed**. Later simulation experiments will demonstrate this.

Rollover happens because, when a wheel hits a bump, the chassis **cannot keep a continuous downward (positive) contact force**. The wheel unloads, roll and pitch lose their restoring force, and the body tips over.

Two causes need to be tested separately:

1. **Mechanics**: the joints are not compliant enough to absorb the disturbance passively; the mechanism itself is the defect.
2. **Control**: even with a more compliant leg, the controller cannot restore normal force in time at high speed.

This repository currently keeps only the two mechanical models and the terrain. Controllers and experiments come next.

## Models kept

- **Prototype** `models/prototype/`: the official [Upkie](https://github.com/upkie/upkie) (URDF from [upkie_description](https://github.com/upkie/upkie_description)). Serial hip and knee, driven wheels. No compliant hip, no four-bar, no telescopic rod.
- **3×-track machine** `models/wide_car/`: the comparison robot. Passive compliant hips (x–z sliders), diamond four-bar legs, two-stage telescopic crossbars, and three times the official hip spacing (±341 mm).

Terrain is a single type: the Unitree [`AddPerlinHeighField`](https://github.com/unitreerobotics/unitree_mujoco) Perlin heightfield. 48 m × 4 m, 1.5625 m base wavelength, 6 octaves, 0.20 m relief. Flat for the first 6 m, blended from 6–10 m, full wrinkles after that.

## Environment

Python 3.12, [uv](https://docs.astral.sh/uv/), MuJoCo 3.13. Headless rendering needs `libosmesa6`.

```bash
uv sync --frozen
uv run python scripts/generate_perlin.py
uv run python -m mujoco.viewer --mjcf models/prototype/scene.xml
uv run python -m mujoco.viewer --mjcf models/wide_car/scene.xml
uv run pytest
```
