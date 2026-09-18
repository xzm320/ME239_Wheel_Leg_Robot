# 高速轮腿机器人与崎岖路面

市面上常见的轮腿机器人（以 [Upkie](https://github.com/upkie/upkie) 这类串髋–串膝–轮毂构型为代表）在平地可以平衡和行走，但**无法在高速下通过崎岖路段**。本仓库用同一条 Unitree Perlin 高度场对比两种机械，并用 PID 把这件事测出来。

侧翻的直接原因是：车轮碰到 bump 时，车身**无法持续提供向下的正压力**。轮端一跳，法向力断掉，横滚/俯仰失去约束，机体就翻。

可能的原因有两条，需要分开验证：

1. **机械**：关节柔性不够，扰动不能被被动吸收，机构本身有缺陷。
2. **控制**：即使用了更柔的腿，高速下控制器也来不及把正压力补回去。

## 保留的模型

- **原型机** `models/prototype/`：官方 [Upkie](https://github.com/upkie/upkie)（描述来自 [upkie_description](https://github.com/upkie/upkie_description)）。串髋串膝、轮毂驱动，没有柔性髋座、没有四连杆、没有伸缩杆。
- **三倍轮距机** `models/wide_car/`：在原型构型上改出的对比样机。包含被动柔性髋关节（x–z 滑移）、菱形四连杆、两级套筒伸缩杆，轮距为官方髋距的三倍（±341 mm）。

地形只保留一种：[Unitree `AddPerlinHeighField`](https://github.com/unitreerobotics/unitree_mujoco) 同款 Perlin 高度场。48 m × 4 m，基波长 1.5625 m，6 层，起伏 0.20 m。前 6 m 平地，6–10 m 过渡，之后全幅皱面。

## PID 原型机（`main`）

控制约束为级联 PID：外环速度 PI 给出俯仰参考，内环俯仰 PD 给出轮矩；髋、膝只做 URDF 零位保持（仿真里把 qdd100 位置环加硬到 kp=55，否则串腿会先软塌）。不预瞄地形。

从 x = 3.5 m 平直段起步，若干目标速度扫过去，判定「在皱面上稳住」需要：不翻、|y| < 1.2 m、横滚 < 12°、到达 x ≥ 10 m，并且皱面上均速不低于目标的一半。

| 目标 (m/s) | 目标 (km/h) | 结果 | 皱面均速 (km/h) | 说明 |
| --- | --- | --- | --- | --- |
| 0.00 | 0.00 | 站稳 | — | 平直段原地平衡 |
| 0.10 | 0.36 | 50 s 未翻，未到皱面 | — | 爬得太慢，50 s 只到 x ≈ 7.8 m |
| **0.15** | **0.54** | **皱面上稳住** | 0.38 | 最高可重复的皱面巡航 |
| 0.18 | 0.65 | 俯仰翻车 | 0.60 | 第一道全幅 bump（x ≈ 9.6 m） |
| 0.20 | 0.72 | 俯仰翻车 | 0.59 | 刚进皱面就翻 |
| 0.25 | 0.90 | 横滚翻车 | 0.77 | 同上 |
| 0.30 | 1.08 | 俯仰翻车 | 1.30 | 同上 |
| 0.40 | 1.44 | 横滚翻车 | 1.31 | 同上 |
| 0.50 | 1.80 | 横滚翻车 | 1.36 | 同上 |

**结论：官方串髋串膝 Upkie 用纯 PID，在这条 Perlin 皱面上最快只能稳定到约 0.15 m/s（0.54 km/h）。** 再快，第一道真实 bump 就会把轮端正压力打断。完整数字见 `results/prototype_pid_perlin.json`，视频见 `docs/media/`。

```bash
uv sync --frozen
uv run python scripts/generate_perlin.py
uv run python scripts/prototype_perlin.py
uv run python scripts/render_perlin.py --robot prototype
uv run python -m mujoco.viewer --mjcf models/prototype/scene.xml
uv run pytest
```

## 环境

Python 3.12，[uv](https://docs.astral.sh/uv/)，MuJoCo 3.13。无界面渲染需要 `libosmesa6` 和 `ffmpeg`。

---

# High-speed wheel-legged robots on rough terrain

Typical wheel-legged robots on the market — the serial hip–knee–wheel layout of [Upkie](https://github.com/upkie/upkie) is the usual example — can balance and walk on flat ground, but **cannot traverse rough terrain at high speed**. This repository puts two machines on the same Unitree Perlin strip and measures that limit with PID.

Rollover happens because, when a wheel hits a bump, the chassis **cannot keep a continuous downward (positive) contact force**. The wheel unloads, roll and pitch lose their restoring force, and the body tips over.

Two causes need to be tested separately:

1. **Mechanics**: the joints are not compliant enough to absorb the disturbance passively; the mechanism itself is the defect.
2. **Control**: even with a more compliant leg, the controller cannot restore normal force in time at high speed.

## Models kept

- **Prototype** `models/prototype/`: the official [Upkie](https://github.com/upkie/upkie) (URDF from [upkie_description](https://github.com/upkie/upkie_description)). Serial hip and knee, driven wheels. No compliant hip, no four-bar, no telescopic rod.
- **3×-track machine** `models/wide_car/`: the comparison robot. Passive compliant hips (x–z sliders), diamond four-bar legs, two-stage telescopic crossbars, and three times the official hip spacing (±341 mm).

Terrain is a single type: the Unitree [`AddPerlinHeighField`](https://github.com/unitreerobotics/unitree_mujoco) Perlin heightfield. 48 m × 4 m, 1.5625 m base wavelength, 6 octaves, 0.20 m relief. Flat for the first 6 m, blended from 6–10 m, full wrinkles after that.

## Prototype PID (`main`)

The controller is cascaded PID only: an outer speed PI sets a pitch reference, an inner pitch PD sets wheel torque. Hips and knees hold the URDF zero pose (the qdd100 position loop is stiffened to kp = 55 in simulation, otherwise the serial legs collapse before the wheels can balance). There is no terrain preview.

Episodes launch at x = 3.5 m on the flat strip. A moving run is counted as held on the wrinkles if the robot stays up, |y| < 1.2 m, roll < 12°, reaches x ≥ 10 m, and keeps at least half the commanded speed once it is on the rough.

| Target (m/s) | Target (km/h) | Outcome | Rough cruise (km/h) | Notes |
| --- | --- | --- | --- | --- |
| 0.00 | 0.00 | stands | — | balance on the flat |
| 0.10 | 0.36 | 50 s, never reaches rough | — | too slow; x ≈ 7.8 m after 50 s |
| **0.15** | **0.54** | **held on the wrinkles** | 0.38 | highest repeatable cruise |
| 0.18 | 0.65 | pitch-over | 0.60 | first full bump near x ≈ 9.6 m |
| 0.20 | 0.72 | pitch-over | 0.59 | fails just onto the rough |
| 0.25 | 0.90 | roll-over | 0.77 | same bump |
| 0.30 | 1.08 | pitch-over | 1.30 | same bump |
| 0.40 | 1.44 | roll-over | 1.31 | same bump |
| 0.50 | 1.80 | roll-over | 1.36 | same bump |

**The stock serial Upkie, PID only, holds about 0.15 m/s (0.54 km/h) on this Perlin strip.** Anything faster loses wheel normal force on the first real bump. Numbers: `results/prototype_pid_perlin.json`. Clips: `docs/media/`.

```bash
uv sync --frozen
uv run python scripts/generate_perlin.py
uv run python scripts/prototype_perlin.py
uv run python scripts/render_perlin.py --robot prototype
uv run python -m mujoco.viewer --mjcf models/prototype/scene.xml
uv run pytest
```

## Environment

Python 3.12, [uv](https://docs.astral.sh/uv/), MuJoCo 3.13. Headless rendering needs `libosmesa6` and `ffmpeg`.
