# 高速轮腿机器人与崎岖路面

轮腿在平地可以平衡和行走，但**无法在高速下通过崎岖路段**。本仓库用同一条 Unitree Perlin 高度场对比两种机械，并用同一套 PID 把这件事测出来。

侧翻的直接原因是：车轮碰到 bump 时，车身**无法持续提供向下的正压力**。轮端一跳，法向力断掉，横滚/俯仰失去约束，机体就翻。

可能的原因有两条，需要分开验证：

1. **机械**：关节柔性不够，扰动不能被被动吸收，机构本身有缺陷。
2. **控制**：即使用了更柔的腿，高速下控制器也来不及把正压力补回去。

为了控制变量，两台机器共用底盘、轮距、车轮和站立高度，只改三条机构。

## 保留的模型

- **原型机** `models/prototype/`：`wide_car` 的削减版。髋座焊死（无 x–z 滑移）、每条腿是串髋–串膝两连杆、没有伸缩横杆。质量、轮距、轮半径和站立高度与三倍机相同。
- **三倍轮距机** `models/wide_car/`：被动柔性髋关节（x–z 滑移）、菱形四连杆、两级套筒伸缩杆。轮距 ±341 mm。

| | 原型机 | wide_car |
| --- | --- | --- |
| 质量 | 5.877 kg | 5.877 kg |
| 髋距 | ±0.3411 m | ±0.3411 m |
| 轮半径 / 轮矩 | 0.120 m / ±6 N·m | 0.120 m / ±6 N·m |
| 站立高度 | 0.408 m | 0.408 m |
| 髋座 | 焊死 | x–z 滑移弹簧 |
| 腿 | 2 连杆（髋、膝伺服钉在 0） | 菱形四连杆 |
| 横杆 | 无 | 两级伸缩，伺服关闭，只留弹簧 |

地形只保留一种：[Unitree `AddPerlinHeighField`](https://github.com/unitreerobotics/unitree_mujoco) 同款 Perlin 高度场。48 m × 4 m，基波长 1.5625 m，6 层，起伏 0.20 m。前 6 m 平地，6–10 m 过渡，之后全幅皱面。

两台机器的轮子都用同一套级联 PID（外环速度 PI → 俯仰参考，内环俯仰 PD → 轮矩）。髋（以及原型机的膝）钉在名义 0。wide_car 关掉伸缩杆位置伺服，只留关节弹簧和髋座滑移吃 bump。不预瞄地形。

判定「皱面上稳住」：不翻、|y| < 1.2 m、横滚 < 12°、到达 x ≥ 10 m，皱面均速不低于目标的 70%。

## 原型机 PID（刚性 2 连杆削减）

从 x = 3.5 m 平直段起步。数字见 `results/prototype_pid_perlin.json`，视频见 `docs/media/`。重新扫描后填表。

## 第 1 版：wide_car 仅 PID、被动吸扰（本分支）

| 目标 (m/s) | 目标 (km/h) | 结果 | 皱面均速 (km/h) | 说明 |
| --- | --- | --- | --- | --- |
| 0.00 | 0.00 | 站稳（有前爬） | — | 平直段能站住 |
| 1.00 | 3.60 | 稳住 | 3.78 | 皱面跟踪良好 |
| 2.00 | 7.20 | 稳住 | 6.70 | |
| 3.00 | 10.80 | 稳住 | 9.46 | 俯仰偏大（18°） |
| 3.50 | 12.60 | 稳住 | 10.29 | |
| **3.70** | **13.32** | **最高稳住** | 10.32 | 横滚 8.3° |
| 3.80 | 13.68 | 跑完但横滚 14° | 11.46 | 不算稳住 |
| 4.00 | 14.40 | 俯仰翻车 | 12.17 | |
| 5.00 | 18.00 | 俯仰翻车 | 13.27 | |

**结论：同一套 PID、腿完全被动时，三倍轮距柔顺机可以在皱面上稳到 3.7 m/s（13.3 km/h）。** 数字见 `results/wide_car_pid_passive_perlin.json`。

```bash
uv sync --frozen
uv run python scripts/generate_perlin.py
uv run python scripts/prototype_perlin.py
uv run python scripts/wide_car_perlin.py
uv run python scripts/render_perlin.py --robot prototype
uv run python scripts/render_perlin.py --robot wide_car
uv run python -m mujoco.viewer --mjcf models/prototype/scene.xml
uv run python -m mujoco.viewer --mjcf models/wide_car/scene.xml
uv run pytest
```

## 环境

Python 3.12，[uv](https://docs.astral.sh/uv/)，MuJoCo 3.13。无界面渲染需要 `libosmesa6` 和 `ffmpeg`。

---

# High-speed wheel-legged robots on rough terrain

Wheel-legged machines can balance on flat ground but **cannot traverse rough terrain at high speed**. This repository puts two machines on the same Unitree Perlin strip and measures that limit with the same PID.

Rollover happens because, when a wheel hits a bump, the chassis **cannot keep a continuous downward (positive) contact force**. The wheel unloads, roll and pitch lose their restoring force, and the body tips over.

Two causes need to be tested separately:

1. **Mechanics**: the joints are not compliant enough to absorb the disturbance passively; the mechanism itself is the defect.
2. **Control**: even with a more compliant leg, the controller cannot restore normal force in time at high speed.

The two robots share chassis, track, wheels, and standing height. Only three mechanical features differ.

## Models kept

- **Prototype** `models/prototype/`: an ablation of `wide_car`. Hip mounts are welded (no x–z slides), each leg is a serial 2-link chain, and there is no telescopic crossbar. Mass, track, wheel radius, and stand height match the 3×-track machine.
- **3×-track machine** `models/wide_car/`: passive compliant hips (x–z sliders), diamond four-bar legs, two-stage telescopic crossbars. Track ±341 mm.

| | Prototype | wide_car |
| --- | --- | --- |
| Mass | 5.877 kg | 5.877 kg |
| Hip spacing | ±0.3411 m | ±0.3411 m |
| Wheel radius / torque | 0.120 m / ±6 N·m | 0.120 m / ±6 N·m |
| Stand height | 0.408 m | 0.408 m |
| Hip mount | welded | x–z spring slides |
| Leg | 2-link (hip/knee held at 0) | diamond four-bar |
| Crossbar | none | two-stage telescopic, servos off |

Terrain is a single type: the Unitree [`AddPerlinHeighField`](https://github.com/unitreerobotics/unitree_mujoco) Perlin heightfield. 48 m × 4 m, 1.5625 m base wavelength, 6 octaves, 0.20 m relief. Flat for the first 6 m, blended from 6–10 m, full wrinkles after that.

Both robots run the same cascaded PID on the wheels. Hips (and the prototype knees) hold the nominal 0 pose. On `wide_car` the strut position servos are switched off, so only the joint springs and the x–z hip slides eat the bumps. No terrain preview.

A moving run is held if it stays up, |y| < 1.2 m, roll < 12°, reaches x ≥ 10 m, and keeps at least 70% of the commanded speed on the wrinkles.

## Prototype PID (rigid 2-link ablation)

Episodes launch at x = 3.5 m. Numbers: `results/prototype_pid_perlin.json`. Clips: `docs/media/`. Table filled after the re-sweep.

## Version 1: wide_car PID only, passive absorption (this branch)

| Target (m/s) | Target (km/h) | Outcome | Rough cruise (km/h) | Notes |
| --- | --- | --- | --- | --- |
| 0.00 | 0.00 | stands (creeps forward) | — | flat-strip balance |
| 1.00 | 3.60 | held | 3.78 | tracks well |
| 2.00 | 7.20 | held | 6.70 | |
| 3.00 | 10.80 | held | 9.46 | pitch peaks at 18° |
| 3.50 | 12.60 | held | 10.29 | |
| **3.70** | **13.32** | **max held** | 10.32 | roll 8.3° |
| 3.80 | 13.68 | finished, roll 14° | 11.46 | not counted as held |
| 4.00 | 14.40 | pitch-over | 12.17 | |
| 5.00 | 18.00 | pitch-over | 13.27 | |

**With the same PID and fully passive legs, the 3×-track compliant machine holds 3.7 m/s (13.3 km/h) on the wrinkles.** Numbers: `results/wide_car_pid_passive_perlin.json`.

```bash
uv sync --frozen
uv run python scripts/generate_perlin.py
uv run python scripts/prototype_perlin.py
uv run python scripts/wide_car_perlin.py
uv run python scripts/render_perlin.py --robot prototype
uv run python scripts/render_perlin.py --robot wide_car
uv run python -m mujoco.viewer --mjcf models/prototype/scene.xml
uv run python -m mujoco.viewer --mjcf models/wide_car/scene.xml
uv run pytest
```

## Environment

Python 3.12, [uv](https://docs.astral.sh/uv/), MuJoCo 3.13. Headless rendering needs `libosmesa6` and `ffmpeg`.
