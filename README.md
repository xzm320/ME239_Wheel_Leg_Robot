# 高速轮腿机器人与崎岖路面

轮腿在平地可以平衡和行走，但**无法在高速下通过崎岖路段**。本仓库用同一条 Unitree Perlin 高度场对比两种机械，并用同一套 PID 把这件事测出来。

侧翻的直接原因是：车轮碰到 bump 时，车身**无法持续提供向下的正压力**。轮端一跳，法向力断掉，横滚/俯仰失去约束，机体就翻。

可能的原因有两条，需要分开验证：

1. **机械**：关节柔性不够，扰动不能被被动吸收，机构本身有缺陷。
2. **控制**：即使用了更柔的腿，高速下控制器也来不及把正压力补回去。

为了控制变量，两台机器共用底盘包、轮距、10 寸轮毂和站立高度，只改三条机构。质量按目录件计算，**不做配平**。完整表见 `models/PHYSICS.md`。

## 保留的模型

- **原型机** `models/prototype/`：`wide_car` 的削减版。髋座焊死、串髋串膝两连杆、没有伸缩杆。2 连杆的膝必须有电机，所以各加一颗 qdd100。
- **三倍轮距机** `models/wide_car/`：髋 x–z 电机导轨、菱形四连杆、两级 500 N 伸缩杆。轮距 ±341 mm。

| | 原型机 | wide_car |
| --- | --- | --- |
| **总质量** | **13.168 kg** | **19.374 kg** |
| 髋距 | ±0.3411 m | ±0.3411 m |
| 轮 | 10 寸轮毂 3.00 kg，±20 N·m | 相同 |
| 站立高度 | 0.408 m | 0.408 m |
| 髋座 | 焊死支架 0.15 kg ×2 | MGN12+NEMA17 模组 1.80 kg ×2 |
| 腿 | 2 连杆 + 膝 qdd100 | 菱形四连杆（膝被动） |
| 横杆 | 无 | 500 N 推杆 1.60 kg ×2 |

地形只保留一种：[Unitree `AddPerlinHeighField`](https://github.com/unitreerobotics/unitree_mujoco) 同款 Perlin 高度场。48 m × 4 m，基波长 1.5625 m，6 层，起伏 0.20 m。前 6 m 平地，6–10 m 过渡，之后全幅皱面。

两台机器的轮子都用同一套级联 PID（外环速度 PI → 俯仰参考，内环俯仰 PD → 轮矩）。髋（以及原型机的膝）钉在名义 0。wide_car 关掉伸缩杆位置伺服，只留关节弹簧和髋座滑移吃 bump。不预瞄地形。

判定「皱面上稳住」：不翻、|y| < 1.2 m、横滚 < 12°、到达 x ≥ 10 m，皱面均速不低于目标的 70%。

## 原型机 PID（刚性 2 连杆削减）

下面速度表是在旧的 5.877 kg 玩具质量下测的，**目录件质量改完后尚未重扫**。几何和控制器还在，数字作废。

| 目标 (m/s) | 目标 (km/h) | 结果 | 皱面均速 (km/h) | 说明 |
| --- | --- | --- | --- | --- |
| 0.00 | 0.00 | 站稳（有前爬） | — | 平直段能站住 |
| 0.50 | 1.80 | 稳住 | 3.00 | |
| 1.00 | 3.60 | 稳住 | 4.60 | |
| 1.50 | 5.40 | 稳住 | 5.89 | |
| 2.00 | 7.20 | 稳住 | 7.94 | |
| 2.50 | 9.00 | 稳住 | 9.11 | |
| 2.60 | 9.36 | 稳住 | 9.46 | 俯仰 27° |
| **2.70** | **9.72** | **最高连续稳住** | 9.72 | 横滚 9.4°，俯仰 9.9° |
| 2.80 | 10.08 | 俯仰翻车 | 10.10 | 第一条连续失效 |
| 2.90 | 10.44 | 跑完但横滚 12.7° | 10.27 | 不算稳住 |
| 3.00 | 10.80 | 俯仰翻车 | 10.63 | |
| 3.20 | 11.52 | 偶发稳住 | 11.13 | bump 相位碰巧 |
| 3.50 | 12.60 | 偶发稳住 | 13.24 | 俯仰 31°，贴着阈值 |
| 3.70 | 13.32 | 俯仰翻车 | 9.17 | wide_car 的上限在这里翻 |

**结论：去掉柔性髋、四连杆和伸缩杆之后，同一套 PID 只能连续稳到 2.7 m/s（9.72 km/h）。** wide_car 在同样判据下是 3.7 m/s。完整数字见 `results/prototype_pid_perlin.json`，视频见 `docs/media/`。

## 第 1 版：wide_car 仅 PID、被动吸扰（本分支）

同样，下表是 5.877 kg 玩具质量下的结果，目录件质量改完后需要重扫。

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

**结论：同一套 PID、腿完全被动时，三倍轮距柔顺机可以在皱面上稳到 3.7 m/s（13.3 km/h），比刚性 2 连杆原型机的连续上限 2.7 m/s 高约 1 m/s。** 数字见 `results/wide_car_pid_passive_perlin.json`。

```bash
uv sync --frozen
uv run python scripts/mass_budget.py
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

The two robots share the chassis pack, track, 10-inch hub motors, and standing height. Only three mechanical features differ. Masses are catalog parts; **they are not ballasted to match**. Full bill of materials: `models/PHYSICS.md`.

## Models kept

- **Prototype** `models/prototype/`: an ablation of `wide_car`. Hip mounts are welded, each leg is a serial 2-link chain, and there is no telescopic actuator. A serial knee needs a qdd100, so those two motors are added.
- **3×-track machine** `models/wide_car/`: x–z motorized hip stages, diamond four-bar legs, two-stage 500 N telescopic actuators. Track ±341 mm.

| | Prototype | wide_car |
| --- | --- | --- |
| **Mass** | **13.168 kg** | **19.374 kg** |
| Hip spacing | ±0.3411 m | ±0.3411 m |
| Wheel | 10-inch hub 3.00 kg, ±20 N·m | same |
| Stand height | 0.408 m | 0.408 m |
| Hip mount | welded bracket 0.15 kg ×2 | MGN12+NEMA17 stage 1.80 kg ×2 |
| Leg | 2-link + knee qdd100 | diamond four-bar (passive knee) |
| Crossbar | none | 500 N actuator 1.60 kg ×2 |

Terrain is a single type: the Unitree [`AddPerlinHeighField`](https://github.com/unitreerobotics/unitree_mujoco) Perlin heightfield. 48 m × 4 m, 1.5625 m base wavelength, 6 octaves, 0.20 m relief. Flat for the first 6 m, blended from 6–10 m, full wrinkles after that.

Both robots run the same cascaded PID on the wheels. Hips (and the prototype knees) hold the nominal 0 pose. On `wide_car` the strut position servos are switched off, so only the joint springs and the x–z hip slides eat the bumps. No terrain preview.

A moving run is held if it stays up, |y| < 1.2 m, roll < 12°, reaches x ≥ 10 m, and keeps at least 70% of the commanded speed on the wrinkles.

## Prototype PID (rigid 2-link ablation)

The speed table below was measured at the old 5.877 kg placeholder mass. **It is stale after the catalog-mass update.**

| Target (m/s) | Target (km/h) | Outcome | Rough cruise (km/h) | Notes |
| --- | --- | --- | --- | --- |
| 0.00 | 0.00 | stands (creeps forward) | — | flat-strip balance |
| 0.50 | 1.80 | held | 3.00 | |
| 1.00 | 3.60 | held | 4.60 | |
| 1.50 | 5.40 | held | 5.89 | |
| 2.00 | 7.20 | held | 7.94 | |
| 2.50 | 9.00 | held | 9.11 | |
| 2.60 | 9.36 | held | 9.46 | pitch 27° |
| **2.70** | **9.72** | **max contiguous hold** | 9.72 | roll 9.4°, pitch 9.9° |
| 2.80 | 10.08 | pitch-over | 10.10 | first contiguous failure |
| 2.90 | 10.44 | finished, roll 12.7° | 10.27 | not counted as held |
| 3.00 | 10.80 | pitch-over | 10.63 | |
| 3.20 | 11.52 | isolated hold | 11.13 | lucky bump phase |
| 3.50 | 12.60 | isolated hold | 13.24 | pitch 31°, against the cap |
| 3.70 | 13.32 | pitch-over | 9.17 | this is the wide_car limit |

**Without the compliant hip, four-bar, and telescopic strut, the same PID only holds a contiguous 2.7 m/s (9.72 km/h).** wide_car holds 3.7 m/s on the same criterion. Numbers: `results/prototype_pid_perlin.json`. Clips: `docs/media/`.

## Version 1: wide_car PID only, passive absorption (this branch)

Same caveat: this table is from the 5.877 kg placeholder mass.

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

**With the same PID and fully passive legs, the 3×-track compliant machine holds 3.7 m/s (13.3 km/h) on the wrinkles — about 1 m/s above the rigid 2-link prototype's contiguous 2.7 m/s limit.** Numbers: `results/wide_car_pid_passive_perlin.json`.

```bash
uv sync --frozen
uv run python scripts/mass_budget.py
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
