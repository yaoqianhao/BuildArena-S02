---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 04526fdd866a8c5c9aceee6b83ed2c05_b11bbff4bb0911f19ba1525400638852
    ReservedCode1: ttWf7ORxaN77+m+/iFB3sjYh/2dGsjZyKdPsg0qEtduybLuhZZ/zcR6jog5xLtl5be8jLsUu3UPEqqcYqbu6WulhW16tjMU16+vLuU747RaYbWThbmB8Iuq+kziH2f5yVnW6EGR90ToyLUZANa+o5imly0oE5u83bNcW7Z0DaFTc22ozN7Rc4+so1BE=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 04526fdd866a8c5c9aceee6b83ed2c05_b11bbff4bb0911f19ba1525400638852
    ReservedCode2: ttWf7ORxaN77+m+/iFB3sjYh/2dGsjZyKdPsg0qEtduybLuhZZ/zcR6jog5xLtl5be8jLsUu3UPEqqcYqbu6WulhW16tjMU16+vLuU747RaYbWThbmB8Iuq+kziH2f5yVnW6EGR90ToyLUZANa+o5imly0oE5u83bNcW7Z0DaFTc22ozN7Rc4+so1BE=
---

# CONTROL.md — S02 Rocket Orbit-Return Mission Controller

机器：`machine.json`（单级可回收垂直起降火箭，LONE ORB 沙盒）
控制器：`controller.py`（Python，10 Hz 闭环，BAT4 全量遥测）

## 1. 遥测输入（输入通道）

| 输入 | 来源 | 用途 |
|---|---|---|
| 根块位置 / 速度 / 角速度 / 姿态四元数 | BAT4 根块记录（`position/velocity/angular_velocity/rotation`） | 轨道根数、制导、姿态控制 |
| 重力场 | LONE ORB 常量（中心 `(0,-600,0)`、`R_surf=600`、`GM=8456800`、`g≤22`） | 重力补偿、能量制导 |
| 逐块燃料 `fuel` / `fuel_valid` | BAT4 全量 profile | 燃料分数、油量告警 |
| 存活块数 / 机器完整性 | `machine.alive_block_count / machine_integrity` | 碰撞检测、S02 回收门槛 |
| 滑杆 | Booster `fthrust`、反应轮 `speed` | 点火前固定（fthrust=最大值、speed=默认值） |

## 2. 控制输出（输出通道）

| 输出 | 通道 | 用途 |
|---|---|---|
| 引擎 | 4 × Booster `ThrustKey`（Bresenham PWM，10 Hz tick） | 推力脉宽调制（引擎无摆角，fthrust 固定最大） |
| 姿态 | 2 × Reaction Steering Block `LeftKey/RightKey` | 绕机器轴的正/负扭矩（bang-bang + 死区） |
| 分离（仅当机器含 Decoupler） | Decoupler `ExplodeKey`（一次性） | 分级/载荷分离；分离后**禁止**任何推进命令 |

## 3. 任务阶段（mission 模式）

```
pad → guided_ascent → orbit_coast → kill_burn → final_descent → touchdown → settled_hold(10s) → done
（若发现 Decoupler：... orbit_coast → separate → payload_coast → done）
```

| 阶段 | 触发条件 | 控制逻辑 |
|---|---|---|
| pad | t≥0.4s 且重力可用 | 记录地面余隙、初值；锁定东向基准 |
| guided_ascent | 点火后持续 | 速度矢量制导：重力补偿（随 v_tan/v_circ 与远地点衰减）+ 爬升率跟踪 + 切向速度亏缺，倾角变化率限幅 10°/s；插入判定满足 S02 门槛（见下）后转轨 |
| orbit_coast | 入轨判定通过 | 能量反馈制导把**近地点**维持 ≥1120（能量目标 `-GM/(2*peri_target)`，固定目标半径的圆轨道能量，**禁止用 `r+peri_target` 会随半径漂移自激**），累积扫角 ≥1080°（三圈）且近地点保持 ≥1100；近地点跌破 1120 时提前把目标抬回 1150；油量 <0.12 且已扫 ≥360° 时提前返航（`coast_early_exit`） |
| kill_burn | 三圈完成 | **纯逆向制动（retrograde）**：推力仅沿 `-v_tan` 消耗轨道能量实现离轨；姿态对齐门控（cos≥0.85 才全开油门，0.5~0.85 仅 30%，<0.5 关车）防止早期推力沿巡航姿态加速轨道；切向速度 <6 或离地 <6 转末段 |
| final_descent | 上条件 | 高空 ≤10° 倾斜消水平速度；近地纯垂直；自适应积分油门（`land_duty`）跟踪目标下沉率；触地判定：离地 ≤0.6 且径向速度 ≤3 |
| touchdown | 触地 | 引擎全关，速度 ≤0.5 开始累计稳定时长；持续 10s 达标，记录完整性门槛（≥0.90） |
| separate（可选） | 三圈完成且发现 Decoupler | 触发 `ExplodeKey` 一次，等待存活块数下降 ≥5 后转 payload_coast；payload_coast 期间**硬保证零推力** |

**S02 入轨门槛**（任一满足即转 orbit_coast，**所有路径强制近地点 ≥ 1130**）：
- 近地点 ≥ **1130**（= 1100 门槛 + 30 裕量），远地点 ∈ [1040, 1975]（相对地面）；或
- 切向速度 ≥ 当地圆轨道速度 且 |径向速度|<4 且 r≥1100 且 **近地点 ≥ 1130**；或
- v_tan>60 且 |能量-圆轨道能量|<1200 且 |径向速度|<6 且 r≥1100 且 **近地点 ≥ 1130**

> 入轨裕量的原因：orbit_coast 只“维持”入轨时交给它的椭圆；若在近地点 991.6 时通过能量判据入轨（旧 `powered_ok` 不查近地点），`min_coast_peri` 会一直停在 991.6，三圈完成后 `orbit_gate_passed=false`。代价仅为多推进约 1s（离线复算：t=47 → t=48.1s 近地点 991.7 → 1133.2）。

**S02 评分门槛检查**（写入 `mission_summary.json`）：
- `orbit_gate_passed`：min_coast_peri ≥ 1100 且 累计扫角 ≥ 1080°（6π）
- `recovery_ok`：触地后完整性 ≥ 0.90 且稳定着陆保持 ≥ 10s

**逃逸边界（escaped_altitude_bound）说明**：高度 >2200（半径 2800）触发中止。该边界**不是**误判——LONE ORB 有限（surface 600、GM 8456800），高度 2200 处逃逸速度约 78 u/s，失败批次越界时速度约 103 u/s，确系真逃逸（旧 kill_burn 径向分量把能量泵入轨道所致）。修复方向是逆向制动，而非放宽该边界。

## 4. 关键参数（MissionConfig 默认值）

| 参数 | 值 | 含义 |
|---|---|---|
| target_altitude | 550 | 入轨目标半径 1150（地面以上） |
| s02_peri_threshold | 1100 | S02 近地点半径门槛（行星中心距离） |
| s02_peri_margin | 30 | 入轨近地点裕量（入轨要求 peri ≥ 1130） |
| s02_radius_threshold | 1100 | 入轨瞬时半径门槛 |
| orbit_sweep_deg | 1080 | 三圈（6π）累计扫角 |
| raise_target_radius / coast_radius | 1150 / 1120 | 插入近地点 / 巡航维持近地点（能量目标 `-GM/(2*peri_target)`，固定） |
| coast_early_exit_fuel | 0.12 | 油量警戒提前返航 |
| kill_burn_exit_vtan / kv / align_gate | 6.0 / 0.9 / 0.85 | 逆向制动交接切向速度 / 制动增益 / 全油门姿态对齐门 |
| required_integrity | 0.90 | 回收完整性门槛 |
| settled_hold_sec | 10 | 稳定着陆保持时长 |
| thrust_accel / full_thrust_accel | 36 | 全推力比冲（需按实飞校准） |
| att_kp / att_kd / deadband / lookahead | 0.55 / 2.2 / 0.04 / 0.35 | 姿态环 |
| throttle_ff_duty / ke / kvr | 0.30 / 0.6 / 0.03 | 零攻角能量制导 |
| ascent_slew_deg_s | 10 | 制导方向限幅 |
| max_duration | 1800 | 任务总超时（秒） |

## 5. 运行与复现

```powershell
# 1) 预检（重建 machine.bsg、校验控制器可加载、硬件可发现）
uv run python -m control.run_example s02_rocket_orbit_return --prepare-only
# 2) 实机闭环任务（Besiege 已启动、加载好机器、选择 LONE ORB）
uv run python -m control.run_example s02_rocket_orbit_return
# 3) 校准（首飞必做：拟合 thrust_accel 与轮扭矩轴）
$env:HEAVY_LAUNCHER_POLICY = "calibrate"; $env:HEAVY_LAUNCHER_REQUIRE_SUCCESS = "0"
uv run python -m control.run_example s02_rocket_orbit_return
# 查看 $env:RUN_DIR\mission_summary.json 中的 calibration.engines.thrust_accel_mag
# 与 wheel_*_left.axis_unit，如有偏差更新 MissionConfig.thrust_accel 或 WHEEL_CALIBRATION_OVERRIDE
```

无人值守一键复现（需 Besiege 已就绪）：见同目录 `reproduce.ps1`。

## 5.1 复现注意事项（提交包）

- **不要直接运行 `run.py`**：本任务的 `run.py` 以模块方式依赖仓库 `control` 包内的
  `run_example` 运行器，直接执行会报 `ModuleNotFoundError`。
- **正确复现命令**（从仓库根 `E:\BuildArena-2.0` 执行）：

```powershell
uv run python -m control.run_example s02_rocket_orbit_return            # 实机闭环任务
uv run python -m control.run_example s02_rocket_orbit_return --prepare-only   # 仅重建 machine.bsg + 校验
```

- 一键无人值守复现见同目录 `reproduce.ps1`（已自动定位仓库根）。

## 6. 已知假设 / 需实飞确认项

1. **推力模型**：`thrust_accel=36` 来自原示例同一机体重建实测；若本次 S02 重建质量变化，先用 calibrate 策略重测。
2. **轮扭矩轴**：`machine.json` 不含安装翻转信息；控制器默认取"轮偏移方向的反向"为 LeftKey 扭矩轴（wheel A≈绕机器 -x，wheel B≈绕机器 -y）。首飞若姿态发散，用 calibrate 跑出的 `axis_unit` 填入 `WHEEL_CALIBRATION_OVERRIDE`。
3. **分级/载荷分离**：本机为单级、无 Decoupler，分离分支自动关闭；控制器保留 `separate → payload_coast` 分支，机器一旦加装 Decoupler 即自动启用。
4. **气动阻力**：未显式建模，由能量反馈制导与 throttle 前馈隐式抵消；巡航期间可能需少量推进维持 1100 门槛，属预期。
5. **逆向制动姿态转动**：kill_burn 需把机体从巡航姿态转到 `-v_tan`（约 160°），期间油门关闭、靠重力拉出下降弧；失败批次实测轮转速约 0.25 rad/s，预计 8~12s 转到位。若实机转动明显更慢（`align` 长时间 <0.5），可把 `kill_burn_align_gate` 下调（如 0.7）或增大轮 `alpha` 上限。
6. **离轨后燃料预算**：coast 结束剩余油量约 0.71；逆向制动约 2~3s 全推耗 <0.06，末段 hover 自适应消耗剩余油量，预算充足（失败批次 kill_burn 误控 21s 后才耗尽前仍有 0.618）。
*（内容由AI生成，仅供参考）*
