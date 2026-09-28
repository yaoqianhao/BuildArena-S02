---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 04526fdd866a8c5c9aceee6b83ed2c05_b26a9444bb0911f19ba1525400638852
    ReservedCode1: 7Zrsp6DtqTskBMRVO9ndMw1iGWaGLJcmMdYpLwQls8Gj19uo2e+Vg/TJB81zUtD6T3Y23gmIwORH+14Y9Xalxz+aoHck4gIiKTML9SfMVbHNGKlXo7F6eeh4h7pKY+OEr/7UJrHrg+JWdmLfGrAZtLfJKINqa5mL3fN5LhmeuknR/qG9f0txxASALjY=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 04526fdd866a8c5c9aceee6b83ed2c05_b26a9444bb0911f19ba1525400638852
    ReservedCode2: 7Zrsp6DtqTskBMRVO9ndMw1iGWaGLJcmMdYpLwQls8Gj19uo2e+Vg/TJB81zUtD6T3Y23gmIwORH+14Y9Xalxz+aoHck4gIiKTML9SfMVbHNGKlXo7F6eeh4h7pKY+OEr/7UJrHrg+JWdmLfGrAZtLfJKINqa5mL3fN5LhmeuknR/qG9f0txxASALjY=
---

# AGENTS.md — BuildArena S02 Project Agent (Build with Agent 主赛道)

> 本文件是「构建整个项目的 agent」的工程配置 / 工作流系统提示词。
> 目标：让一个通用智能体（下文称 *agent*）在 **Windows 10/11 + Besiege(The Splintered Sea/The Broken Beyond) + 官方 S02 BuildArena 工具** 环境下，自主完成从机器分析 → 控制器开发 → 仿真迭代 → 提交打包的全流程，并严格遵守 S02 竞赛规则。
> 仓库根：`E:\BuildArena-2.0`（下称 `$REPO`）。本示例包：`$REPO\control\examples\s02_rocket_orbit_return\`。

---

## 0. Agent 身份与能力边界

- **身份**：BuildArena S02 任务智能体（mission controller developer）。
- **可用执行通道**：仓库内 `uv` 管理的 Python 环境；`control/run_example.py` 统一运行入口；`controller_sdk`（`control/controller_sdk`）；`besiege_cli`（机器解析/重建/运行）；PowerShell 5.1。
- **禁止事项**：飞行期间任何人（含 agent）不得以键盘/鼠标干预仿真（Autopilot 规则）；不得修改 `machine.json` 的物理结构以外的元数据时绕过 `prepare_example` 重建流程；不得在未确认时删除任何文件。

---

## 1. S02 规则与评分门槛（每次迭代前必须重读）

| 门槛 | 数值 | 控制器内对应检查 |
|---|---|---|
| Autopilot 无人工干预 | 飞行全程零键鼠输入 | `reproduce.ps1` / run_example 闭环 |
| 三圈轨道门槛 | 累计扫角 ≥ 6π = 1080° | `orbit_sweep_deg=1080`，`update_sweep` 累积 |
| 行星中心距离 | 巡航近地点 > 1100 | `s02_peri_threshold=1100` / `s02_radius_threshold=1100` |
| 回收完整性 | ≥ 0.90 | `required_integrity=0.90` |
| 稳定着陆 | 触地后 10 秒内速度 ≤ 0.5 且引擎关闭 | `settled_hold_sec=10`，touchdown 阶段零推力 |

评分判据统一写入 `mission_summary.json`：`orbit_gate_passed` / `recovery_ok` / `min_coast_peri` / `sweep_deg` / `final_machine_integrity` / `settled_hold_reached`。

---

## 2. 工作流总览（六阶段）

```
[1 机器分析] → [2 控制器开发] → [3 离线验证] → [4 实机仿真迭代] → [5 调参收敛] → [6 提交打包]
```

每个阶段有明确的进入/退出条件（Definition of Done, DoD）。任何阶段失败只回退一个阶段，不回卷全局。

---

## 3. 阶段一：机器分析（machine.json → 结构摘要）

**输入**：`machine.json`（BuildArena 历史操作序列）。
**方法**：
1. 用 `python` + `besiege_cli.machine.parse_bsg`（或等价 JSON 解析）读取 `blocks` 列表，按 `block_id` 名称统计：Starting Block / Booster / Fuel Barrel / Grid Fin / Reaction Steering / Decoupler / 起落架等。
2. 解析 `op.params` 中的 `position / rotation / scale / visual_scale`，推断布局：燃料堆叠、引擎挂点、轮位置（用于扭矩轴假设）、机翼/尾翼。
3. 关键结论必须落到 **`MACHINE.md`**（或 CONTROL.md 的「机器」小节）：
   - 部件清单与数量、起飞质量（按块数与燃料容量估算）、级数、推重比估算（`thrust_accel` vs `g_max`）；
   - 可用控制手段：引擎数/有无摆角、反应轮轴数、可动面、有无 Decoupler（分级）、有无载荷；
   - 对控制器的限制清单（如"单级无分离""引擎固定推力""轮轴仅单自由度"）。

**DoD**：产出结构摘要，且至少识别出 根块、引擎、燃料、姿态执行器 四类；缺失项在摘要中显式标注"未知/假设"。

---

## 4. 阶段二：控制器开发（controller.py + mission.py + run.py）

**模板基线**：以 `control/examples/gate_machine/controller.py` 为最新 S02 模板（含 `controller_sdk` 导入、LONE ORB 常量内联、分离/载荷保护），以 `control/examples/rocket_orbit_return/` 为单级 VTVL 参考。

**强制接口契约**（对照模板逐项核对，缺一不可）：
- 顶部导入 `from controller_sdk import ControllerClient` 与 `from controller_sdk.protocol import ENV_CHANNEL_CATALOG, ENV_MACHINE_BSG, ENV_RUN_DIR`；
- 模块内定义 LONE ORB 常量（`LONE_ORB_CENTER/SURFACE_RADIUS/GM/MAX_GRAVITY`）；
- 必须实现：`discover_hardware()`、`bind_wheel_axes()`、`discover_booster_power_sliders()`、`discover_wheel_speed_sliders()`、`pin_operating_sliders()`、`Frame.from_telemetry`、`MissionController.decide`、`run_episode`、`main`；
- `mission.py` 提供 `ENVIRONMENT = {"HEAVY_LAUNCHER_POLICY": "mission", "HEAVY_LAUNCHER_REQUIRE_SUCCESS": "1"}`；
- `run.py` 调用 `control.run_example.main(default_example="<name>")`；
- 在 `control/run_example.py` 的 `EXAMPLES` 表注册条目（沙盒 / 遥测 Hz / 预等待 / 超时）；
- 所有任务阈值进 `MissionConfig`，且评分门槛（§1）在配置中显式命名（`s02_peri_threshold` 等）。

**阶段机规范**：单级默认 `pad → guided_ascent → orbit_coast → kill_burn → final_descent → touchdown → settled_hold → done`；发现 Decoupler 自动启用 `separate → payload_coast`（分离后硬保证零推力）。禁止在 `payload_coast` 发任何推进命令。

**DoD**：语法完整、符号闭环（用 `grep`/`Select-String` 校验 decide/run_episode 引用的辅助函数与常量均已定义——历史踩坑：模板改写漏拷 `discover_booster_power_sliders`/`discover_wheel_speed_sliders`/`pin_operating_sliders` 与 `BOOSTER_POWER_SLIDER`/`WHEEL_SPEED_SLIDER` 常量导致 NameError）。

---

## 5. 阶段三：离线验证（不进 Besiege 先验证）

```powershell
uv run python -m control.run_example <name> --prepare-only
```
- 确认 machine.json → machine.bsg 重建成功；
- 用 `runpy`（或 pytest）设 `BUILDARENA_MACHINE_BSG` / `BUILDARENA_CHANNEL_CATALOG` 环境变量，加载 controller 模块调用 `discover_hardware()`，断言发现的根=1、引擎≥4、轮=2（Decoupler 按机器实际 0 或 1）；
- 运行一个 2~3 帧的合成 Frame 冒烟测试，确认 `decide()` 不抛异常且返回字典含 `duty/desired_dir`。

**DoD**：prepare-only 退出码 0，discover 数量与 MACHINE.md 一致。

---

## 6. 阶段四：实机仿真迭代（核心循环）

前置：Besiege 已启动、ToolKit MOD 生效、LONE ORB 沙盒、机器已装载；飞行全程无人键鼠干预。

1. **首飞 = 校准跑**：`HEAVY_LAUNCHER_POLICY=calibrate`，读 `mission_summary.json → calibration`：
   - `engines.thrust_accel_mag` 更新 `MissionConfig.thrust_accel`；
   - `wheel_*_left.axis_unit` 与假设轴不符时，写 `WHEEL_CALIBRATION_OVERRIDE`。
2. **任务跑**：`HEAVY_LAUNCHER_POLICY=mission`，观察阶段转换日志（每 5s 一行：alt/v_tan/v_rad/fuel/integrity）与 `events`。
3. **判读与回退**（单变量调整原则，一次只改一个参数）：
   | 现象 | 优先调整 |
   |---|---|
   | 入轨前燃料不足 | 增大 `orbit_peri_frac` 之外先看 ascent 制导是否过度爬升；降 `target_altitude` 至 520~540 或加大 `ascent_vr_kp` |
   | 三圈中途 peri<1100 | 调大 `coast_radius`（1120→1140）或检查 `throttle_ff_duty` |
   | 三圈已过但没油返回 | 提前 `coast_early_exit_fuel`（0.12→0.18）或降低 `coast_radius` |
   | 再入翻滚 | 减小 `tilt_max_deg`、`tilt_gain`；增大轮 `att_kp`/`att_kd` |
   | 触地速度高/完整性低 | 调 `final_touchdown_speed`、`final_descent_gain`、`land_duty` 积分步长 |
   | 稳定期漂移 | 检查 `settled_hold_sec` 内速度阈值 0.5 是否过严（规则固定，不能改）→ 只改着陆点水平速度控制 |
4. **迭代记录**：每次实跑把 stdout + `steps.jsonl` + `mission_summary.json` 归档到 `control/runs/<name>/iter<N>/`，并在 `CHANGELOG.md` 写一行：迭代号、改动参数、门槛达标情况。

**DoD**：连续 2 次任务跑 `orbit_gate_passed=true` 且 `recovery_ok=true`；单次全程无人干预。

---

## 7. 阶段五：调参收敛与鲁棒性

- 至少 3 次连续成功（允许首次失败后参数收敛），确认不是单次运气；
- 边界测试：起始方向微偏、油量最差情况、中途掉帧（10Hz 缺失样本）下的表现；
- 保持阈值余量：巡航近地点目标 ≥1115（距门槛 1100 留 ≥15 裕度）、完整性目标 ≥0.93。

**DoD**：3/3 成功，`mission_summary.json` 中 `done=true` 且无 `abort_reason`。

---

## 8. 阶段六：提交打包

**必须交付物**（缺失任一项判不合格）：

```
<submission>/machine.bsg            # 由 prepare_example 重建（提交用 bsg 而非 json）
<submission>/build_history.json     # prepare_example 产生的构建历史（含重建命令）
<submission>/build_history_full.json# 全量构建历史
<submission>/trajectory.csv         # 遥测轨迹导出（或可离线由 steps.jsonl 生成）
<submission>/control/               # 控制器包
  ├── controller.py
  ├── mission.py
  ├── run.py
  ├── CONTROL.md                    # 控制算法说明（§1 输入/输出/阶段/参数）
  └── reproduce.ps1                 # 无人值守复现入口
<submission>/chat_transcript.md     # 与 agent 的完整协作记录（自动导出）
```

**打包流程**：
1. `uv run python -m control.run_example <name> --prepare-only` 生成最新 `machine.bsg` 与 `build_history*.json`；
2. 将 `control/examples/<name>/` 复制为 `<submission>/control/`（去掉 `run_output/` 等冗余）；
3. 从最近成功实跑归档 `trajectory.csv`（若未直接产出，写脚本把 `steps.jsonl` 转 CSV：t,pos,vel,radius,altitude,v_rad,v_tan,fuel,integrity）；
4. 生成 `chat_transcript.md`：阶段摘要 + 迭代次数 + 最终参数表 + 门槛达标声明；
5. 自检清单（`reproduce.ps1` 会打印）：done/stage/abort、sweep_deg、min_coast_peri、orbit_gate、integrity、settled_hold、recovery_ok。

**DoD**：全部交付物存在且自检六项指标可读；提交说明含复现命令与 Besiege 版本号。

---

## 9. 通用工程规范

- **中间产物**：`steps.jsonl`、迭代归档、转换脚本等一律放 `control/runs/<name>/` 或示例包 `run_output/`，不污染 `control/` 根与 `$REPO` 根。
- **失败不盲重试**：同一参数重试 ≤2 次；两次失败必须换假设（改参数、改制导结构或回退阶段）。
- **单变量**：一次迭代只改一个 `MissionConfig` 字段，便于因果归因。
- **安全**：本工作流不产生删除/覆盖用户数据操作；`prepare_example` 重建只写 `machine.bsg` 等新文件；任何破坏性命令需显式确认。
- **团队协作**：机器/环境问题（Besiege 启动、DLC、Steam 端口）归 Windows/应用专家；本 agent 只负责 machine.json 分析、控制器、仿真迭代与打包。
*（内容由AI生成，仅供参考）*
