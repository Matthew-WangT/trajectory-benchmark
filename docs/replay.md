# Marvin replay 运行手册

在 Marvin 双臂上按 [test_requirements.md](test_requirements.md) 回放 `data/` 里的用例，录制真机反馈，产出第 4 节规定的交付 CSV 和评价报告。仓库自带运行所需的全部代码，不依赖其他仓库。

## 控制链路

```
eef_command.csv ──> OCS2 滚动时域 IK ──q_cmd──> 关节阻抗 (SDK) ──> Marvin
                    500 Hz 评估              500 Hz 下发
```

- **IK**：`src/mpc/` 移植自参考管线的 `controller::DualArmOcs2IkController`，参数和 task `.info` 都是原样拷贝（`robots/marvin/ocs2/`）。后台线程持续跑 MPC，控制线程每 2 ms `Evaluate` 一次。
- **参考轨迹**（`robots/marvin/robot.yaml` 的 `ik.reference_mode`）：
  - `lookahead`（默认，与参考管线一致）：每当有 waypoint 到达或进入窗口时，参考设为"当前命令的 FK 位姿（now）+ 未来 `reference_window_s`（默认 0.2 s，即 MPC 视界）内到期的所有 waypoint"，相当于线上 VLA 交给 MPC 的 chunk。**这违反了测试要求第 3 节第 3 条（不得前瞻），交付"说明"里必须写明。**
  - `latest`：严格因果，只给最新到达的那个点，放在 `now + reference_lead_s` 处并保持。
  - case02 dry-run 的 TCP 误差 RMS：lookahead 约 17 mm（等效时延 7 ms）；latest 在 lead 取 0.2 s 时约 92 mm（185 ms），取 5 ms 时约 57 mm（95 ms）。lookahead 下剩下约 16 mm 是 IK 本身的误差，不是时延（`default_q` 正则、输入代价 R、每次求解只迭代 1 次）。
- **预热**：进入阻抗模式之前，MPC 先对着静止目标跑 `ik.warm_up_s`（1 s），不向机器人下发命令。如果不预热，首次运行的 S1 起步阶段会出现 75–420 deg/s 的关节命令瞬态。
- **下发**：1 ms 的 SCHED_FIFO 循环，每个 tick 读一次反馈；`q_cmd` 每 `ik.send_every_ticks` 个 tick 发一次（默认 2，即 500 Hz，和 PD 速度前馈的 2 ms step 一致）。在 infer-10 上实测：按 1 kHz 发送时，进入阻抗模式后的前几秒控制器每秒只收到约 920 帧，命令在接收端排起队，晚约 270 ms 才到达手臂，而且之后不会消退；按 500 Hz 发送时，控制器 3 ms 内收到，下发到实测运动约 7–10 ms。
- **控制模式**：只用关节阻抗。kp/kd、vel/acc ratio、速度前馈、工具参数都在 `config/marvin.yaml` 里，左右臂分别配置。
- **配置拆分**：机器人本体放在 `robots/<name>/`，包括 `robot.yaml`（URDF、关节名、TCP frame、关节加速度限值、`ik:` 段、`startup:` 段即 `q_pd` 与各段时长、`solve:` 段）、`urdf/` 和 `ocs2/`，轨迹求解和 replay 共用这一份。`config/marvin.yaml` 只放真机相关的内容，并通过 `robot_dir` 引用机器人目录。

每次运行的时间轴：

| 段 | 内容 |
|---|---|
| S0 | 关节空间 min-jerk（五次多项式）插值，从当前位置到 `robot.yaml` 的 `startup.q_pd`，不经过 IK |
| S1 | 用 `q_cmd` 重置 IK；TCP 从 FK(q_pd) 走直线 + slerp、min-jerk 时间缩放，到 `meta.json` 起始位姿，经过 IK |
| S2 | 在起始位姿静止 2 s |
| S3 | 按 `t_s` 回放轨迹，`t_s = 0` 就是交付 CSV 的零点 |
| S4 | 在末位姿保持 2 s |

同一个进程连续执行 `--runs` 次（默认 3 次），每次都从 S0 开始。任何一次 abort，后面的都不再执行。

## 安全

启动前检查：
- 两臂 TCP 的最小间距（按参考轨迹算）不能小于 `safety.min_tcp_distance_m`。
- 从控制器拉取 `robot.ini`，两臂都必须满足 `PosDTType=1` 和 `JointPIDCtlType=1`，不满足就拒绝启动，不会自动修改。
- 当前关节和 `q_pd` 的偏差不能超过 `max_initial_deviation_rad`。
- 真机运行前，必须有同一用例、同一机器人的轨迹求解结果，而且它的输入文件哈希要和当前文件一致。结果要满足：S1–S4 的关节速度峰值低于 `solve.precheck_max_joint_vel_deg_s`（180 deg/s，速度前馈的前提），并且没有位置越限。`./benchmark.sh` 会自动找最新的结果，找不到就先跑一次求解，再通过 `--solve-result` 交给 replay。

运行中每个 tick 检查：跟踪误差、命令步长、URDF 关节限位（扣除余量）、反馈超时、IK 连续 hold，以及命令传输延迟：把控制器收到的命令（`RT_IN m_Joint_CMD_Pos`）和最近发出的命令做比对，命令在运动时如果连续 50 帧都晚于 30 ms，就 abort。注意，阻抗模式下 `FB_Joint_Cmd` 读到的是实测位置，不是命令。任何一项触发（包括 Ctrl-C），都会立即停止更新命令并下使能，抱闸锁住手臂。数据照常落盘，`run_info.json` 里标记为 `aborted` 并记录原因。

## 步骤

只需要一条命令，在宿主机上执行（需要 docker；第一次运行会构建镜像：pinocchio 3.7.0 + hpp-fcl 2.4.5 源码编译）：

```bash
./benchmark.sh
```

它会自动增量编译（日志在 `build/build.log`），然后用方向键菜单依次选择（↑/↓ 或 j/k，Enter 确认，q 退出）：

1. **模式**：真机 / dry-run（不连机器人，`FakeArm` 用 10 ms 一阶滞后模拟）/ 轨迹求解（见下节，之后只再选用例和机器人）
2. **用例**：`data/` 下的所有用例
3. **配置**：`config/*.yaml`（只有一个文件时自动选中）
4. **运行**：完整用例 3 次 / 1 次，或者分步上机的阶段：
   - `connect`：只做连接、`robot.ini` 检查、读反馈
   - `S0`：只做关节插值到 `q_pd`，然后保持
   - `S2`：只跑 S0–S2（慢速移到起始位姿并静止），不跑轨迹

接着打印配置摘要（IP、增益、速度前馈、下发节拍、负载、IK 参考模式）。真机模式下要按 Enter 确认后才开始。之后全部自动完成：

- 真机跑完整用例时，先做轨迹求解预检（见上文"安全"）
- replay：终端只显示关键行，完整日志在 `<session>/replay.log`
- 对每次完成的 run 生成交付 CSV 并评价，最后汇总
- 终端打印结果表：两臂各自的关节误差、关节时延、TCP 误差，取多次运行的均值 [min, max]

Ctrl-C 会让 replay 走 abort 流程（停止更新命令、下使能、抱闸），已完成的 run 照常评价。

输出在 `runs/<case>/<real|dry>_<时间戳>/`：`replay.log`、`summary.txt`；每个 `run_<i>/` 里有 `raw_cmd.csv`（500 Hz）、`raw_send.csv`、`raw_fb.csv`（每个新反馈帧一行，含控制器收到的命令）、`run_info.json`、`delivery.csv`（第 4 节交付格式）、`report/`（`report.txt`、`metrics.json`、图）。

所有菜单项也都可以从命令行直接指定，不再询问：

```bash
./benchmark.sh --mode dry --case case02_fold_clothes_spd50 --runs 3 -y
./benchmark.sh --mode real --case case02_fold_clothes_spd50 --stage S2
./benchmark.sh --mode solve --case case02_fold_clothes_spd50 --robot marvin
```

外部公司交回的 CSV 用同一套评价（没有 URDF，依赖模型的阶段会自动跳过）：

```bash
docker/run.sh python3 scripts/evaluate.py their.csv --reference data/case02_fold_clothes_spd50/eef_command.csv
```

## 轨迹求解

只跑 IK，不接机械臂：看一条轨迹在某台机器人上经过线上同款 OCS2 MPC IK 能解成什么样，包括求解误差、关节限位余量、速度和加速度分布。`./benchmark.sh` 选"轨迹求解"，再选用例和机器人（`robots/` 下的目录）。

- **求解**：`replay --solve --robot robots/<name>` 用的是同一个 1 ms 实时循环、后台 MPC 线程和 lookahead 参考，机器人换成理想跟随（`q_meas = q_cmd`）。从 `q_pd` 出发，S0 是在 `q_pd` 静止 `startup.joint_min_duration_s`，作用和 replay 里 S0 结束时一样：让预热留下的 policy 过期，否则 S1 第一拍会跳。之后是 S1–S4。实时求解每次结果会略有差异；`--runs N` 跑多次时，汇总取每一项的最差值。
- **不中止**：越限只记次数、首次时刻和涉及的关节，曲线上标红。只有命令出现 NaN，或者 IK 连续 hold 超过 `solve.max_consecutive_holds`，才停下并标为 `failed`。
- **指标**：除"违规"一项统计 S1–S4 外，其余都只统计 S3。
  - 求解误差：TCP 目标和 FK(q_cmd) 比较，给出位置 xyz 三轴、模长和姿态角的 rms / p50 / p95 / p99 / max。另外在 −100…200 ms 范围内搜索等效时延，并给出时间对齐后的残差。
  - 余量：
    - 位置余量 = 到 URDF 限位的最小距离，单位 deg，同时给出占行程的百分比；
    - 速度余量 = 1 − max|dq| / URDF velocity；
    - 加速度余量 = 1 − max|ddq| / `max_acc_deg_s2`（URDF 里没有加速度限值，Marvin 取 j1–j2 450、j3–j7 900 °/s²）。
  - 分布：|dq| 和 |ddq| 按关节给出 rms / p50 / p95 / p99 / max。速度用 2 ms 网格上的中心差分。加速度先对 `q_cmd` 做 4 阶 Butterworth 零相位低通（`solve.acc_lowpass_hz`，20 Hz），再做差分；原始二阶差分只报 p99 和 max，它反映的是 MPC policy 切换时的台阶。
  - 违规：位置越过 URDF 限位减去 `joint_limit_margin_rad`；速度超过 URDF 限值；加速度超过 `max_acc`；命令步长超过 `max_joint_velocity_rad_s`；IK hold，记次数和最长连续段。
- **输出**：`runs/<case>/solve_<robot>_<时间戳>/`，包括：
  - `report.txt`：内容和终端打印一致；
  - `metrics.json`：全部指标、违规、真机预检结论（`precheck`），以及输入文件的 sha256；
  - `config_snapshot/`：这次用到的 robot.yaml、URDF、controller.yaml 和 .info；
  - 每个 `run_<i>/` 下有 `raw_cmd.csv`、`solve.csv`（500 Hz，含目标与求解 TCP、q_cmd、dq、ddq）和 `plots/`。`plots/` 里依次是：两臂的 TCP 目标与求解叠画并附误差、关节 q 与限位、速度和加速度曲线与限值、S3 速度和加速度直方图（标出 p50/p95/p99）。
- **3D 回放**：求解完成、结果表打印出来之后，会自动启动 meshcat，并打印一个浏览器地址（`http://127.0.0.1:7000/static/`；容器用的是宿主机网络，所以在宿主机上直接打开即可）。机器人按 `q_cmd` 运动，蓝线是目标 TCP 轨迹，橙线是求解得到的 FK(q_cmd)，小球标出当前时刻的位置。在面板 Animations 里可以播放、暂停、拖时间条、调倍速。按 Enter 关闭。
  - 在远程机器上跑时，先在本地执行 `ssh -L 7000:localhost:7000 <host>`。
  - 以前的求解也能回看：`docker/run.sh python3 scripts/view_solve.py runs/<case>/solve_<robot>_<时间戳> [--run 2]`。
  - 不需要回放时加 `--no-view`。
  - 网格在 `robots/<name>/meshes/`（走 LFS），URDF 里写相对路径。
- **新增一台机器人**（必须是双臂，每臂 7 个关节）：
  1. 把 `robots/marvin` 拷贝成 `robots/<name>`；
  2. 换上自己的 URDF 和网格（网格用相对 URDF 的路径）；
  3. 在 `robot.yaml` 里改关节名、TCP frame、`q_pd`、加速度限值；
  4. 在两份 `.info` 里改 `baseFrame`、`eeFrame` 和 `removeJoints`。

  菜单会自动列出新机器人。第一次求解要生成 CppAD 库，需要几分钟。

## 报告内容

| 阶段 | 内容 | 需要 |
|---|---|---|
| sanity | 采样率、丢帧、命令二阶差分、速度/力矩饱和 | 关节列（饱和需要 URDF） |
| **joint_error** | 各关节 `q_cmd - q_meas` 的 RMS / p95 / max | 关节列 |
| joint | 时延（滑窗互相关、频域相位）、等效时延、误差回归 | 关节列（重力项需要 URDF） |
| tcp | TCP 命令 vs 实测的分轴位置 RMS / p95 / max、姿态误差、等效时延 | 位姿列 |
| ee | FK(q_cmd) - FK(q_meas)、Jacobian 归因、频谱 | URDF |
| margin | 关节限位、速度和力矩余量、cond(J) | URDF |
| health | IK 评估率、hold/限幅、policy age、下发和反馈间隔、丢帧、命令回显一致性 | Marvin raw 记录 |

## "说明"模板（第 4 节第 2 条）

> `T_yours_ours` = 单位变换（Marvin 就是原机，平移 (0, 0, 0) m，四元数 (0, 0, 0, 1)）。实测位姿由编码器关节角经 URDF 正运动学得到（`serial_marvin_with_gripper.urdf`，TCP 为法兰前方 0.227 m 的虚拟点）。控制模式为关节阻抗：kp = [22,22,22,15,8,8,8]，kd = 0.3，控制器开启 PD 速度前馈（step 2 ms）。关节命令由 OCS2 滚动时域 IK 在线生成，500 Hz 评估、500 Hz 下发。负载：F90C 夹爪 / 空载（二选一）。控制器固有的插值和滤波：①**与第 3 节第 3 条不一致**：IK 参考使用未来 0.2 s 内的轨迹点（与线上 VLA 的 chunk 用法一致），属于前瞻；②IK 输出关节增量按 10.5 rad/s 限幅；③起始段 S1 的 TCP 插值不属于被测轨迹。
