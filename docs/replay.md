# Marvin replay 运行手册

在 Marvin 双臂上按 [test_requirements.md](test_requirements.md) 回放 `data/` 里的用例，录制真机反馈，产出第 4 节规定的交付 CSV 和评价报告。仓库自带运行所需的全部代码，不依赖其他仓库。

## 控制链路

```
eef_command.csv ──> OCS2 滚动时域 IK ──q_cmd──> 关节阻抗 (SDK) ──> Marvin
                    500 Hz 评估              500 Hz 下发
```

- **IK**：`src/mpc/` 移植自参考管线的 `controller::DualArmOcs2IkController`，参数和 task `.info` 都是原样拷贝（`config/ocs2/`）。后台线程持续跑 MPC，控制线程每 2 ms `Evaluate` 一次。
- **参考轨迹**（`mpc.reference_mode`）：
  - `lookahead`（默认，与参考管线一致）：每当有 waypoint 到达或进入窗口时，参考设为"当前命令的 FK 位姿（now）+ 未来 `reference_window_s`（默认 0.2 s，即 MPC 视界）内到期的所有 waypoint"，相当于线上 VLA 交给 MPC 的 chunk。**这违反了测试要求第 3 节第 3 条（不得前瞻），交付"说明"里必须写明。**
  - `latest`：严格因果，只给最新到达的那个点，放在 `now + reference_lead_s` 处并保持。
  - case02 dry-run 的 TCP 误差 RMS：lookahead 约 17 mm（等效时延 7 ms）；latest 在 lead 取 0.2 s 时约 92 mm（185 ms），取 5 ms 时约 57 mm（95 ms）。lookahead 下剩下约 16 mm 是 IK 本身的误差，不是时延（`default_q` 正则、输入代价 R、每次求解只迭代 1 次）。
- **预热**：进入阻抗模式之前，MPC 先对着静止目标跑 `mpc.warm_up_s`（1 s），不向机器人下发命令。如果不预热，首次运行的 S1 起步阶段会出现 75–420 deg/s 的关节命令瞬态。
- **下发**：1 ms 的 SCHED_FIFO 循环，每个 tick 读一次反馈；`q_cmd` 每 `control.send_every_ticks` 个 tick 发一次（默认 2，即 500 Hz，和 PD 速度前馈的 2 ms step 一致）。在 infer-10 上实测：按 1 kHz 发送时，进入阻抗模式后的前几秒控制器每秒只收到约 920 帧，命令在接收端排起队，晚约 270 ms 才到达手臂，而且之后不会消退；按 500 Hz 发送时，控制器 3 ms 内收到，下发到实测运动约 7–10 ms。
- **控制模式**：只用关节阻抗。kp/kd、vel/acc ratio、速度前馈、工具参数都在 `config/marvin.yaml` 里，左右臂分别配置。

每次运行的时间轴：

| 段 | 内容 |
|---|---|
| S0 | 关节空间 min-jerk（五次多项式）插值，从当前位置到 `startup.q_pd`，不经过 IK |
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
- 真机运行前，必须已有同一用例的 dry-run 结果，且 IK 输出的关节速度峰值低于 180 deg/s（速度前馈的前提）。

运行中每个 tick 检查：跟踪误差、命令步长、URDF 关节限位（扣除余量）、反馈超时、IK 连续 hold，以及命令传输延迟：把控制器收到的命令（`RT_IN m_Joint_CMD_Pos`）和最近发出的命令做比对，命令在运动时如果连续 50 帧都晚于 30 ms，就 abort。注意，阻抗模式下 `FB_Joint_Cmd` 读到的是实测位置，不是命令。任何一项触发（包括 Ctrl-C），都会立即停止更新命令并下使能，抱闸锁住手臂。数据照常落盘，`run_info.json` 里标记为 `aborted` 并记录原因。

## 步骤

所有命令都在容器里执行：

```bash
docker/run.sh                  # 第一次会构建镜像（pinocchio 3.7.0 + hpp-fcl 2.4.5 源码编译）
docker/build.sh                # 编译到 build/
ctest --test-dir build         # 单元测试；test_mpc_smoke 第一次运行会生成 CppAD 库，要几分钟
```

1. **dry-run**：不连真机，`FakeArm` 用 10 ms 一阶滞后来模拟。用来验证整条链路，并测出关节速度峰值：

   ```bash
   build/bin/replay --config config/marvin.yaml --case data/case02_fold_clothes_spd50 --dry-run
   ```

   CppAD 库缓存在 `runs/.cppad_cache`，只有第一次需要生成。

2. **真机，分步上机**：

   ```bash
   B="build/bin/replay --config config/marvin.yaml --case data/case02_fold_clothes_spd50"
   $B --stop-after connect      # 只做连接、robot.ini 检查和读反馈
   $B --stop-after S0           # 只做 S0 加保持
   $B --runs 1                  # 跑一次完整用例
   $B                           # 连续 3 次
   ```

   输出在 `runs/<case>/real_<时间戳>/run_<i>/`，包括 `raw_cmd.csv`（500 Hz）、`raw_send.csv`（1 kHz）、`raw_fb.csv`（每个新反馈帧一行）、`run_info.json`，还有 `robot.ini` 的副本。

3. **交付 CSV 和评价**：

   ```bash
   S=runs/case02_fold_clothes_spd50/real_<时间戳>
   for r in $S/run_*; do
     python3 scripts/make_delivery_csv.py $r && python3 scripts/evaluate.py $r
   done
   python3 scripts/summarize.py $S
   ```

   外部公司交回的 CSV 也用同一个脚本评价（没有 URDF，依赖模型的阶段会自动跳过）：

   ```bash
   python3 scripts/evaluate.py their.csv --reference data/case02_fold_clothes_spd50/eef_command.csv
   ```

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
