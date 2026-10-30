# Astribot S1

腰 4 个关节，两臂各 7 个关节，两臂各装一个 F90C 夹爪。轨迹求解用 OCS2 全身 IK；真机通过 SDK 的 WBC 跟踪 TCP，见 [docs/replay.md](../../../docs/replay.md#astribot-s1)。

## 模型

`urdf/s1_with_gripper.urdf` 来自 mindon_robotics 的 `assets/resource/robot/s1/urdf/serial_s1_with_gripper.urdf`（由 `astribot_s1_whole_body_with_wheel.xacro` 生成，轮子和夹爪手指都是固定关节），改了三处：

1. 网格改成相对 URDF 的路径，只拷贝了实际引用的 32 个文件（腰、臂、头、轮子、F90C 夹爪和法兰），放在 `meshes/` 下，走 LFS。
2. 腰和臂的关节限位取三份模型的交集：本模型、SDK 的 `astribot_whole_body.urdf`、SDK 的单体 `astribot_torso/arm_left/arm_right.urdf`。速度限值也取三者中的最小值；腰按单体 URDF 取 1.8 rad/s。

   | 关节 | 下限 | 上限 | 说明 |
   |---|---|---|---|
   | torso 1 / 2 / 3 / 4 | 0 / −2.3 / 0 / **−1.2** | 1.4 / 0 / 2.3 / **1.2** | whole_body 里 joint 4 是 ±1.5，比单体宽，取 ±1.2 |
   | 臂 1 / 2 / 4 / 5 / 6 / 7 | −3 / −1.4 / 0 / −2 / −0.6 / −1.4 | 3 / 0.4 / 2.4 / 2 / 0.6 / 1.4 | 两臂相同 |
   | 左臂 3 / 右臂 3 | −3 / −1.4 | 1.4 / 3.0 | |

3. 新增根 `benchmark_base`，就是轨迹的 BASE。它通过固定关节 `benchmark_base_to_torso_base` 连到原根 `astribot_torso_base`，平移 `(0.10286, 0, −0.13620)` m，不旋转。这样在 `q_pd` 下，两个肩关节（`astribot_arm_*_joint_1`）的中点落在 `(0.1, 0, 1.121)`：先和 Marvin 的肩中点 `(0, 0, 1.121)` 对齐（测试要求第 2 节），再把机器人沿 x 前移 0.1 m，也就是在机器人看来，轨迹在 x 方向回缩了 10 cm。S1 两肩间距 0.30 m（Marvin 是 0.423 m），用一个刚体变换只能对齐中点。要调整放置，只改这个平移：solve、真机换算、dry-run 的伪机器人和 3D 回放都从这里读。

TCP 是 `{left,right}_gripper_f90c_end_effector_link`，它和 Marvin 的 TCP 定义相同：夹爪 base 前方 0.20226 m，x 轴就是接近方向。所以轨迹的姿态可以直接用，不需要再加旋转偏置。

## 交付说明里的 `T_yours_ours`

`astribot_torso_base` 原点在 BASE 下是 `(0.10286, 0, −0.13620)` m，反过来 BASE 原点在 `astribot_torso_base` 下是 `(−0.10286, 0, 0.13620)` m。SDK 的 chassis 系又在 `astribot_torso_base` 下方 0.097 m（`astribot_torso.yaml` 的 `weld_to_base_pose`），各系姿态相同。所以 BASE 到 chassis 是纯平移：

```
p_chassis = p_BASE + (−0.10286, 0, 0.23320) m，四元数 (0, 0, 0, 1)
```

核对：Marvin 肩中点在 BASE 下高 1.121 m，换到 chassis 下高 1.354 m，等于 S1 肩中点在 `astribot_torso_base` 下的 1.257 m 加 0.097 m；x 方向上 S1 肩中点在 BASE 下是 +0.1 m，所以轨迹离 S1 的肩比离 Marvin 的肩近 10 cm。

这样放的局限：S1 肩宽 0.30 m，比 Marvin 的 0.423 m 窄，所以相对各自一侧的肩，S1 上的轨迹在左右方向各往外偏约 6 cm；S1 臂基座还有 −1.22 rad 的俯仰，和 Marvin 水平朝外的肩关节轴不同。平移量按 home 腰姿计算，如果腰参与运动，肩的位置也会随之变化。

0.097 m 写在 `config/s1.yaml` 的 `sdk_frame.torso_base_in_chassis` 里。真机 connect 阶段会核对它：SDK 上报的末端位置和"实测关节 + 本 URDF"做 FK 的结果相差要在 5 mm 以内。

## 起始姿态与 IK

- `q_pd` 取 SDK `move_to_home` 的姿态：腰 `[0.29, −0.58, 0.29, 0]`，两臂是 home 角。solve 从这里出发，真机的 S0 也移到这里。此时两个 TCP 在 BASE 下约为 `(0.616, ±0.274, 0.815)`。
- `ocs2/task_s1_whole_body.info`：一个 MPC，状态是 18 维的腰、左臂、右臂。去掉头部的 2 个关节，`eeFrames` 里依次是左、右 TCP。模板是 mindon 的 `task_s1_dual_arm_left.info`，时域、代价权重和臂的速度限值都沿用它。腰的速度代价设为臂关节的 10 倍，速度限值 1.0 rad/s。
- `ocs2/controller.yaml`：`default_q_reg` 里腰的权重是 20，臂沿用 Marvin 的 `[5, 5, 5, 1, 1, 1, 1]`。这样腰能起冗余作用，但不会漂走。
- 没有厂家给的加速度限值，所以只统计加速度，不给余量。预检用每个关节的 URDF 速度限值，没有用 Marvin 那个 180 deg/s 的 PD 前馈限值。
- 第一次求解在 `runs/.cppad_cache/s1` 生成 CppAD 库。自碰撞没有开。

case02 单次求解：两臂 TCP 位置误差 RMS 约 3 mm（max 12 mm），腰的速度峰值约 30 deg/s，没有越限。
