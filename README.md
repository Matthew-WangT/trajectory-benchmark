# EEF Trajectory Benchmark

双臂末端轨迹跟踪测试数据。测试规则见 [docs/test_requirements.md](docs/test_requirements.md)。

**测试只需要 `eef_command.csv` 和 `meta.json`**，其余文件用于核对原始数据，可以忽略。

```
data/case01_fold_clothes_spd45/   叠衣服，46.5 s
  eef_command.csv   被测轨迹
  meta.json         时长与两臂起始位姿
  bag/              原始 ROS 2 bag（可选）
data/case02_fold_clothes_spd50/   叠衣服，13.0 s，文件同上
msg/                bag 的消息定义（可选）
scripts/bag_to_csv.py   bag → CSV（可选；pip install "rosbags>=0.11" numpy）
```

## Marvin 基线复现与评价

Marvin 上的回放程序（OCS2 滚动时域 IK + 关节阻抗）、交付 CSV 生成和评价脚本都在本仓库里。执行 `./benchmark.sh`，在菜单里选择模式、用例和配置，之后会自动完成 replay、评价，并打印结果，见 [docs/replay.md](docs/replay.md)。"轨迹求解"模式只跑 IK，不接机械臂，用来分析一条轨迹在某台机器人上的求解误差、关节限位余量，以及速度和加速度分布；真机运行前的预检也用它。`scripts/evaluate.py` 也能直接评价按第 4 节格式交回的任何 CSV。

```
src/          replay 程序：mpc/（移植的 IK controller）、arm/（Marvin SDK / dry-run）、replay/（时间轴、安全、录制）
src/s1/       Astribot S1 真机：与 SDK sidecar 的 UDP 协议、运行循环、安全检查
robots/       每台机器人一个目录：robot.yaml（关节、TCP、加速度限值、IK 与起始参数）、urdf/、meshes/、ocs2/（IK controller 与 task .info）
config/       marvin.yaml（真机：IP、增益、工具、安全参数，引用 robots/marvin）、s1.yaml（S1 真机：sidecar、坐标、安全开关）
third_party/  Marvin 控制 SDK（Apache-2.0）、OCS2 子集（BSD-3，加了多末端）、astribot_sdk（submodule，BSD-3，只有 S1 真机需要）
docker/       主镜像；s1_bridge/ 是 S1 SDK sidecar 的镜像（ROS 2 Humble）
python/       trackeval 评价包
benchmark.sh  唯一入口：构建 + 菜单 + replay + 评价 + 打印
scripts/      benchmark.py（菜单与流程）、analyze_solve.py、view_solve.py（3D 回放）、make_delivery_csv.py、evaluate.py、summarize.py、s1_bridge.py（S1 SDK sidecar）
```

| 用例 | 线速度 p99 / max（左，右） | 角速度 p99 / max（左，右） | 两臂最小间距 |
|---|---|---|---|
| case01 | 1.26 / 1.56，1.22 / 1.44 m/s | 2.30 / 3.03，2.22 / 2.69 rad/s | 240 mm |
| case02 | 1.31 / 1.55，1.25 / 1.34 m/s | 2.57 / 2.77，2.70 / 3.08 rad/s | 253 mm |

### AR5 双臂轨迹求解

已接入带 Marvin F90C 夹爪的 `ar5_dual`（base 间距 240 mm，肩部高度 1.121 m）。运行：

```sh
./benchmark.sh --mode solve --case case02_fold_clothes_spd50 --robot ar5_dual --runs 1 --no-view
```

安装坐标、OCS2 参数和约束说明见 [robots/ar5_dual/README.md](robots/ar5_dual/README.md)。

### Astribot S1（带腰）

`robots/s1` 有 4 个腰关节，两臂各 7 个关节，装 F90C 夹爪。轨迹求解用 OCS2 全身 IK（一个 MPC，18 个关节）；真机通过 SDK 的 WBC 流式跟踪 TCP：

```sh
./benchmark.sh --mode solve --case case02_fold_clothes_spd50 --robot s1 --no-view
./benchmark.sh --mode dry --case case02_fold_clothes_spd50 --config s1.yaml --runs 1 -y   # sidecar 仿真机器人
./benchmark.sh --mode real --case case02_fold_clothes_spd50 --config s1.yaml --stage connect
```

链路、协议、安全开关见 [docs/replay.md](docs/replay.md#astribot-s1)；模型和坐标约定见 [robots/s1/README.md](robots/s1/README.md)。
