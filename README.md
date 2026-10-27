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

Marvin 上的回放程序（OCS2 滚动时域 IK + 关节阻抗）、交付 CSV 生成和评价脚本都在本仓库里。执行 `./benchmark.sh`，在菜单里选择模式、用例和配置，之后会自动完成 replay、评价，并打印结果，见 [docs/replay.md](docs/replay.md)。`scripts/evaluate.py` 也能直接评价按第 4 节格式交回的任何 CSV。

```
src/          replay 程序：mpc/（移植的 IK controller）、arm/（Marvin SDK / dry-run）、replay/（时间轴、安全、录制）
config/       marvin.yaml（增益、工具、起始与安全参数）、ocs2/（IK controller 与 task .info）
third_party/  Marvin 控制 SDK（Apache-2.0）、OCS2 子集（BSD-3）
python/       trackeval 评价包
benchmark.sh  唯一入口：构建 + 菜单 + replay + 评价 + 打印
scripts/      benchmark.py（菜单与流程）、make_delivery_csv.py、evaluate.py、summarize.py
```

| 用例 | 线速度 p99 / max（左，右） | 角速度 p99 / max（左，右） | 两臂最小间距 |
|---|---|---|---|
| case01 | 1.26 / 1.56，1.22 / 1.44 m/s | 2.30 / 3.03，2.22 / 2.69 rad/s | 240 mm |
| case02 | 1.31 / 1.55，1.25 / 1.34 m/s | 2.57 / 2.77，2.70 / 3.08 rad/s | 253 mm |
