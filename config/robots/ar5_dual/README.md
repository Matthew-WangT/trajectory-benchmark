# AR5 双臂模型

入口：`urdf/ar5_dual.urdf`。网格已复制到 `meshes/`，使用相对 URDF 的路径；移动或交付时请保留整个 `ar5_dual` 目录。无需原始 Downloads 文件或 ROS package 搜索路径（加载器需支持相对网格路径）。

## 安装约定

肩部坐标系 `base_link` 位于两个机械臂 base 原点的中点，X 向前、Y 向左、Z 向上，单位为米、弧度。

| 安装关节 | 平移 xyz (m) | 旋转 rpy (rad) |
| --- | --- | --- |
| left_mount_joint | 0, +0.12, 0 | −π/2, 0, 0 |
| right_mount_joint | 0, −0.12, 0 | +π/2, 0, 0 |

两个 base **坐标原点**的距离严格为 240 mm；不表示外壳边缘净间距。原始两臂 +Z 轴分别朝公共 +Y / −Y，原始 +X 轴均朝公共 +X，因此零位双臂向两侧伸直。相对于上一版，左臂绕自身 base +Z 旋转 −90°，右臂绕自身 base +Z 旋转 +90°；在 J1、J3 为零时，两臂 J4 正向弯曲均朝公共 +X（前方），弯曲平面为水平面。安装方向是依据参考图推定，尚未由机械安装图确认。

原始 `world` 和对应固定关节被移除，两条链通过上述固定关节接到肩部 `base_link`，再通过固定关节连接唯一根 `benchmark_base`。原始左右臂名称已有 L/R 区分，因此保留所有机械臂 link/joint 名称，以及关节轴、零位、限位、惯量、视觉和碰撞几何。两臂各 7 个转动关节，总计 14 自由度。

## Marvin F90C 夹爪

左右 AR5 的 `AR5-5_08L-W4C1C5_flan_link` / `AR5-5_08R-W4C1C5_flan_link` 各连接一套来自 `config/robots/marvin/urdf/serial_marvin_with_gripper.urdf` 的转接法兰与 F90C 夹爪。新增固定安装关节为 `left_ar5_to_gripper_flange` / `right_ar5_to_gripper_flange`，xyz 为 `0 0 0`，左侧 rpy 为 `0 0 −π/2`，右侧为 `0 0 +π/2`，即左夹爪绕 AR5 末端法兰 +Z 旋转 −90°、右夹爪旋转 +90°，转接法兰 +Z 与 AR5 末端 +Z 保持对齐。未沿用 Marvin Link7 到法兰的变换，因为 AR5 已有独立的末端法兰坐标系。

保留 Marvin 夹爪的网格、碰撞体、惯量、手指固定姿态与 TCP 变换。删除固定关节上无效的 mimic 标签。夹爪不增加活动自由度，整机仍为 14 自由度，暂不支持开合控制。网格复制到本目录，模型不依赖 Marvin 目录。

新 TCP：`left_gripper_f90c_end_effector_link` / `right_gripper_f90c_end_effector_link`。相对 AR5 末端法兰的位置均为 `(0, 0, 0.22706)` m（转接件 0.0248 m + 夹爪 TCP 0.20226 m）；TCP 相对夹爪 base 的旋转沿用 Marvin 左右各自定义；相对 AR5 法兰的总旋转需在此前乘左侧 `Rz(−π/2)` / 右侧 `Rz(+π/2)`。夹爪内部 TCP，左 rpy 为 `(-1.570796, -1.570796, 0)`，右为 `(1.570796, -1.570796, 0)`。

此处为模型装配，转接法兰的实体孔位及机械兼容性尚未核对。

## 预览与示例姿态

`example_pose.json` 与 `startup.q_pd` 一致，采用 Marvin 启动角并将 J2、J4、J6 取反，J1、J3、J5、J7 不变。左臂为 `[0.40, -0.30, 0.27, 2.43, 0.36, -0.05, 0.12]`，右臂为 `[-0.37, -0.14, -0.36, 2.46, -0.39, -0.18, -0.28]`，单位 rad。该映射依据零位及映射后姿态的公共坐标系关节轴方向核对，所有角度均满足 AR5 限位及 0.02 rad 余量；两者连杆长度不同，不保证相同 TCP 位置。URDF 不保存初始关节角，查看器需加载这组关节值才能显示 `preview.png` 的姿态；默认全零会显示双臂横向展开。

生成实际网格预览及结构校验（需要 numpy、trimesh、matplotlib）：

```sh
python config/robots/ar5_dual/preview.py
python config/robots/ar5_dual/preview.py --zero
```

中间支撑仅为示意视觉：立柱半径 65 mm、长度 690 mm，Z 范围 −750 到 −60 mm；肩部横梁半径 55 mm、长度 240 mm。支撑没有碰撞体和惯量，不能据此做支撑碰撞检测或整机动力学仿真。原始 base 本身也没有惯量。此模型适合固定基座可视化与运动学；真实支撑尺寸、动力学及自碰撞应另行确认。

## OCS2 轨迹求解

```sh
./benchmark.sh --mode solve --case case02_fold_clothes_spd50 --robot ar5_dual --runs 1 --no-view
```

交互菜单会自动显示 `ar5_dual`。去掉 `--no-view` 可在交互终端中启动求解结果的 3D 回放。仅接入轨迹求解，未配置 AR5 真机驱动。

根坐标系 `benchmark_base` 与输入 CSV 的 BASE 对齐；固定关节 `benchmark_base_to_shoulders` 将肩部 `base_link` 放在 `(0, 0, 1.121)` m。左右安装原点为 `(0, ±0.12, 1.121)` m，采用与 Marvin 相同的肩部高度。两臂共用这一固定平移，不修改或缩放输入轨迹。预览脚本仍在肩部中点坐标系绘图；求解、分析和回放则在 benchmark_base 中工作。立柱仍为示意模型，未按该安装高度重建。

`robot.yaml` 定义关节顺序、夹爪 TCP 和起始姿态；左右 `.info` 各冻结另一臂的七个关节，以七维模型求解。初始姿态及默认姿态正则项均取上文按轴方向映射的 Marvin 左右启动角。位置限位读取 URDF，速度约束逐关节采用 URDF 的 `[175, 175, 270, 300, 240, 240, 240]` deg/s。全局命令步长上限为 300 deg/s；每关节 OCS2 速度约束是软约束，实际超限由结果分析报告。

MPC 代价、时域和时序以 Marvin 配置为起点；两臂关节速度代价 `inputCost.R.arm.scaling` 已从 0.1 调整为 0.01，以减轻轨迹峰谷被削平的问题。case02 单次对照中，左右位置 RMS 从约 15.62 / 15.55 mm 降至 6.04 / 6.16 mm；其他用例与重复运行尚待验证。保留 0.2 s lookahead（与严格无前瞻测试规则存在偏差）。自碰撞约束未开启。CppAD 缓存独立放在 `runs/.cppad_cache/ar5_dual`。没有厂家加速度限值，因此不声明加速度限值或余量，只统计加速度。175 deg/s 的预检阈值仅为保守分析阈值，不代表已具备真机执行条件。
