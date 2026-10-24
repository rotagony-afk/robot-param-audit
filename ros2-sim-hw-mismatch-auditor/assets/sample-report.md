<!-- 本文件由 rsw-audit v1.0.0 真实运行生成，仅将绝对路径脱敏为相对仓库根的 . 
     生成命令: python scripts/audit.py full --project ../examples/demo_robot --log ../examples/demo_robot/logs/run_straight.csv -->

# 机器人仿真实机参数错配诊断报告

| 项目 | 值 |
|---|---|
| 工具 | rsw-audit v1.0.0 |
| 生成时间 | 2026-10-23 13:46:08 +0800 |
| 工程根目录 | .\examples\demo_robot |
| 执行命令 | `python scripts/audit.py full --project examples/demo_robot --log examples/demo_robot/logs/run_straight.csv` |
| 读取文件数 | 6 |
| 跳过文件数 | 0 |


## 0. 结论摘要

| 级别 | 数量 |
|---|---|
| [严重] CRITICAL | 1 |
| [高] HIGH | 7 |
| [中] MEDIUM | 2 |
| [低] LOW | 1 |
| [信息] INFO | 5 |

**需要先处理的问题（8 项）：**

| 规则 | 级别 | 置信度 | 状态 | 结论 | 首个证据位置 |
|---|---|---|---|---|---|
| S02 | [严重] | HIGH | 待确认 | 疑似单位不一致：最大线速度（比值 1000） | config/legacy_params.yaml:6 |
| C01 | [高] | HIGH | 已确认 | 根因收敛：驱动轮半径 应为 0.0508（当前 0.05） | description/config/demo_robot_params.yaml:7 |
| R01 | [高] | HIGH | 已确认 | 里程计线位移偏差 1.6%（真值 0.995 m / 里程计 0.9793 m） | run_straight.csv:2 |
| S01 | [高] | HIGH | 已确认 | 驱动轮半径 多处定义且数值不一致（2 组） | description/config/demo_robot_params.yaml:7 |
| S04 | [高] | HIGH | 已确认 | 左右轮距与 URDF 轮子安装位置不一致（声明 0.3 m / 几何 0.32 m） | description/config/demo_robot_params.yaml:8 |
| S05 | [高] | HIGH | 已确认 | 轮子 joint 的旋转轴方向不一致 | description/urdf/demo_robot.urdf:82 |
| S06 | [高] | HIGH | 已确认 | 转动惯量三个主轴全为 0 | description/urdf/demo_robot.urdf:13 |
| S07 | [高] | MEDIUM | 已确认 | 控制频率 50 Hz 高于仿真更新频率 20 Hz | description/config/demo_robot_params.yaml:11 |

## 1. 输入清单与指纹

| 文件 | 类型 | 字节 | sha256(前16) |
|---|---|---|---|
| `config/legacy_params.yaml` | yaml | 242 | `3b373364d2e89f77` |
| `description/config/demo_robot_params.yaml` | yaml | 597 | `548c7bbbd25fce02` |
| `description/urdf/demo_robot.urdf` | urdf | 3389 | `5e4899665b55e242` |
| `launch/demo_robot.launch.py` | launch | 743 | `e4be4a41baab8094` |
| `src/diff_drive_controller.cpp` | source | 806 | `444972add3911b0b` |
| `logs/run_straight.csv` | log | 8236 | `e67b3adc2cd35eb6` |

### 运行日志概览

- `logs/run_straight.csv` · 格式 csv · 200 行 · 识别列 cmd_lin, odom_lin, t, truth_x, truth_y

## 2. 静态一致性检查（S 类）

### [严重] S02 · 疑似单位不一致：最大线速度（比值 1000）

置信度 **HIGH** · 状态 **待确认** · 标签 unit, max_linear_vel

最大线速度 在不同来源相差约 1000 倍，最可能是米/毫米/厘米混用。当前取值：yaml 500 m/s @ config/legacy_params.yaml:6 / yaml 0.5 m/s @ description/config/demo_robot_params.yaml:12。只要有一处是按错误单位写入，里程计与限速都会被整体放大或缩小。工具无法判断哪一份配置会被真正加载，因此标记为待确认——请确认这份文件是否仍在编译/启动链路中。

**证据**

- `config/legacy_params.yaml:6`  (yaml legacy_robot.ros__parameters.max_linear_vel)

      max_linear_vel: 500

- `description/config/demo_robot_params.yaml:12`  (yaml demo_robot.ros__parameters.max_linear_vel)

      max_linear_vel: 0.5


**指标**

| 名称 | 值 |
|---|---|
| canonical | max_linear_vel |
| unit_ratio | 1000 |
| values | ["500", "0.5"] |

### [高] S01 · 驱动轮半径 多处定义且数值不一致（2 组）

置信度 **HIGH** · 状态 **已确认** · 标签 conflict, wheel_radius

驱动轮半径 共被读取到 7 次，分成 2 个不同的值，最大相对差 1.575%。 当前取值：yaml 0.05 m @ description/config/demo_robot_params.yaml:7 / yaml 0.05 m @ description/config/demo_robot_params.yaml:15 / source 0.05 m @ src/diff_drive_controller.cpp:10 / urdf 0.0508 m @ description/urdf/demo_robot.urdf:38 / urdf 0.0508 m @ description/urdf/demo_robot.urdf:44 / urdf 0.0508 m @ description/urdf/demo_robot.urdf:58 / urdf 0.0508 m @ description/urdf/demo_robot.urdf:64。仅凭静态文件无法判定哪一处对应实物，需结合运行日志反推（见 C 类结论）。

**证据**

- `description/config/demo_robot_params.yaml:7`  (yaml demo_robot.ros__parameters.wheel_radius)

      wheel_radius: 0.05

- `description/config/demo_robot_params.yaml:15`  (yaml demo_robot.ros__parameters.wheel_radius)

      wheel_radius: 0.05

- `description/urdf/demo_robot.urdf:38`  (link left_wheel cylinder radius)

      <cylinder radius="0.0508" length="0.03"/>

- `description/urdf/demo_robot.urdf:44`  (link left_wheel cylinder radius)

      <cylinder radius="0.0508" length="0.03"/>

- `description/urdf/demo_robot.urdf:58`  (link right_wheel cylinder radius)

      <cylinder radius="0.0508" length="0.03"/>

- `description/urdf/demo_robot.urdf:64`  (link right_wheel cylinder radius)

      <cylinder radius="0.0508" length="0.03"/>

- `src/diff_drive_controller.cpp:10`  (cpp_const WHEEL_RADIUS)

      constexpr double WHEEL_RADIUS = 0.05;


**指标**

| 名称 | 值 |
|---|---|
| canonical | wheel_radius |
| rel_diff | 0.015748 |
| values | ["0.05", "0.0508"] |
| sources | ["source", "urdf", "yaml"] |

### [高] S04 · 左右轮距与 URDF 轮子安装位置不一致（声明 0.3 m / 几何 0.32 m）

置信度 **HIGH** · 状态 **已确认** · 标签 conflict, wheel_separation, geometry

配置里 wheel_separation = 0.3 m，但 URDF 中左右轮 joint origin 的 y 分量为 left_wheel_joint=0.16, right_wheel_joint=-0.16，两者相差 6.25%。差速转向角速度 = (v_r - v_l) / wheel_separation，该值偏小会让机器人转身角度偏大。

**证据**

- `description/config/demo_robot_params.yaml:8`  (yaml demo_robot.ros__parameters.wheel_separation)

      wheel_separation: 0.30

- `src/diff_drive_controller.cpp:11`  (cpp_const WHEEL_SEPARATION)

      constexpr double WHEEL_SEPARATION = 0.30;

- `description/urdf/demo_robot.urdf:81`  (joint left_wheel_joint origin)

      <origin xyz="0 0.16 0.0508" rpy="0 0 0"/>

- `description/urdf/demo_robot.urdf:89`  (joint right_wheel_joint origin)

      <origin xyz="0 -0.16 0.0508" rpy="0 0 0"/>


**指标**

| 名称 | 值 |
|---|---|
| declared | 0.3 |
| implied | 0.32 |
| rel_diff | 0.0625 |

### [高] S05 · 轮子 joint 的旋转轴方向不一致

置信度 **HIGH** · 状态 **已确认** · 标签 axis, urdf

差速驱动要求所有驱动轮绕同一条轴旋转，否则同一速度指令下轮子转向不同。当前 URDF 中：left_wheel_joint=[0.0, 0.0, 1.0], right_wheel_joint=[0.0, 1.0, 0.0]。

**证据**

- `description/urdf/demo_robot.urdf:82`  (joint left_wheel_joint axis)

      <axis xyz="0 0 1"/>

- `description/urdf/demo_robot.urdf:90`  (joint right_wheel_joint axis)

      <axis xyz="0 1 0"/>


**指标**

| 名称 | 值 |
|---|---|
| axes | [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0]] |

### [高] S06 · 转动惯量三个主轴全为 0

置信度 **HIGH** · 状态 **已确认** · 标签 inertial, urdf

惯量矩阵全 0 会让物理引擎求逆失败，表现为连杆乱飞或仿真报 singular。至少给 ixx/iyy/izz 一个非零值。

**证据**

- `description/urdf/demo_robot.urdf:13`  (link base_link inertia)

      <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>

### [高] S07 · 控制频率 50 Hz 高于仿真更新频率 20 Hz

置信度 **MEDIUM** · 状态 **已确认** · 标签 timing, rate

控制器每秒下发 50 次指令，而 Gazebo 侧每 20 Hz 才更新一次，频率比 2.5不是整数。多出来的指令会被丢弃或合并，仿真中看到的行为与实际控制器不同，这也常是“仿真里好好的、上实物就抖”的根源之一。

**证据**

- `description/config/demo_robot_params.yaml:11`  (yaml demo_robot.ros__parameters.control_rate)

      control_rate: 50.0

- `launch/demo_robot.launch.py:18`  (launch param controller_frequency)

      'controller_frequency': 50.0,

- `src/diff_drive_controller.cpp:12`  (cpp_const CONTROL_RATE)

      constexpr double CONTROL_RATE = 50.0;

- `description/urdf/demo_robot.urdf:102`  (gazebo update_rate)

      <update_rate>20</update_rate>


**指标**

| 名称 | 值 |
|---|---|
| control_rate | 50 |
| sim_rate | 20 |
| ratio | 2.5 |

### [中] S06 · 1 个连杆没有惯性参数

置信度 **HIGH** · 状态 **待确认** · 标签 inertial, urdf

这些连杆没有 inertial 块，Gazebo 会套用默认质量与零惯量，与设计意图不符且容易引发不稳定。

**证据**

- `description/urdf/demo_robot.urdf:69`  (link 'caster_link' 没有 inertial 块)

### [中] S10 · 同一 YAML 作用域内重复定义 wheel_radius

置信度 **HIGH** · 状态 **已确认** · 标签 yaml

description/config/demo_robot_params.yaml 中 wheel_radius 出现 2 次（第 7, 15 行）。加载器通常只保留最后一次，前面的值被静默丢弃，改错了地方也不会报错。

**证据**

- `description/config/demo_robot_params.yaml:7`  (yaml wheel_radius)
- `description/config/demo_robot_params.yaml:15`  (yaml wheel_radius)

### [低] S09 · launch 传入的参数 wheel_diameter_mm 在配置文件中找不到

置信度 **LOW** · 状态 **待确认** · 标签 launch

launch/demo_robot.launch.py 向节点传入了数值参数 wheel_diameter_mm=101.6，但没有任何 YAML 配置声明同名键。可能是拼写错误，也可能是只在 launch 里临时覆盖；请人工确认，工具不会替你判断。

**证据**

- `launch/demo_robot.launch.py:17`  (launch param wheel_diameter_mm)

      'wheel_diameter_mm': 101.6,

## 3. 运行数据诊断（R 类）

### [高] R01 · 里程计线位移偏差 1.6%（真值 0.995 m / 里程计 0.9793 m）

置信度 **HIGH** · 状态 **已确认** · 标签 odom, scale, linear

真值 0.995 m，里程计积分 0.9793 m，比值 1.015991。真值来源：由 truth_x/truth_y 轨迹累积。这个比值只说明“尺度错了”，不能直接说明是哪个参数错了：轮半径、每转计数、减速比三者在运动学里等价。下面给出的修正候选需要与静态冲突交叉验证后才能定案。

**证据**

- `run_straight.csv:2`  (第 1 个样本（first odom sample）)

      t=0 cmd_lin=0.2 odom_lin=0.19693 truth_x=0.005

- `run_straight.csv:201`  (第 200 个样本（last odom sample）)

      t=4.975 cmd_lin=0.2 odom_lin=0.19685 truth_x=1


**指标**

| 名称 | 值 |
|---|---|
| odom_distance | 0.979339 |
| truth_distance | 0.995 |
| ratio | 1.015991 |
| truth_source | 由 truth_x/truth_y 轨迹累积 |
| used_segments | 199 |
| skipped_segments | 0 |
| time_axis_ok | True |

### [信息] R03 · 控制周期稳定（中位 25 ms ≈ 40 Hz）

置信度 **HIGH** · 状态 **已确认** · 标签 timing, pass

p95 相对中位抖动 0%，低于 20% 门限。

**证据**

- `run_straight.csv:3`  (第 2 个样本（period statistics）)

      t=0.025 cmd_lin=0.2 odom_lin=0.19687 truth_x=0.01


**指标**

| 名称 | 值 |
|---|---|
| n | 199 |
| median | 0.025 |
| mean | 0.025 |
| p95 | 0.025 |
| max | 0.025 |
| min | 0.025 |
| jitter_frac | 0 |
| median_hz | 40 |

### [信息] R06 · 速度环响应平稳（超调 0%，穿越 0 次）

置信度 **MEDIUM** · 状态 **已确认** · 标签 pid, pass

恒速段 setpoint = 0.2 m/s，未见明显振荡。

**证据**

- `run_straight.csv:62`  (第 61 个样本（setpoint 0.2）)

      t=1.5 cmd_lin=0.2 odom_lin=0.19678 truth_x=0.305

- `run_straight.csv:201`  (第 200 个样本（response tail）)

      t=4.975 cmd_lin=0.2 odom_lin=0.19685 truth_x=1


**指标**

| 名称 | 值 |
|---|---|
| setpoint | 0.2 |
| overshoot_pct | -1.45 |
| crossings | 0 |
| freq_hz | 0 |
| segment_samples | 200 |

### [信息] R08 · 时间轴与数值列完整

置信度 **HIGH** · 状态 **已确认** · 标签 data, pass

共 200 个样本，无 NaN，时间戳单调递增。

**证据**

- `run_straight.csv`

## 4. 根因关联与修改建议（C 类）

### [高] C01 · 根因收敛：驱动轮半径 应为 0.0508（当前 0.05）

置信度 **HIGH** · 状态 **已确认** · 标签 correlation, root-cause

运行数据给出的尺度比值（真值/里程计）= 1.015991，即里程计每报告 1 m，实际走了 1.015991 m。把这一比值代入各候选参数：驱动轮半径 0.05 -> 0.05079956（与静态文件中的 0.0508 一致）；编码器每转计数 1024 -> 1007.88271598；电机减速比 30 -> 30.47973689。其中 编码器每转计数、电机减速比 在数学上同样能解释该误差，但没有任何静态文件支持它，因此判定为次要可能。 修改建议来自两路证据的交叉验证，请你确认轮子实物尺寸后再应用。

**证据**

- `description/config/demo_robot_params.yaml:7`  (yaml demo_robot.ros__parameters.wheel_radius)

      wheel_radius: 0.05

- `description/urdf/demo_robot.urdf:38`  (link left_wheel cylinder radius)

      <cylinder radius="0.0508" length="0.03"/>

- `run_straight.csv:2`  (第 1 个样本（first odom sample）)

      t=0 cmd_lin=0.2 odom_lin=0.19693 truth_x=0.005

- `run_straight.csv:201`  (第 200 个样本（last odom sample）)

      t=4.975 cmd_lin=0.2 odom_lin=0.19685 truth_x=1


**指标**

| 名称 | 值 |
|---|---|
| canonical | wheel_radius |
| ratio | 1.015991 |
| old | 0.05 |
| new | 0.0508 |
| candidates | ["wheel_radius", "ticks_per_rev", "gear_ratio"] |

**建议修改（默认不写入磁盘）**

- `description/config/demo_robot_params.yaml:7`  `0.05` -> `0.0508`  — 按 r_true = r_used * (真值/里程计) 反推得到 0.05079956，与 description/urdf/demo_robot.urdf:38 中的 0.0508 一致，采用该值
- `description/config/demo_robot_params.yaml:15`  `0.05` -> `0.0508`  — 按 r_true = r_used * (真值/里程计) 反推得到 0.05079956，与 description/urdf/demo_robot.urdf:38 中的 0.0508 一致，采用该值
- `src/diff_drive_controller.cpp:10`  `0.05` -> `0.0508`  — 按 r_true = r_used * (真值/里程计) 反推得到 0.05079956，与 description/urdf/demo_robot.urdf:38 中的 0.0508 一致，采用该值

## 5. 未验证 / 无法判定的内容（请勿当成结论）

#### [信息] R02 · 缺少里程计角速度/姿态列，无法核对转角尺度

置信度 **HIGH** · 状态 **无法判定** · 标签 odom, missing-input

日志里既没有 odom_ang 也没有 odom_yaw。

**证据**

- `run_straight.csv`

#### [信息] R04 · 指令几乎是常量，无法测量响应延迟

置信度 **HIGH** · 状态 **无法判定** · 标签 latency

cmd_lin 变化范围小于 0.05 m/s，需要阶跃或变化指令才能测延迟。

**证据**

- `run_straight.csv`

**工具本身的能力边界**：

- 只判断“文件里写了什么”，不判断“实物到底是什么”。轮子真实直径、电机实际转速仍需人工测量。
- 没有真值（truth 列或 --expect-distance / --expect-yaw）时，绝对尺度一律判为无法判定，不会给出偏差百分比。
- 表达式参数（xacro / launch 变量替换）不参与数值比对，最多在 S03 中列出。
- 不支持 rosbag2 的 CDR 反序列化；如需分析 bag，请先导出为 CSV 或逐行文本。
- 工具不会自动改代码；--apply 只在显式传入时生效，并生成 .bak 备份。

## 6. 已确认通过的检查项

- R03 控制周期稳定（中位 25 ms ≈ 40 Hz）
- R06 速度环响应平稳（超调 0%，穿越 0 次）
- R08 时间轴与数值列完整

## 7. 复现与下一步


```
# 完整复现（静态 + 运行）
python scripts/audit.py full --project examples/demo_robot --log examples/demo_robot/logs/run_straight.csv

# 只跑静态检查
python scripts/audit.py static --project .

# 确认无误后再落盘修改（会生成 .bak 备份）
python scripts/audit.py full --project . --log logs/run_straight.csv --apply
```


**已生成的补丁文件**（人工审阅后可用 git apply 或手工修改）：

- `patches/description__config__demo_robot_params.yaml.patch`
- `patches/src__diff_drive_controller.cpp.patch`

**调试痕迹**：`trace.jsonl`（逐条 JSONL，记录阶段、动作、读取的文件与判定理由）、``（同一份内容的人类可读版）。
