# rsw-audit · 机器人参数 / 模型验证器

**把"仿真能跑、上实物就跑偏"从凭经验猜参数，变成有文件行号证据的排查。**

许可证 Apache-2.0 · Python ≥ 3.9 · 运行期零第三方依赖 · 91 个自动化测试全部通过

支持 **URDF / Xacro / MJCF / SDF / JSON 规格 / STL / OBJ**，覆盖
**差速、阿克曼、麦轮、全向、滑移** 五类轮系，附带一个零依赖本地 **Web App**。

---

## 1. 它解决什么问题

差速底盘最常见的调试场景：仿真里直行、转弯都正常，上实车之后

- 走 1 米，里程计只记 0.98 米；
- 该转 90°，实际转过 96°；
- 同一份 PID 参数，昨天能用今天抖；
- 改了 params.yaml 却不生效。

这些现象背后往往是同一类事故：**同一个物理量被写在 3～4 个地方，而且互不一致。**
轮半径既写在 URDF 的圆柱半径里，又写在 params.yaml 里，还硬编码在 C++ 里；
轮距既可以从左右轮 joint origin 算出来，又可以在配置里手填。

换成别的底盘，问题变成另外几种：

- 麦轮平台上"横移"指令发下去，车不走直线侧移——因为模型里滚子角忘了写；
- 阿克曼车按 (v, ω) 命令走，实际轨迹半径对不上——因为轮距/轴距写反；
- 手里只有 3D 模型，模型里的质量是随手填的，加速和制动全都算不准。

rsw-audit 对这三类问题给同一套答案：**读进来 → 算清楚 → 给证据 → 判不了就说判不了**。

## 2. 三层能力

| 层 | 输入 | 能回答的问题 |
|---|---|---|
| 工程静态比对 | URDF/YAML/launch/源码 | 同一个参数在哪几个文件里写得不一致？差多少？第几行？ |
| 模型级验证 | URDF/MJCF/SDF/JSON + STL/OBJ | 这条指令这个底盘做不做得到？轮速/附着/力矩够不够？网格质量和惯量对不对？ |
| 仿真对比 | 运行日志 + 实测参数 | 模型能解释这次运行吗？如果不能，该改哪个参数、改成多少？ |

## 3. 核心功能与规则

| 类别 | 编号 | 检查内容 |
|---|---|---|
| 静态一致性 | S01 | 同一参数多处定义且数值不一致（跨 urdf/yaml/launch/源码） |
| | S02 | 疑似单位不一致（10 / 100 / 1000 / 2.54 / 25.4 倍） |
| | S03 | 表达式参数（xacro / launch 变量）未求值，已排除出数值比对 |
| | S04 | 轮距声明值与 URDF 轮子 joint origin 几何不一致 |
| | S05 | 驱动轮 joint 旋转轴方向不一致 |
| | S06 | 质量为 0/负、惯量全 0、连杆缺 inertial |
| | S07 | 控制频率高于 Gazebo 插件更新频率 |
| | S08 | 限速超过轮子实际可达速度 |
| | S09 | launch 传入了配置里不存在的数值参数 |
| | S10 | 同一 YAML 作用域内重复定义同一键 |
| 模型级 | M01 | 模型参数不完整（无轮子 / 轮半径为 0 / 质量为 0） |
| | M02 | 该底盘物理上做不出这条指令（残留侧滑 / 转向角超限） |
| | M03 | 轮速超限 |
| | M04 | 附着不足（需求牵引力 > μ·N） |
| | M05 | 力矩不足（超出电机扭矩-转速曲线） |
| | M06 | 阿克曼几何异常（缺轴距/轮距，或内外轮角差异过大） |
| | M07 | 网格反推质量/惯量与模型不符、网格不封闭、轮子超出包围盒 |
| | M08 | 实测参数（轮半径/质量/摩擦）与模型值不一致 |
| | M09 | 仿真与实测是否一致 |
| | M10 | 根因收敛：应修改的参数与数值（直接比值 / 灵敏度两条路径） |
| | M11 | 转向几何修正：车身原点需要的侧向速度 |
| | A01 | 机械臂目标点不可达 |
| | A02 | 机械臂解接近奇异位形 |
| | A03 | 机械臂负载力矩超过关节额定 |
| | A04 | 机械臂自由度不足 3 |
| | P01 | 螺旋桨声明推力与 kT/转速不符 |
| | P02 | 满转扭矩超过电机额定 |
| | P03 | 满油门推力不足以悬停 |
| | P04 | 桨盘载荷偏高（效率提示） |
| | L01 | 激光雷达声明点数与扫描几何不符 |
| | L02 | 安装高度/俯仰导致地面盲区过大 |
| | L03 | 精度优于量程分辨率（规格矛盾） |
| 运行诊断 | R01 | 里程计线位移尺度偏差（需真值或 --expect-distance） |
| | R02 | 里程计转角尺度偏差（需真值或 --expect-yaw） |
| | R03 | 控制周期抖动（中位/p95/最大） |
| | R04 | 指令到反馈的响应延迟（互相关） |
| | R05 | 指令限幅 / 输出饱和 |
| | R06 | 超调与振荡（穿越次数、振荡频率） |
| | R07 | 有指令但反馈长时间为 0 |
| | R08 | 时间轴与数值列的数据质量（NaN、非单调） |
| 根因关联 | C01 | 线尺度根因收敛（R01 + S01/S02/S10） |
| | C02 | 转角尺度根因收敛（R02 + S04） |

每条结论都带 severity（CRITICAL/HIGH/MEDIUM/LOW/INFO）、confidence（HIGH/MEDIUM/LOW）、
status（confirmed / suspected / **unverifiable**）和 evidence（文件:行号 + 原文摘录）。

## 4. 技术栈

- **语言**：Python 3.9+，**只用标准库**（re / csv / json / math / difflib / hashlib / struct / http.server）。
- **自研解析器**：带行列的 XML tokenizer（URDF / MJCF / SDF）、缩进感知 YAML 扫描器（flow 列表逐元素列号）、
  launch 正则提取器、C++/Python 常量提取器、**STL/OBJ 网格解析器**（二进制 + ASCII）。
- **自研几何**：有符号四面体分解算体积 / 质心 / 惯量张量。
- **自研运动学**：一行雅可比覆盖五种轮系，外加固有不侧滑约束与转向几何。
- **自研仿真**：运动学 + 一阶执行器滞后 + 传输延迟 + 轮速/附着限制，参数反推用直接比值与数值灵敏度。
- **自研 App**：`http.server` 单文件前端，token 校验，不联网。
- **测试**：pytest，**91** 个用例（含网格惯量与解析解逐项对比）。

## 5. 目录结构

~~~
robot-param-audit/
├── README.md
├── LICENSE                       Apache-2.0
├── pyproject.toml
├── requirements.txt              说明「无第三方运行时依赖」
├── run-app.cmd / run-app.sh      一键启动 Web App
├── run-demo.cmd                  一键跑完四个示例
├── ros2-sim-hw-mismatch-auditor/        ← Skill 目录（自包含）
│   ├── SKILL.md
│   ├── scripts/
│   │   ├── audit.py
│   │   └── rswaudit/             21 个模块：扫描 / 规则 / 运动学 / 仿真 / 网格 / 机构 / Simulink / App
│   │       └── webui.html        App 前端（单文件，无外部资源）
│   ├── references/rules.md       规则语义、阈值与公式速查
│   └── assets/sample-report.md   真实生成的示例报告
├── examples/
│   ├── demo_robot/               差速底盘，埋了 9 处缺陷 + 3 份日志
│   ├── synthetic_clean/          参数完全一致的对照工程（验证不误报）
│   └── universal/                麦轮 / 阿克曼 / 网格 / MJCF / 实测参数
├── tests/                        91 个 pytest 用例
├── tools/
│   ├── make_demo_video.py        用真实运行输出渲染演示视频
│   ├── html_to_pdf.py            作品介绍 HTML → PDF
│   └── make_submission.py        生成提交用 zip
└── docs/                         需求定义卡、测试记录、视频方案、通用化说明、作品介绍
~~~

## 6. 安装

无需安装即可运行：

~~~bash
git clone <this repo> && cd robot-param-audit
python --version          # 需要 3.9+
~~~

想安装成命令行工具（可选）：

~~~bash
pip install -e .
rsw-audit --version
~~~

## 7. 运行方法

~~~bash
# A. 工程静态比对
python ros2-sim-hw-mismatch-auditor/scripts/audit.py static --project examples/demo_robot

# B. 工程 + 运行日志 + 根因收敛
python ros2-sim-hw-mismatch-auditor/scripts/audit.py full     --project examples/demo_robot     --log examples/demo_robot/logs/run_straight.csv --out audit-out/demo

# C. 任意底盘模型验证（差速/阿克曼/麦轮/全向/滑移）
python ros2-sim-hw-mismatch-auditor/scripts/audit.py model     --model examples/universal/mecanum_bot.json --lateral 0.2 --speed 0

python ros2-sim-hw-mismatch-auditor/scripts/audit.py model     --model examples/universal/mesh_bot.json     --mesh examples/universal/chassis.stl --density 1200

python ros2-sim-hw-mismatch-auditor/scripts/audit.py model     --model examples/demo_robot/description/urdf/demo_robot.urdf     --log examples/demo_robot/logs/run_turn.csv --used-track 0.30

# D. 本地 Web App
python ros2-sim-hw-mismatch-auditor/scripts/audit.py app --port 8765
# 或双击 run-app.cmd（Windows）/ ./run-app.sh（Linux/macOS）
# 或双击 run-demo.cmd 一次跑完四个示例
~~~

退出码：`0` 无严重/高级问题，`2` 存在 CRITICAL/HIGH，`1` 输入不可读。

## 8. 示例输入

差速底盘（`examples/demo_robot`）：

~~~csv
t,cmd_lin,odom_lin,truth_x,truth_y
0.000,0.2000,0.196883,0.005000,0.000000
0.025,0.2000,0.196883,0.010000,0.000000
~~~

~~~
# 配置第 7 行                           # URDF 第 38 行
    wheel_radius: 0.05                   <cylinder radius="0.0508" length="0.03"/>
~~~

麦轮（`examples/universal/mecanum_bot.json`）：

~~~json
{"drive": "mecanum", "wheel_radius": 0.075,
 "wheels": [{"name": "fl", "x": 0.22, "y": 0.18, "roller_deg": 45}, "..."]}
~~~

## 9. 示例输出

**差速底盘 + 直线日志**（真实输出）：

~~~text
统计: 严重 1 | 高 7 | 中 2 | 低 1 | 信息 5
[严重] S02 [HIGH/待确认] 疑似单位不一致：最大线速度（比值 1000）
[高] C01 [HIGH/已确认] 根因收敛：驱动轮半径 应为 0.0508（当前 0.05）
[高] R01 [HIGH/已确认] 里程计线位移偏差 1.6%（真值 0.995 m / 里程计 0.9793 m）
[高] S01 [HIGH/已确认] 驱动轮半径 多处定义且数值不一致（2 组）
    建议: description/config/demo_robot_params.yaml:7  0.05 -> 0.0508
~~~

**麦轮横移能力**（真实输出）：

~~~text
# 滚子角 45°：可行，四轮 1.886 rad/s
# 滚子角被写成 0°：
[高] M02 [HIGH/已确认] 该底盘无法完成横向运动（残留侧滑 0.200 m/s）
~~~

**3D 网格质量核对**（真实输出）：

~~~text
[高] M07 [HIGH/已确认] 网格反推质量 7.200 kg 与模型声明 2.400 kg 相差 200.0%
[中] M07 [MEDIUM/已确认] 网格反推偏航惯量 0.0780 与模型声明 0.0200 相差 290%
~~~

**仿真对比与参数反推**（真实输出）：

~~~text
[信息] M09 [HIGH/已确认] 仿真与实测一致（位移差 0.03%）
       按固件轮半径 0.05 m 推出来的里程计是 0.9816 m，实测 0.9793 m，相差 0.23%
[高] M10 [HIGH/已确认] 根因收敛：轮半径应为 0.0508 m（当前用 0.05 m 换算）
~~~

产物（`--out` 指定的目录）：`report.md` / `findings.json` /
`trace.jsonl` / `trace.md` / `patches/*.patch`。

## 10. Web App

`run-app.cmd`（或 `python ... app --port 8765`）后浏览器打开终端打印的地址：

- 拖入模型 / 日志 / 网格 / 实测参数，填命令的 vx、vy、ω 与固件轮半径轮距；
- 右侧直接显示分级结论、证据（文件行号 + 原文）、建议修改；
- 勾选「同时导出 Simulink 模型包」后，还会生成 build_rsw_*.m / .mdl / params.m，并可以点「用本机 MATLAB 构建并运行」，直接跑一次 sim 拿到记录信号的末值；
- 可下载 report.md / findings.json / trace；
- 只绑 127.0.0.1、校验一次性 token、上传限定扩展名与 8 MB、不联网、不执行上传内容。

## 11. 机械臂 / 螺旋桨 / 激光雷达 与 Simulink 导出

三类全新机构，各自一套公式、一族规则，接口统一：

| 机构 | 算什么 | 规则 |
|---|---|---|
| 机械臂（JSON `arm.joints`） | 正运动学、几何雅可比、阻尼最小二乘逆解、静力矩、可操作度 | A01 不可达 / A02 奇异 / A03 力矩超限 / A04 自由度不足 |
| 螺旋桨 | T = kT·w²、Q = kQ·w²、P = Q·w、悬停转速、桨盘载荷 | P01 推力不符 / P02 扭矩超限 / P03 飞不起来 / P04 效率低 |
| 激光雷达 | 点数/秒、光斑直径、地面盲区半径 | L01 点数不符 / L02 盲区过大 / L03 规格矛盾 |

~~~bash
python ros2-sim-hw-mismatch-auditor/scripts/audit.py model     --model examples/mechanisms/arm6.json --payload 2.0 --target 0.45,0,0.30

python ros2-sim-hw-mismatch-auditor/scripts/audit.py model     --model examples/mechanisms/quad_x.json --simulink out/simulink
~~~

真实结果：机械臂 2 kg 负载 → `A03 j5_pitch 需要 1.94 N·m / 额定 1.50 N·m`；
四旋翼声明 14 N 实际 10.66 N → `P01 相差 24%`；雷达声明 10 万点/秒实际 28.8 万 → `L01`。

### Simulink 导出

`--simulink <目录>` 按刚诊断的对象生成模型包：`build_<name>.m`（在 MATLAB 里跑它，
用 `new_system`/`add_block`/`add_line`/`save_system` 生成真正的 `.slx`）、
`<name>.mdl`（文本版）、`params.m`（全部数值 + 审计建议的修改）、
`README.txt`（结构、自检结果、边界）。

轮式 = 一阶速度环 + 里程计增益；机械臂 = 每关节二阶伺服 + 限位饱和；螺旋桨 = 转速滞后 + T=kT·w²。
生成后做离线结构自检（模块名唯一、连线无悬空、端口合法）并把结果写进 README。

**验证状态（2026-10 实测）**：在 MATLAB R2025b + Simulink 上执行 build_rsw_arm6，成功生成并保存
rsw_arm6.slx（48 个模块 / 48 条连线，模型图正确渲染）。本项目构建环境无法启动 MATLAB，所以自动化只覆盖
「生成脚本语法 + 离线结构自检」这一层；sim 的运行波形请在 MATLAB 里点 Run 确认。

## 12. 测试方法

~~~bash
python -m pytest        # 91 passed
~~~

覆盖范围：

- 解析器：XML 多行标签行列号、YAML flow 列表逐元素列号、launch/源码常量、CSV 与 key=value 日志；
- 每条规则：示例工程查出全部 9 处埋设缺陷；对照工程**零发现**（防误报）；
- **证据一致性**：所有证据行号必须与文件里该行的原文逐字相等（自动核对 20+ 条）；
- 网格几何：立方体 / 长方体 / 圆柱的体积与三个主轴惯量逐项对比解析解；
- 运动学：差速轮速、麦轮横移、阿克曼内外轮角、不可横移判定；
- 仿真与反推：复现"里程计少报 1.6%"、反推轮半径 0.0508 / 轮距 0.32；
- App：文件名净化、session 过滤、错误 token 拒绝。

## 13. 设计原则

1. **没有读到就不存在**：参数只来自实际解析到的字面量，绝不按文件名推断。
2. **没有真值就不给偏差**：缺 `truth_*` 且没给 `--expect-distance/--expect-yaw` 时，R01/R02 判 `unverifiable`。
3. **数学等价就承认不确定**：轮半径、每转计数、减速比都能解释同一尺度误差，没有第三方证据时列出全部候选。
4. **模型说要能动，物理说不能动，以物理为准**：滚子角为 0 的"麦轮"就是不能横移，工具会直接说。
5. **报告单列「未验证 / 无法判定」章节**，并写明工具自身的能力边界。
6. **默认不改用户代码**，补丁只生成不落盘；`--apply` 才写，且留 `.bak`。

## 14. 已知限制

- **运动学 + 准静态动力学，不是接触动力学**：没有 MuJoCo/PyBullet 级接触求解，不预测振动、跳齿、翻车。
- 不解 rosbag2 的 CDR 二进制，请先导出 CSV 或 key=value 文本。
- 滑移转向按"普通轮不可侧滑"保守处理，只给打滑倾向，不建轮胎模型。
- 物理真值（轮子真实直径、地面摩擦）无法从文件推出，仍需人工测量。
- 未在装有 ROS 2 的 Linux 环境实测（工具不依赖 ROS 运行时，只解析文件与日志）。
- 中文报告；代码与接口为英文。

## 15. 许可证

Apache License 2.0，见 `LICENSE`。
