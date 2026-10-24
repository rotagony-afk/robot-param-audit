# 规则速查（rsw-audit）

本文件是给 AI Agent 在加载 Skill 后按需查阅的参考：每条规则的触发条件、阈值、
公式和「什么情况下不下结论」。报告里出现的编号都指向这里。

阈值都是命令行可调的：
`--tolerance`（静态数值相等判定，默认 1e-3 相对值）、
`--scale-tolerance`（运行尺度偏差判定，默认 0.01）、
`--expect-distance` / `--expect-yaw`（外部真值）。

---

## 静态规则

### S01 同一参数多处定义且数值不一致

- **条件**：同一规范参数在至少两个来源（或两处）取值相对差 > `--tolerance`。
- **严重度**：跨来源冲突为 HIGH；同一文件内的冲突为 MEDIUM。
- **不下结论时**：只说「这里有冲突」，不给修正值——静态文件本身无法说明哪一处对应实物。
- **规范化**：参数名先去除非字母数字并转小写，再查别名表；`wheel_diameter` 会自动 ×0.5 折算成半径。

### S02 疑似单位不一致

- **条件**：两个取值之比落在 10 / 100 / 1000 / 2.54 / 25.4 / 0.3048（±1%）附近。
- **严重度**：CRITICAL，status = `suspected`（无法判断哪份配置真的被加载）。

### S03 表达式参数未求值

- **条件**：字面量里出现 `dollar-brace` 形式的 xacro/launch 替换、或 `dollar-paren` 形式的 launch 替换。
- **处理**：记为 unresolved，**不参与任何数值比对**，仅在报告中列出，status = `unverifiable`。

### S04 轮距与 joint origin 几何不一致

- **公式**：从 URDF 中所有名字含 `wheel` 的 joint 取 `origin` 的 y 分量，
  当存在一正一负（左右两侧）时，`implied = y_left - y_right`。
- **条件**：`|implied - declared| / max > tolerance`。
- **严重度**：HIGH。轮距偏小会让转角偏大（`omega = (v_r - v_l) / L`）。

### S05 驱动轮 joint 旋转轴不一致

- **条件**：多个轮 joint 的 `axis` 取绝对值后方向不同 → HIGH；
  全部一致但轴与前进方向（x）平行 → MEDIUM（多半写错了）。
- **说明**：只比较方向，不比较符号——左右镜像写法是合法习惯。

### S06 惯性参数

- `mass <= 0` → HIGH
- `inertia` 的 ixx/iyy/izz 全为 0 → HIGH（物理引擎求逆失败）
- 连杆完全没有 `inertial` 块 → MEDIUM（Gazebo 会用默认质量 + 零惯量）

### S07 控制频率 vs 仿真插件更新频率

- **条件**：`control_rate > sim_update_rate * 1.01`。
- **严重度**：频率比不是整数 → HIGH；是整数 → MEDIUM。
- **说明**：`sim_update_rate` 来自 URDF 中 Gazebo 插件的 `update_rate` 或 `real_time_update_rate`。

### S08 限速不可达

- **公式**：`reachable = max_wheel_vel * wheel_radius`。
- **条件**：`max_linear_vel > reachable * 1.01` → MEDIUM。
- **跳过条件**：轮半径或轮速存在未解决的冲突（此时无法唯一确定）。

### S09 launch 传入的数值参数无人声明

- **条件**：launch 里出现数值参数，其名字既不能映射到规范参数名，也没有任何 YAML 声明同名键。
- **严重度**：LOW，status = `suspected`（可能是临时覆盖，也可能是拼写错误）。

### S10 YAML 重复键

- **条件**：同一 `(path, key)` 在同一个文件里出现多次。
- **严重度**：MEDIUM（加载器只保留最后一个，前面的静默丢失）。

---

## 运行规则

日志列名会被同义词表识别，例如
`cmd_lin | cmd_linear | v_cmd | cmd_vel_x`、
`odom_lin | odom_v | v_meas`、
`truth_x | gt_x | mocap_x`、
`t | time | stamp | timestamp`。
没有 `t` 列时会按行号合成时间轴，并把 `time_axis_ok = false` 写进指标。

### R01 线位移尺度

- **公式**：`ratio = truth_distance / odom_distance`，
  其中里程计距离是 `odom_lin` 的梯形积分，真值来自 `truth_x/truth_y` 的轨迹弧长、
  `truth_lin` 的积分或 `--expect-distance`。
- **条件**：`|ratio - 1| > --scale-tolerance`（默认 1%）。
- **缺少真值**：直接判 `unverifiable`，**不输出偏差百分比**。

### R02 转角尺度

- **公式**：优先用 `odom_yaw` 与 `truth_yaw` 的首尾差（角度先解缠），否则积分 `odom_ang`。
- **推导**：`L_true = L_used / ratio`，因为 `omega_odom / omega_true = L_true / L_used`。

### R03 控制周期抖动

- **指标**：时间差的中位数、p95、最大值，`jitter_frac = (p95 - median) / median`。
- **阈值**：≥20% → MEDIUM；≥50% → HIGH。

### R04 指令到反馈延迟

- **方法**：`cmd_lin` 与 `odom_lin` 在 0..min(60, n/4) 个样本上的归一化互相关，取峰值滞后。
- **换算**：`lag_s = lag_samples * median_dt`。
- **阈值**：≤ max(1.5 个周期, 30 ms) 视为正常；> 150 ms → HIGH；否则 MEDIUM。
- **前置**：指令变化幅度 < 0.05 m/s 时无法测延迟，判 `unverifiable`。

### R05 限幅 / 饱和

- **条件**：大量样本停在同一个极值（≥5%），且该极值明显高于其余样本的中位数（> 1.5 倍）。
- **注意**：恒速测试不算饱和——这条后置条件就是为了避免把「全程 0.2 m/s」误判成限幅。

### R06 超调与振荡

- 取最长的恒速段（≥12 个样本、|setpoint| > 0.05），丢弃前 30% 的过渡过程，在剩余窗口上统计
  `overshoot = (peak - |setpoint|) / |setpoint|` 与带 2% 死区的零点穿越次数。
- **条件**：超调 > 10% 或穿越 ≥ 3 次 → MEDIUM；超调 > 30% 或穿越 ≥ 8 次 → HIGH。
- 有可信时间轴时给出 `振荡频率 ≈ 穿越次数 / (2 * 窗口时长)`。

### R07 反馈长时间为 0

- **条件**：`|cmd| > 0.05` 且 `|odom| < 0.005` 连续 ≥ 0.5 s → HIGH。

### R08 数据质量

- 统计时间列 NaN、时间戳未递增、所有数值列的 NaN 数量。
- 不致命，但会让积分与频率统计失真；报告会明确写出「规则已跳过这些样本」。

---

## 关联规则

### C01 线尺度根因收敛

候选集：`wheel_radius`（`r_true = r_used * ratio`）、
`ticks_per_rev`（`N_true = N_used / ratio`）、
`gear_ratio`（`g_true = g_used * ratio`）。

判定顺序：

1. 若某个候选的修正值与工程里**另一处已存在的字面量**一致（容差 0.5%），取该值为根因，confidence = HIGH，
   并把其余用旧值的配置/源码位置生成为补丁；
2. 若只有唯一候选、但没有第二来源印证，confidence = MEDIUM，status = `suspected`；
3. 若有多个候选且没有印证，status = `unverifiable`，列出全部候选修正值，**不给补丁**。

### C02 转角尺度根因收敛

- **公式**：`L_true = L_used / ratio_yaw`。
- 印证来源优先取 URDF 左右轮 joint origin 推导出的轮距，其次取工程里其它轮距字面量。

---

## 补丁语义

- 补丁按「文件 → 行号 → 列号 → 原字面量」定位，同一行有多个同名字面量也不会改错。
- 只替换数值字面量本身，不重排、不格式化、不改注释。
- 默认只生成 `patches/*.patch`；`--apply` 才写回，并在写前生成 `<文件>.bak`。
- 幂等：改完再跑一次，同一处不会再生成 hunk。

## 退出码

| 码 | 含义 |
|---|---|
| 0 | 正常结束，无 CRITICAL/HIGH |
| 1 | 参数错误或输入不可读（工程目录不存在、日志不存在） |
| 2 | 正常结束，但存在 CRITICAL/HIGH（可直接用于 CI 卡点） |

## findings.json 结构

~~~json
{
  "tool": "rsw-audit",
  "version": "1.0.0",
  "summary": {"CRITICAL": 1, "HIGH": 6, "MEDIUM": 3, "LOW": 1, "INFO": 5},
  "blocking_count": 7,
  "inputs": [{"path": "...", "kind": "yaml", "size": 597, "sha256": "548c7bbbd25fce02"}],
  "findings": [
    {
      "rule": "S01",
      "severity": "HIGH",
      "confidence": "HIGH",
      "status": "confirmed",
      "title": "驱动轮半径 多处定义且数值不一致（2 组）",
      "detail": "...",
      "evidence": [{"path": "description/urdf/demo_robot.urdf", "line": 38,
                    "col": 27, "excerpt": "<cylinder radius="0.0508" length="0.03"/>",
                    "locator": "link left_wheel cylinder radius",
                    "location": "description/urdf/demo_robot.urdf:38"}],
      "proposals": [],
      "tags": ["conflict", "wheel_radius"],
      "metrics": {"canonical": "wheel_radius", "rel_diff": 0.015748}
    }
  ],
  "boundaries": ["..."]
}
~~~
