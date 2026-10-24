# 示例与样例数据

| 路径 | 内容 |
|---|---|
| `demo_robot/` | 一个含 9 处故意埋设缺陷的 ROS 2 差速小车工程，用来演示静态检查 |
| `demo_robot/logs/run_straight.csv` | 直线行驶 5 s，带真值位移，用来演示里程计线尺度偏差 |
| `demo_robot/logs/run_turn.csv` | 原地转向 5 s，带真值偏航角，用来演示转角尺度偏差 |
| `demo_robot/logs/run_pid.csv` | 闭环记录，无真值，用来演示死区、超调、限幅、周期抖动，以及工具在缺真值时如何拒绝下结论 |
| `make_sample_logs.py` | 生成上面三个日志的脚本（固定随机种子，结果可复现） |
| `synthetic_clean/` | 一份各项参数完全一致的最小工程，用于验证“正常输入不报警” |

重新生成日志：

```bash
python examples/make_sample_logs.py
```

复现一次完整诊断：

```bash
python ros2-sim-hw-mismatch-auditor/scripts/audit.py full \
    --project examples/demo_robot \
    --log examples/demo_robot/logs/run_straight.csv \
    --out audit-out/demo
```
