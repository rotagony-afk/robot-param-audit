# demo_robot（示例工程，含故意埋设的缺陷）

这是一个**教学用** ROS 2 差速小车工程，专门用来演示 rsw-audit 能发现什么。
它不是从任何真实项目里拷贝出来的，所有文件都是为了测试而手写的最小实现。

## 埋设的缺陷

| 编号 | 位置 | 缺陷 |
|---|---|---|
| 1 | `description/config/demo_robot_params.yaml` | wheel_radius = 0.05，而 URDF 中轮子圆柱半径是 0.0508（2 英寸轮的实际值） |
| 2 | 同上 | wheel_radius 在同一作用域内声明了两次 |
| 3 | `config/legacy_params.yaml` | max_linear_vel = 500，单位仍是从 ROS 1 时代留下来的 mm/s |
| 4 | `description/urdf/demo_robot.urdf` | 左右轮 joint 的 axis 不一致（一个是 z，一个是 y） |
| 5 | 同上 | base_link 的惯量三个主轴全为 0；caster_link 完全没有 inertial |
| 6 | 同上 | 轮子在 joint origin 中相距 0.32 m，配置里却写 0.30 m |
| 7 | 同上 | Gazebo 插件 update_rate = 20 Hz，控制器却以 50 Hz 运行 |
| 8 | `launch/demo_robot.launch.py` | 多传了一个 wheel_diameter_mm，配置里没有对应键 |
| 9 | `src/diff_drive_controller.cpp` | 源码里再次硬编码了 0.05 的轮半径 |

## 配套的运行日志

`logs/` 下的三个 CSV 由 `../make_sample_logs.py` 生成，可以随时重跑：

- `run_straight.csv`：直线行走 5 s，含真值列。里程计因为用了 0.05 的轮半径而少报 1.6% 的位移。
- `run_turn.csv`：原地转向 5 s，含真值偏航角。里程计因为轮距写成 0.30 而多报 6.7% 的转角。
- `run_pid.csv`：带噪声的控制记录，包含启动死区、25% 超调、限幅平台和控制周期抖动，且**没有**真值列。

第三个日志有意不带真值：它用来演示工具在缺少真值时如何拒绝给出绝对偏差结论。
