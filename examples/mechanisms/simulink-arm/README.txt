rsw-audit 生成的 Simulink 模型包
========================================

文件：
  build_rsw_arm6.m   在 MATLAB 里运行它，生成并保存 rsw_arm6.slx（推荐）
  mdl/rsw_arm6.mdl   同一模型的文本版本，可尝试直接双击打开
  params.m                 模型用到的全部数值（含审计建议的修改）

使用步骤：
  1. 把整个目录拷到装了 MATLAB + Simulink 的机器上；
  2. 在 MATLAB 当前目录指向本目录，命令行执行  build_rsw_arm6
  3. 打开生成的 .slx，点 Run，在 Scope 里看响应。

模型内容：
  - 每个关节一个二阶伺服模型：tau -> 1/(J s + b) -> omega -> q
  - J 与 b 用 params.m 里的标称值，请用 CAD 得到的真实连杆惯量替换

模块数 48，连线数 48。

说明：文本模型放在 mdl/ 子目录，避免同名 .mdl 与模型名在 MATLAB 路径上相互遮蔽（会报 shadowing 警告）。

离线结构自检：通过（模块名唯一、连线无悬空、端口合法）

必须说明的边界：
  - 这些模型是「参数级」验证模型：一阶速度环 / 二阶关节伺服 / 桨推力映射，
    用于验证参数改了之后响应怎么变，**不是**接触动力学（无碰撞、无轮胎模型、
    无气动非定常效应）。需要真实接触请用 Simscape Multibody / MuJoCo 等。
  - 本工具运行环境无法启动 MATLAB，因此 .slx 未被实际生成与打开验证；
    build_*.m 只使用了 new_system / add_block / add_line / save_system
    这些长期稳定的公开接口，并已做离线结构自检。若你的 MATLAB 版本报告
    模块路径差异，把错误发我即可，我会按你的版本调整路径。
