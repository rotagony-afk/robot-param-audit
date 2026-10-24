@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
echo === 1/4 工程静态 + 运行日志（差速底盘）===
python "ros2-sim-hw-mismatch-auditor\scripts\audit.py" full --project examples\demo_robot --log examples\demo_robot\logs\run_straight.csv --out audit-out\demo
echo.
echo === 2/4 麦轮横移能力（roller 角 45 度）===
python "ros2-sim-hw-mismatch-auditor\scripts\audit.py" model --model examples\universal\mecanum_bot.json --lateral 0.2 --speed 0 --out audit-out\mecanum
echo.
echo === 3/4 滚子角写错的对照（应为 0 度）===
python "ros2-sim-hw-mismatch-auditor\scripts\audit.py" model --model examples\universal\mecanum_no_rollers.json --lateral 0.2 --speed 0 --out audit-out\no-rollers
echo.
echo === 4/4 3D 网格质量核对 ===
python "ros2-sim-hw-mismatch-auditor\scripts\audit.py" model --model examples\universal\mesh_bot.json --mesh examples\universal\chassis.stl --density 1200 --out audit-out\mesh
echo.
echo 全部完成，产物在 audit-out 目录下。
pause
