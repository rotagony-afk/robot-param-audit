@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
set PORT=%1
if "%PORT%"=="" set PORT=8765
echo 启动 rsw-audit Web App ...
python "ros2-sim-hw-mismatch-auditor\scripts\audit.py" app --port %PORT%
if errorlevel 1 pause
