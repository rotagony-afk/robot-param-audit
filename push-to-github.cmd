@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0"

if "%~1"=="" (
  echo Usage: push-to-github.cmd YOUR_GITHUB_ACCOUNT
  echo Example: push-to-github.cmd zhangsan
  echo Do not type angle brackets.
  pause & exit /b 1
)
set ACCOUNT=%~1
echo Account: %ACCOUNT%
where git >nul 2>nul
if errorlevel 1 (
  echo [ERROR] git not found. Install Git for Windows first.
  pause & exit /b 1
)
echo.

rem ---- step 1: init, then repository-local identity ----
if not exist .git git init

set GITNAME=
for /f "delims=" %%i in ('git config --get user.name 2^>nul') do set GITNAME=%%i
if not defined GITNAME git config user.name "%ACCOUNT%"
set GITMAIL=
for /f "delims=" %%i in ('git config --get user.email 2^>nul') do set GITMAIL=%%i
if not defined GITMAIL git config user.email "%ACCOUNT%@users.noreply.github.com"
for /f "delims=" %%i in ('git config --get user.name') do set GITNAME=%%i
for /f "delims=" %%i in ('git config --get user.email') do set GITMAIL=%%i
echo Author: !GITNAME! ^<!GITMAIL!^>
echo.

rem ---- step 2: stage and commit ----
git add -A
git commit -m "rsw-audit: robot parameter and model verifier (42 rules, 91 tests)"
if errorlevel 1 (
  git rev-parse --verify HEAD >nul 2>nul
  if errorlevel 1 (
    echo.
    echo [ERROR] commit failed. Send the whole error text above to the assistant.
    pause & exit /b 1
  )
  echo No new changes, reusing the existing commit.
)
git branch -M main
echo.

rem ---- step 3: remote ----
git remote get-url origin >nul 2>nul
if errorlevel 1 (
  git remote add origin "https://github.com/%ACCOUNT%/robot-param-audit.git"
) else (
  git remote set-url origin "https://github.com/%ACCOUNT%/robot-param-audit.git"
)
echo Remote:
git remote -v
echo.

rem ---- step 4: connectivity probe, so a network problem is not mistaken for a repo problem ----
echo Checking connectivity to github.com:443 ...
set NETOK=
for /f "delims=" %%r in ('powershell -NoProfile -Command "try { $c = New-Object Net.Sockets.TcpClient; $c.Connect('github.com',443); if ($c.Connected) { 'yes' }; $c.Close() } catch { 'no' }" 2^>nul') do set NETOK=%%r
if not defined NETOK set NETOK=unknown
echo Connectivity: !NETOK!
echo.

rem ---- step 5: push ----
set REPOURL=https://github.com/%ACCOUNT%/robot-param-audit
echo Pushing to GitHub ...
git push -u origin main > push_log.txt 2>&1
set PUSHCODE=%errorlevel%
type push_log.txt
echo.

set REPONOTFOUND=
findstr /C:"Repository not found" push_log.txt >nul 2>nul
if not errorlevel 1 set REPONOTFOUND=1
set NETFAIL=
findstr /C:"Could not connect" push_log.txt >nul 2>nul
if not errorlevel 1 set NETFAIL=1
findstr /C:"Failed to connect" push_log.txt >nul 2>nul
if not errorlevel 1 set NETFAIL=1
findstr /C:"Connection timed out" push_log.txt >nul 2>nul
if not errorlevel 1 set NETFAIL=1
findstr /C:"Operation timed out" push_log.txt >nul 2>nul
if not errorlevel 1 set NETFAIL=1

if not "%PUSHCODE%"=="0" (
  if defined NETFAIL (
    echo [DIAGNOSIS] This machine cannot reach github.com:443. The repo and your
    echo   git setup are fine. Fix the network first, then run this script again.
    echo.
    echo   Option A: use your proxy - Clash default port is 7890:
    echo     git config --global http.proxy  http://127.0.0.1:7890
    echo     git config --global https.proxy http://127.0.0.1:7890
    echo   Option B: switch git to SSH over port 443
    echo     ssh -T -p 443 git@ssh.github.com
    echo     git remote set-url origin git@ssh.github.com:%ACCOUNT%/robot-param-audit.git
    echo   Option C: use a mobile hotspot or another network
  ) else if defined REPONOTFOUND (
    echo [DIAGNOSIS] The repository %ACCOUNT%/robot-param-audit does not exist yet.
    echo   A browser page will open. Fill it exactly like this:
    echo     Repository name: robot-param-audit
    echo     Public or Private: either
    echo     Do NOT tick README, .gitignore or License
    echo   Then run this script again.
    start "" "https://github.com/new?name=robot-param-audit"
  ) else (
    echo [DIAGNOSIS] The remote rejected the push. Likely credentials or a private repo.
    echo   Try: git push -u origin main      and log in when prompted
    echo   If asked for a password, use a Personal Access Token, not your web password.
  )
) else (
  echo SUCCESS. Repository: %REPOURL%
  start "" "%REPOURL%"
)
del push_log.txt >nul 2>nul
pause
