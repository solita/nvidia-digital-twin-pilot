@echo off
setlocal

set "ISAAC_SIM_BAT=C:\isaac-sim-standalone-5.1.0-windows-x86_64\isaac-sim.bat"
set "SCRIPT_DIR=%~dp002_core_scripts"
set "LAUNCHER_SCRIPT=shotcrete_direct_cave_preview_launcher_v1.py"

if not exist "%ISAAC_SIM_BAT%" (
  echo Isaac Sim launcher not found:
  echo   %ISAAC_SIM_BAT%
  exit /b 1
)

if not exist "%SCRIPT_DIR%\%LAUNCHER_SCRIPT%" (
  echo Preview launcher script not found:
  echo   %SCRIPT_DIR%\%LAUNCHER_SCRIPT%
  exit /b 1
)

pushd "%SCRIPT_DIR%"
call "%ISAAC_SIM_BAT%" --exec "%LAUNCHER_SCRIPT%"
set "EXIT_CODE=%ERRORLEVEL%"
popd

endlocal & exit /b %EXIT_CODE%
