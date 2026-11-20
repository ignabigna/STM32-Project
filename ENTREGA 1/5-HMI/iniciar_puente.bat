@echo off
chcp 65001 >nul
title Puente Python HMI - BP_A_Master_F (OpenOCD :6666 a WebSocket :8765)
cd /d "%~dp0"
echo Iniciando puente Python para la HMI...
echo.
python bridge.py %*
if errorlevel 1 (
    echo.
    echo [ERROR] El puente finalizo con un codigo de error.
    pause
)
