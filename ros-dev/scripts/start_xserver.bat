@echo off
setlocal

set VCSRV_PATH=C:\Program Files\VcXsrv\vcxsrv.exe

if not exist "%VCSRV_PATH%" (
    echo VcXsrv not found at:
    echo %VCSRV_PATH%
    echo.
    echo Install VcXsrv or update VCSRV_PATH in this script.
    exit /b 1
)

taskkill /IM vcxsrv.exe /F >nul 2>&1

start "" "%VCSRV_PATH%" :0 -multiwindow -clipboard -wgl -ac

echo VcXsrv started on display :0
echo Docker DISPLAY should be: host.docker.internal:0.0

endlocal
