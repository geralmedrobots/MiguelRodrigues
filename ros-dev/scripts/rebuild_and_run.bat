@echo off
setlocal

set IMAGE_NAME=ros2-jazzy-rviz
set CONTAINER_NAME=ros2_jazzy_dev

cd /d "%~dp0.."

if not exist "ros2_ws" (
    mkdir ros2_ws
)

if not exist "ros2_ws\src" (
    mkdir ros2_ws\src
)

echo Building Docker image...
docker build -t %IMAGE_NAME% .

if errorlevel 1 (
    echo Failed. Check Docker Desktop and the build output above.
    exit /b 1
)

echo Removing old container if it exists...
docker rm -f %CONTAINER_NAME% >nul 2>&1

echo Starting container...
docker run -it ^
    --name %CONTAINER_NAME% ^
    --hostname ros2-docker ^
    --privileged ^
    --network host ^
    -e DISPLAY=host.docker.internal:0.0 ^
    -e LIBGL_ALWAYS_INDIRECT=0 ^
    -e QT_X11_NO_MITSHM=1 ^
    -v "%CD%\ros2_ws:/home/ros/ros2_ws" ^
    -w /home/ros/ros2_ws ^
    %IMAGE_NAME%

endlocal
