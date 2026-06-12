@echo off
setlocal

set CONTAINER_NAME=ros2_jazzy_dev

docker start %CONTAINER_NAME% >nul 2>&1
docker exec -it %CONTAINER_NAME% bash

endlocal
