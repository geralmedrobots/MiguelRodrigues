@echo off
setlocal

set CONTAINER_NAME=ros2_jazzy_dev

docker exec -it %CONTAINER_NAME% bash

endlocal
