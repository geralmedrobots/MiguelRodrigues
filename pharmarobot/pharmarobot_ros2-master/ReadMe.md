
The file explains how to run and operate within the docker-based ROS environment 


## ToDo: 
    [  ] 2D sick scans 
    [x ] teleop package does not work
    [x ] Odometry 
    [x ] If USBs are not pluged in, the container does not build. -> I mounted the at /dev instead of the device directly 
    [x ] if the USBs unplugs, one has to rebuild the container to make USB to work again; -> Same as previous
    [x] [Robteqdriver] when controlling the wheel continues  to rotated   
    [x ] Set PID at the config file  
    [x ] Test joystick with remote controler
    [x ] Test Roboteq driver 





## Setup Robot
**Note** You need to login into the docker acount

```
ROS_DISTRO=humble
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0 <- local or within a network
```

Pull latest docker image from repo: 

```
docker pull tiagobarrosisr/pharmarobot:latest
```


## Hardware:
- Realsense: D455
- Roboteq: SBL2340 
- Sick Laser: NanoScan3
- Remote controller: 054c:0268 Sony Corp. Batoh Device / PlayStation 3 Control


## V1.0 

This version has the following features:
- initiates when the OS boots
- enables Teloperation
- runs realsense camera



## Acknowledgment

**Roboteq ros driver package fork from:** https://github.com/CJdev99/roboteq_ros2_driver

Follow the installation guide at roboteq_ros2_driver/Readme.md



**Joystick Driver Package fork from:**
https://github.com/FurqanHabibi/joystick_ros2

https://wiki.ros.org/joy


**Sick laser scanner**
https://github.com/SICKAG/sick_safetyscanners2






# Compile and build ROS2 workspace 
2) Compile ROS2 workspace, at the workspace root:
```
colcum build
```

# Run Telop

***Note***: USBs must be pluged in !
```
 ros2 run  joy_to_cmdvel joy_to_cmd_vel
 ros2 run joy_linux joy_linux_node 
 ros2 run joy_linux joy_linux_node --ros-args -p  deadzone:=0.001
 ros2 run roboteq_ros2_driver roboteq_ros2_driver 


 ros2 launch roboteq_ros2_driver roboteq_ros2_driver.launch.py 
```

