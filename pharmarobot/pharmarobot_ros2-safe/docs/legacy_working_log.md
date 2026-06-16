
Following the following tutorial 
https://medium.com/@oelmofty/ros2-in-docker-why-and-how-b72b3880dc97


30/07/2025
- realsense is working 
- build v1.0, uploaded it to docker and git repositories



26/06/2025
 - teleop is running when booting docker
 - transferred and build docker image on poharmarobot's local pc. 


12/05/2025
working on the following docker image
 nav2_image_isaac_sim

To run:  
```
 docker run -it --rm nav2_image_isaac_sim /bin/bash
```

```
Device on my laptop: /dev/ttyUSB0
```

10.231.214.157
source install/setup.bash && ros2 launch teleop teleop_launch.xml


Boot docker at boot in pharmarobot platform

ssh medrobots@10.231.214.157

 rsync -avzP build_and_run_docker.sh  medrobots@10.231.214.157:/home/medrobots/pharmarobot/ros2_ws/

Wifi 
ssh medrobots@10.231.214.129

### Boot docker with robteqdriver

this has worked https://chatgpt.com/share/687f8d55-f470-8000-b610-bc95b27a0393

sudo nano /etc/systemd/system/pharmarobot.service


docker stop pharma_container
sudo systemctl stop pharmarobot
sudo systemctl disable --now pharmarobot

### Production vs development

docker build -t pharmarobot:prod --build-arg BUILD_MODE=production .
# Run the container (no mounting needed)
docker run -it --rm pharmarobot:latest




docker build -t pharmarobot:dev --build-arg BUILD_MODE=development .
docker build -t pharmarobot:release --build-arg BUILD_MODE=production .




# Sick Laser scanners
192.168.1.11 -> 00:06:77:32:25:85 -> SICK AG

192.168.1.12 -> 00:06:77:32:2a:71 -> SICK AG

# Nav2

## Tutorial 

https://roboticsbackend.com/ros2-nav2-tutorial/ 

```
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 run turtlebot3_teleop teleop_keyboard
```

To initiate a terminal outside VS code 

```
docker exec -it -u tbarros 7d98fbe98d2c bash -c "cd /ros2_wk && bash"
```


## Nav2 documentations

For nav2 overview:
```
https://docs.nav2.org/concepts/index.html
```


## Pharma Teleop

- the robot platform initiates 

