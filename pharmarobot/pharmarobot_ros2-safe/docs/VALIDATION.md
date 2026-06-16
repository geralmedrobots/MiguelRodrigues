# Validation performed on the cleaned archive

Completed in the artifact environment:

- shell syntax validation (`bash -n`) for deployment and optional-test scripts;
- Python syntax compilation for launch files;
- XML parsing for ROS package manifests and XML launch files;
- scan for removed legacy launch/Kalman/SLAM references in active source;
- scan for generated `build/install/log`, backup and swap files;
- verification that optional logger packages contain `COLCON_IGNORE`.

Not completed in this environment:

- ROS 2/colcon compilation against the NUC image;
- Docker image build;
- hardware-in-the-loop joystick, Roboteq and LiDAR tests;
- controller fault/STO validation;
- network and systemd deployment validation on the NUC.

Before replacing the current NUC workspace, build the image and test the cleaned version with the robot lifted or otherwise mechanically secured. Keep the original backup archive available for rollback.


## Stable USB mapping validation

```bash
./deployment/scripts/check_usb_devices.sh
readlink -f /dev/roboteq
readlink -f /dev/lidar_front
readlink -f /dev/lidar_back
```

After recording the three resolved devices, reboot the NUC and repeat the
checks. Then disconnect and reconnect one device at a time and confirm that each
persistent name returns to the same hardware function. Do not run the motor
stack if `/dev/roboteq` resolves to either LiDAR adapter.
