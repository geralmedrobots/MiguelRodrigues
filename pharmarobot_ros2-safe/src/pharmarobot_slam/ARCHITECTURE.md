# PharmaRobot SLAM v1 architecture

Production uses the repository's SLLIDAR architecture. `/front_scan`
(`sensor_msgs/msg/LaserScan`, frame `front_laser`) and the existing `/odom`
(`nav_msgs/msg/Odometry`, frame `odom`, child `base_footprint`) feed the Humble
`slam_toolbox` asynchronous online mapper. The mapper owns `map -> odom` and
publishes `/map`; no other node in this package publishes TF.

TF ownership is: odometry owns `odom -> base_footprint`; the static sensor
launch owns the genuine `base_footprint -> base_link` transform
`(0, 0, 0.042, 0, 0, 0)` and `base_link -> front_laser` / `base_link ->
back_laser`. Each edge has exactly one publisher.
There must be no static or second dynamic publisher for `map -> odom`.

The two production SLLIDARs expose independent scans (`/front_scan` and
`/back_scan`). v1 uses only `/front_scan`; no scan merger is introduced. The
rear SLLIDAR can be evaluated later after a validated merged-scan design.

The launch starts only `async_slam_toolbox_node`. It does not start drivers,
odometry, joystick, D455, Nav2, EKF, AMCL, or any motion-producing node.
Manual joystick driving is a later physical-test procedure, outside this
package and this change. Wheel geometry remains provisional and unchanged.

Before a physical test, independently verify the TF chain
`map -> odom -> base_footprint -> base_link -> front_laser`, live LaserScan timestamps and frame,
laser range/angle fields, odometry freshness, and exclusive TF ownership.
