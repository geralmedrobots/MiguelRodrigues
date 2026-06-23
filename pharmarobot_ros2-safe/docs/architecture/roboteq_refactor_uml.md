# Roboteq Driver Refactor Architecture

This document describes the behavior-preserving Roboteq driver decomposition. It is intended to match the implementation in `src/roboteq_ros2_driver` after the serial I/O worker and component extraction refactors.

## Ownership Boundaries

- `Roboteq::Roboteq` owns ROS parameters, subscriptions, timers, publishers, odometry messages, and TF publication.
- `roboteq_ros2_driver::SerialIoWorker` is the only component that owns and calls the serial transport.
- `roboteq_ros2_driver::RoboteqSerialTransport` wraps the concrete serial port access.
- `roboteq_ros2_driver::command_conversion` converts `/cmd_vel/safe` twists to left/right wheel speeds and then to configured Roboteq channels.
- `roboteq_ros2_driver::command_scaling` applies coupled saturation inside the worker command formatting path.
- `roboteq_ros2_driver::configuration` builds the required controller readback validation table.
- `roboteq_ros2_driver::odometry::OdometryIntegrator` maps channel encoder samples to wheel ticks and integrates pose/twist.
- `odom_covariance`, `odom_tf`, `odom_twist`, `roboteq_protocol`, and `command_watchdog` remain focused helper modules.

## Class Diagram

```mermaid
classDiagram
    class RoboteqNode["Roboteq::Roboteq"] {
      -SerialIoWorker serial_worker_
      -OdometryIntegrator odometry_integrator_
      -Timer command_watchdog_timer_
      -Timer odom_timer_
      -Publisher odom_pub
      -Publisher ticks_publisher_
      -TransformBroadcaster odom_tf_broadcaster_
      +cmdvel_callback(Twist)
      +odom_loop()
      +odom_publish(IntegrationResult)
      +command_watchdog_loop()
      +start_serial_worker()
    }

    class SerialIoWorker {
      -IRoboteqSerialTransport transport_
      -SerialWorkerConfig config_
      -DesiredMotorCommand desired_command_
      -EncoderSample latest_encoder_sample_
      -ConnectionState state_
      -thread worker_thread_
      +start()
      +stop()
      +submitCommand(channel_1_mps, channel_2_mps)
      +invalidateCommands()
      +takeLatestEncoderSample()
      -run()
      -connectAndValidate()
      -sendDesiredCommand()
      -pollEncoder()
      -markFailure()
    }

    class IRoboteqSerialTransport {
      <<interface>>
      +open(error)
      +close()
      +isOpen()
      +sendCommands(commands, error)
      +query(command, prefix, response, error)
    }

    class RoboteqSerialTransport {
      +open(error)
      +close()
      +isOpen()
      +sendCommands(commands, error)
      +query(command, prefix, response, error)
    }

    class OdometryIntegrator {
      -DifferentialDriveKinematics kinematics_
      -RobotPose current_pose_
      -bool twist_initialized_
      +init(wheel_radius, wheelbase, encoder_cpr)
      +integrate_channel_sample(channel_1, channel_2, dt, channel_1_name, channel_2_name)
    }

    class command_conversion {
      <<namespace>>
      +twist_to_wheel_speeds()
      +wheels_to_channels()
      +twist_to_channel_speeds()
    }

    class command_scaling {
      <<namespace>>
      +scale_pair_to_limit()
    }

    class configuration {
      <<namespace>>
      +required_controller_settings()
    }

    class roboteq_protocol {
      <<namespace>>
      +parse_config_readback()
      +parse_encoder_counts()
      +parse_firmware_id()
    }

    RoboteqNode --> SerialIoWorker : owns
    RoboteqNode --> OdometryIntegrator : owns
    RoboteqNode ..> command_conversion : uses
    RoboteqNode ..> configuration : builds settings
    SerialIoWorker --> IRoboteqSerialTransport : owns
    RoboteqSerialTransport ..|> IRoboteqSerialTransport
    SerialIoWorker ..> command_scaling : formats limited commands
    SerialIoWorker ..> roboteq_protocol : validates replies
    OdometryIntegrator ..> odom_twist : calculates measured twist
```

## Motor Command Sequence

```mermaid
sequenceDiagram
    participant Cmd as /cmd_vel/safe subscriber
    participant Node as Roboteq::Roboteq
    participant Conv as command_conversion
    participant Worker as SerialIoWorker
    participant Scale as command_scaling
    participant Serial as IRoboteqSerialTransport

    Cmd->>Node: Twist(linear.x, angular.z)
    Node->>Node: reject non-finite values
    Node->>Conv: twist_to_wheel_speeds()
    Conv-->>Node: left/right wheel mps
    Node->>Conv: wheels_to_channels(channel_1, channel_2)
    Conv-->>Node: channel_1/channel_2 mps
    Node->>Worker: submitCommand(channel_1_mps, channel_2_mps)
    Note over Node,Worker: callback returns without encoder polling
    Worker->>Worker: worker thread wakes, latest command wins
    Worker->>Scale: scale_pair_to_limit()
    Scale-->>Worker: coupled RPM or power pair
    Worker->>Serial: sendCommands(!S or !G channel commands)
```

## Encoder And Odometry Sequence

```mermaid
sequenceDiagram
    participant Worker as SerialIoWorker
    participant Serial as IRoboteqSerialTransport
    participant Protocol as roboteq_protocol
    participant Node as Roboteq::Roboteq
    participant Odom as OdometryIntegrator
    participant Pub as /wheel_ticks, /odom, TF

    loop encoder_poll_period
      Worker->>Serial: query("?CR\\r", "CR=")
      Serial-->>Worker: CR response
      Worker->>Protocol: parse_encoder_counts(response)
      Protocol-->>Worker: channel_1/channel_2 counts
      Worker->>Worker: store latest EncoderSample
    end

    loop odom timer
      Node->>Worker: takeLatestEncoderSample()
      Worker-->>Node: optional EncoderSample
      Node->>Node: compute dt from odom_last_time
      Node->>Odom: integrate_channel_sample(sample, dt, channel mapping)
      Odom-->>Node: wheel ticks, pose, measured twist
      Node->>Pub: publish WheelTicks
      Node->>Pub: publish Odometry
      opt pub_odom_tf enabled
        Node->>Pub: publish odom -> base_link transform
      end
    end
```

## Reconnect Sequence

```mermaid
sequenceDiagram
    participant Worker as SerialIoWorker
    participant Serial as IRoboteqSerialTransport
    participant Protocol as roboteq_protocol

    Worker->>Serial: open()
    Worker->>Serial: send startup stop
    Worker->>Worker: state = configuring
    loop required controller settings
      Worker->>Serial: query setting readback
      Serial-->>Worker: setting response
      Worker->>Protocol: parse_config_readback()
      Protocol-->>Worker: actual value
      Worker->>Worker: require actual == expected
    end
    Worker->>Serial: query("?FID\\r", "FID=")
    Worker->>Protocol: parse_firmware_id()
    alt require_fresh_command_after_reconnect
      Worker->>Worker: state = waiting_for_fresh_command
      Worker->>Worker: reject pre-reconnect commands by sequence
    else fresh command not required
      Worker->>Worker: state = ready
    end
```

## Shutdown Sequence

```mermaid
sequenceDiagram
    participant Node as Roboteq::Roboteq
    participant Worker as SerialIoWorker
    participant Thread as worker thread
    participant Serial as IRoboteqSerialTransport

    Node->>Worker: stop()
    Worker->>Worker: stop_requested = true
    Worker->>Worker: invalidate desired command
    Worker->>Thread: notify condition variable
    Thread->>Serial: send stop commands
    Thread->>Serial: close()
    Worker->>Thread: join()
```

## Controller State Machine

```mermaid
stateDiagram-v2
    [*] --> disconnected
    disconnected --> connecting: reconnect interval elapsed
    connecting --> configuring: transport open and startup stop sent
    configuring --> waiting_for_fresh_command: validation passed and fresh command required
    configuring --> ready: validation passed and fresh command not required
    waiting_for_fresh_command --> ready: new post-reconnect command applied
    ready --> ready: command applied / encoder polled
    ready --> unhealthy: serial failure, malformed reply, timeout, config mismatch
    waiting_for_fresh_command --> unhealthy: serial failure
    configuring --> unhealthy: validation failure
    connecting --> unhealthy: open or startup stop failure
    unhealthy --> connecting: reconnect interval elapsed
    ready --> [*]: stop requested, shutdown stop sent
    waiting_for_fresh_command --> [*]: stop requested, shutdown stop sent
    disconnected --> [*]: stop requested
```

## Preserved Behavior And Safety Notes

- Command input remains `/cmd_vel/safe`.
- Default channel ownership remains `channel_1 = right`, `channel_2 = left`.
- Reverse steering is preserved in `command_conversion::twist_to_wheel_speeds`.
- Wheel radius, wheelbase, encoder PPR/CPR, encoder sign correction, motor direction, command timeout, joystick timeout behavior, odometry math, covariance, and TF behavior are unchanged by this decomposition.
- Runtime controller configuration is read back and verified by `SerialIoWorker`; the worker does not blindly overwrite required controller settings.
- Serial ownership remains single-threaded: ROS callbacks interact with `SerialIoWorker` through synchronized state, while only the worker thread calls the transport.
- Slow or malformed encoder readback cannot execute inside the command callback path. It can still make the worker enter reconnect, which sends a stop, closes the transport, invalidates commands, and waits for a fresh command if configured.

## Validation Results

Agent 3 ran targeted validation on branch `codex-pharmarobot-safe` without hardware commands, without `/dev/roboteq` writes, without `ros2 launch`, without `ros2 run`, and without publishing velocity commands.

Commands:

```bash
source /opt/ros/foxy/setup.bash && colcon build --packages-select roboteq_ros2_driver
source /opt/ros/foxy/setup.bash && colcon test --packages-select roboteq_ros2_driver --ctest-args -R "test_command_conversion|test_roboteq_configuration|test_roboteq_odometry|test_roboteq_protocol|test_serial_worker|test_command_scaling|test_odom_twist|test_odom_covariance|test_odom_tf|test_command_watchdog"
source /opt/ros/foxy/setup.bash && colcon test-result --verbose
```

Result:

```text
Summary: 67 tests, 0 errors, 0 failures, 0 skipped
```
