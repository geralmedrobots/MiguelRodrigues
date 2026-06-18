# Optional test and logging tools

These packages and scripts are intentionally excluded from the normal robot build and startup.

- `src/odom_test_logger/COLCON_IGNORE`
- `src/robot_test_logger/COLCON_IGNORE`
- no production launch file starts either logger
- no systemd service starts the test scripts

To use them explicitly:

```bash
./tools/optional_tests/build_optional_test_nodes.sh
./tools/optional_tests/run_5_inverse_2m_tests.sh
```

The motion-test scripts publish to `/cmd_vel/test`, which is routed through the command arbiter at lower priority than the joystick. Use them only with a clear test area and an accessible emergency stop. The inverse test has a 30-second wall-clock safety timeout.
