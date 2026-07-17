# FCR safe remote control

This package closes the command path so that `command_mux` is the only publisher
to the actuator topics `cmd_vel` and `cmd_gimbal`.

## Command paths

- Manual input: `teleop/cmd_vel`, `teleop/cmd_gimbal`
- Automatic input: `auto/cmd_vel`, `auto/cmd_gimbal`
- Safety input: `teleop/heartbeat`, `teleop/deadman`, `teleop/estop`
- Final output: `cmd_vel`, `cmd_gimbal`
- State: `remote_control/status`

The mux starts in `manual` mode, but outputs zero until a fresh heartbeat,
deadman lease, and manual command are all present. Software ESTOP is latched.
Clearing it requires a fresh heartbeat, released deadman, and zero output.

## Jetson startup

Build and start the chassis plus safety mux for the first suspended-wheel test:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select teleop_control_pkg bringup_pkg servo_control_pkg robot_platform_pkg
source install/setup.bash
ros2 launch teleop_control_pkg remote_chassis.launch.py
```

In a second interactive terminal, start the keyboard client:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_control_pkg keyboard_platform_teleop
```

After chassis-only acceptance, start all platform drivers with
`ros2 launch teleop_control_pkg remote_platform.launch.py use_sim:=false`.

Terminal input cannot report physical key release. Repeat a motion key to renew
the 250 ms deadman lease. Stopping key input therefore stops the robot. Controls:

- `W/S`: forward/back; `A/D`: strafe; `Q/E`: rotate
- Arrow keys: gimbal pitch/yaw
- Space: stop; `X` or Escape: latch software ESTOP; `C`: clear ESTOP
- `M`: manual; `O`: auto; `P`: safe stop; `+/-`: speed scale

## Acceptance checks

Keep the wheels suspended for the first run. Verify:

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /remote_control/status
```

`/cmd_vel` must report exactly one publisher: `/command_mux`. Check that motion
requires repeated motion keys, stops within about 300 ms when keys or Wi-Fi disappear,
ESTOP remains latched, clearing ESTOP while deadman is active is rejected, and
manual/auto mode changes include a zero-output dwell.

When the keyboard runs on another computer, both machines must use the same
`ROS_DOMAIN_ID`, and the operator machine must have this workspace sourced.

Software ESTOP is not a substitute for a physical power-cut emergency stop.
