# Gimbal CAN setup

The complete suspended-wheel MVP hardware acceptance procedure is documented in
[`docs/mvp_real_test_modes.md`](../../docs/mvp_real_test_modes.md). The checklist
below is mandatory after every USB-CAN unplug/replug or Jetson restart.

The gimbal is connected through a `gs_usb` USB-CAN adapter. Linux may assign
that adapter to either `can0` or `can1`, so do not hard-code the interface
number.

On the Jetson, run this before starting the gimbal driver:

```bash
cd ~/ros2_ws/src/fcr_ros2_3
chmod +x tools/can/setup_gimbal_can.sh
./tools/can/setup_gimbal_can.sh
```

The script elevates through `sudo`, detects the interface using the `gs_usb`
kernel driver, configures 1 Mbit/s SocketCAN, and prints the selected interface.

After every reconnect:

1. Stop all gimbal, mux, MVP, and keyboard nodes.
2. Run this script again; never reuse the previous `can0/can1` assumption.
3. Export the reported interface as `CAN_IF`.
4. Verify `driver=gs_usb`, link `UP`, `ERROR-ACTIVE`, bitrate `1000000`,
   `restart-ms 100`, and queue length `100`.
5. Start ROS only after all checks pass.

Example result:

```text
GIMBAL_CAN_INTERFACE=can0
```

```bash
export CAN_IF=can0  # replace with the script's actual result
ethtool -i "$CAN_IF" | grep -E "driver|bus-info"
ip -details -statistics link show "$CAN_IF"
```

Use that value when launching:

```bash
ros2 launch bringup_pkg fcr_gimbal_manual_test.launch.py can_interface:=can0
```

If more than one `gs_usb` adapter is connected, select one explicitly:

```bash
./tools/can/setup_gimbal_can.sh --interface can0
```

If configuration reports that the device disappeared, or `dmesg` contains
`Unexpected unused echo id` / `out of range echo id`, stop all ROS nodes and
unplug/replug the USB-CAN adapter before rerunning the script. Do not replace a
detected `gs_usb` interface with a board `mttcan` interface unless the gimbal is
physically connected to the board CAN transceiver.
