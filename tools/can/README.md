# Gimbal CAN setup

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

Example result:

```text
GIMBAL_CAN_INTERFACE=can0
```

Use that value when launching:

```bash
ros2 launch bringup_pkg fcr_gimbal_manual_test.launch.py can_interface:=can0
```

If more than one `gs_usb` adapter is connected, select one explicitly:

```bash
./tools/can/setup_gimbal_can.sh --interface can0
```
