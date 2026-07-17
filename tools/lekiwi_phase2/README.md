# LeKiwi 第二阶段：ROS 2 C++底盘驱动验收

本阶段在 Jetson上编译 `robot_platform_pkg`，只启动 `chassis_driver_node`，然后执行
三轮悬空、低速、六方向及250 ms命令超时停车验收。

## 安全条件

- 第一阶段 LeRobot底盘验收已经通过。
- 三个轮子全部悬空，物理断电开关触手可及。
- `lekiwi_host`、第一阶段脚本和其他串口程序均已退出。
- 使用第一阶段确认过的 `/dev/serial/by-id/...` 路径。

## 1. 构建

```bash
cd ~/ros2_ws/src/fcr_ros2_3/tools/lekiwi_phase2
chmod +x build_on_jetson.sh run_acceptance.sh
./build_on_jetson.sh
```

脚本使用 ROS 2 Humble，以 `RelWithDebInfo`构建 `robot_platform_pkg`及其工作区依赖，
运行包测试，并确认 `chassis_driver_node`已经安装。

## 2. 悬空验收

```bash
./run_acceptance.sh \
  --port /dev/serial/by-id/实际设备名称 \
  --confirm-wheels-off-ground
```

验收程序逐项要求人工确认：前进、后退、左移、右移、逆时针、顺时针。每项以低速运行
0.4秒并显式停车；最后停止发布速度命令，验证250 ms看门狗自动停车。

输出保存在：

```text
~/ros2_ws/log/lekiwi_phase2/chassis_driver_*.log
~/ros2_ws/log/lekiwi_phase2/lekiwi_phase2_report.json
```

如任一方向错误、轮子异响、通信超时、校验错误或停车失败，脚本判定失败。立即断电并保留
日志，不要继续落地测试。
