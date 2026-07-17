# LeKiwi 第一阶段：Jetson 硬件验收

本阶段只验证 Feetech 控制板和底盘轮 ID 7/8/9，不运行 ROS 2、不修改电机 ID。

## 1. 安全准备

- 将底盘架起，三个轮子全部悬空。
- 保证物理断电开关触手可及。
- 停止 `chassis_driver_node`、`lekiwi_host` 等所有可能占用串口的程序。
- 不要同时运行本脚本和 ROS 2 底盘驱动。

## 2. 安装固定版本

将本目录复制到 Jetson，然后执行：

```bash
chmod +x setup_lerobot_v060.sh
./setup_lerobot_v060.sh
conda activate lerobot-v060
```

脚本固定安装 LeRobot `v0.6.0`（提交前缀 `30da8e6`）和 Python 3.12。

## 3. 查找串口

```bash
lerobot-find-port
ls -l /dev/serial/by-id/
```

优先记录 `/dev/serial/by-id/...` 路径。如果提示权限不足：

```bash
sudo usermod -aG dialout "$USER"
```

注销并重新登录后继续。

## 4. 只读探测

```bash
python verify_lekiwi_base.py \
  --port /dev/serial/by-id/实际设备名称
```

通过标准：ID 7、8、9全部响应，能读取速度和电压，不出现超时或校验错误。

## 5. 悬空低速点动

确认三个轮子全部悬空后：

```bash
python verify_lekiwi_base.py \
  --port /dev/serial/by-id/实际设备名称 \
  --motion \
  --confirm-wheels-off-ground
```

脚本逐项测试前进、后退、左移、右移、逆时针和顺时针。每项默认只运行 0.4 秒，
原始轮速限制为 350；每次结束、异常和 Ctrl+C 都会重复发送零轮速并断开扭矩。

验收结果写入 `lekiwi_phase1_report.json`。将这个文件带回开发机，作为第二阶段 ROS 2
驱动方向映射和实机对比依据。

## 6. 禁止事项

- 本阶段不要运行 `lerobot-setup-motors`；它会改写电机 ID。
- 不要落地测试或提高 `--max-raw`。
- 不要让两个进程同时打开同一个串口。
- 出现单轮方向异常、异响、过流或通信丢失时立即断电，不继续下一项。
