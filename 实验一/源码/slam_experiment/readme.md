# 实验一修复版

请先阅读同目录 README_运行说明.md，按其中的 ToDesk 文件传输、依赖安装、编译、驱动、建图与定位顺序执行。

快速命令（在小车 Ubuntu / ROS Noetic 终端中）：

```bash
bash scripts/check_env.sh
bash scripts/install_deps.sh
JOBS=2 bash scripts/build_all.sh
```

TEASER++ 的首次安装是独立步骤，详见完整说明。

分别保留终端运行：

```bash
bash scripts/run_lidar.sh
bash scripts/run_imu.sh /dev/ttyUSB0 921600
bash scripts/run_mapping.sh
```

建图终端 Ctrl+C 正常保存后，使用 maps/map.bag 定位：

```bash
bash scripts/run_localization.sh "$PWD/maps/map.bag"
```

默认雷达型号 RSHELIOS_16P，网卡、串口和 LiDAR-IMU 外参须与实际设备一致。

修复源码已完成本地检查，尚未进行小车真实 ROS 编译与联调。测试范围见 VALIDATION.md。
