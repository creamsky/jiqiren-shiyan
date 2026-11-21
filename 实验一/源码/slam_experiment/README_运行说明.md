# 实验一修复版 ToDesk 编译与运行说明

本包用于在电脑通过 ToDesk 连接的小车 Ubuntu 桌面上编译和运行。代码目标为原附件使用的 ROS 1 Noetic，默认雷达型号为 RSHELIOS_16P，IMU 默认为 /dev/ttyUSB0、921600。网卡、串口、安装外参仍需与实际小车一致。

修复版提供完整源码、编译和启动脚本，并补充了“建图导出关键帧地图 → 加载地图定位”的流程。本次已做本机静态检查和独立测试，尚未在小车 ROS 环境完成真实编译或传感器联调。原始 DOCX、RAR 没有被修改。请在新目录使用修复版，保留原来的程序和地图。

## 1 通过 ToDesk 传文件

1. 电脑打开 ToDesk，连接你的小车，进入小车的 Ubuntu 桌面。
2. 使用远程窗口顶部的“文件”或文件传输框，发送 `实验一_修复版代码.zip`，接收目录选小车的 `~/Downloads`。
3. Linux 端使用文件传输框；不要依赖 Windows 之间才支持的拖拽或复制文件方式。界面名称以当前版本显示为准。[ToDesk 官方文件传输说明](https://todesk.com/helpcenter/questions-147.html)
4. 后续所有终端命令都在远程 Ubuntu 桌面中执行。

在小车打开终端（Ubuntu 常用 Ctrl+Alt+T），解压：

```bash
mkdir -p "$HOME/robot_labs"
python3 -m zipfile -e "$HOME/Downloads/实验一_修复版代码.zip" "$HOME/robot_labs"
cd "$HOME/robot_labs/slam_experiment"
```

若 ZIP 实际保存在其他位置，修改第一条解压命令中的 ZIP 路径。解压目标 `robot_labs/slam_experiment` 应是新目录；不要覆盖已有运行中的工作区。脚本均使用 Bash 调用，不需要对整个目录执行 chmod 777。

## 2 检查小车环境

```bash
lsb_release -a
uname -r
ls /opt/ros
bash scripts/check_env.sh
```

正常目标是 Ubuntu 20.04 + ROS Noetic。若小车已有 Noetic，继续下一步；若只有 Melodic、ROS 2 或没有 ROS，当前脚本会明确停止。先保留现有系统，确认实验指定环境后再安装或适配；本包不会自动更换小车系统。

`check_env.sh` 只检查，不改网络、不安装、不编译。首次没有地图是正常情况；缺少编译依赖时会列出 MISSING。它也会列出网卡、串口、SDK 和 Python ROS 模块。

## 3 安装缺少的依赖

已有依赖可跳过。需要时手动运行：

```bash
cd "$HOME/robot_labs/slam_experiment"
bash scripts/install_deps.sh
```

该脚本需要小车联网，输入的是小车 Ubuntu 用户的 sudo 密码；终端输入密码时通常不显示字符。脚本只支持已有 ROS Noetic 的 Ubuntu 20.04，安装 catkin_tools、ROS serial、PCL、Eigen、TBB、yaml-cpp、libpcap、RViz、NumPy 等依赖，不会自动安装另一个 ROS 版本。

### TEASER++ 单独安装一次

Quatro 还依赖 TEASER++。如果环境检查已经找到 `teaserppConfig.cmake`，不要重复安装。缺少时可使用以下固定上游版本；该过程需要联网，TEASER++ 的 CMake 还会下载其自身依赖：

```bash
mkdir -p "$HOME/slam_deps"
cd "$HOME/slam_deps"
git clone --branch v2.0 --depth 1 https://github.com/MIT-SPARK/TEASER-plusplus.git TEASER-plusplus-v2
cmake -S TEASER-plusplus-v2 -B TEASER-plusplus-v2/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTS=OFF \
  -DBUILD_DOC=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_MATLAB_BINDINGS=OFF \
  -DENABLE_DIAGNOSTIC_PRINT=OFF
cmake --build TEASER-plusplus-v2/build -j2
sudo cmake --install TEASER-plusplus-v2/build
sudo ldconfig
```

一条成功后再执行下一条。如果该目录已存在，先确认它就是这次的 v2.0 源码；不要删除其他人的依赖目录。v2.0 对应上游提交 `d19ce8c8fc58f43fcd1e19a9cfd6ed32717ef3cf`；固定版本便于排查，不代表已经在本小车验证。[TEASER++ 官方安装说明](https://teaser.readthedocs.io/en/latest/installation.html)

修复版已包含 Livox SDK 源码，并改为随工作区编译。即使实际使用 RS 雷达，也要保留这个目录，因为 FAST-LIO 用到 Livox ROS 消息包。构建过程不会删除它或重新从网络拉取它。

## 4 一次编译三个工作区

回到修复版根目录：

```bash
cd "$HOME/robot_labs/slam_experiment"
bash scripts/check_env.sh
set -o pipefail
JOBS=2 bash scripts/build_all.sh 2>&1 | tee build_robot.log
```

脚本会依次编译雷达、IMU、Nano-GICP、Quatro 和整个 SLAM 工作区；先加载 ROS 环境，再编译，成功后才加载对应 `devel/setup.bash`。只有全部成功才会显示 `Build completed successfully`。

小车内存较少或出现 `cc1plus` 被 killed 时，降低并发：

```bash
JOBS=1 bash scripts/build_all.sh
```

局部重编译接口：

```bash
JOBS=2 bash scripts/build_all.sh --drivers-only
JOBS=2 bash scripts/build_all.sh --slam-only
```

若 TEASER++ 装在自定义目录，可指定它的 CMake 配置目录，例如：

```bash
teaserpp_DIR=/实际安装路径/lib/cmake/teaserpp JOBS=2 bash scripts/build_all.sh --slam-only
```

上面的 `/实际安装路径` 是占位示意，需要换成实际路径。不要在 Windows PowerShell 中运行这些构建命令。

## 5 检查雷达网络和 IMU

```bash
cd "$HOME/robot_labs/slam_experiment"
bash FAST-LIO-Localization/imu_drive/net.sh
```

默认模式只检查设备。原附件自动设置 eth1、eth2 和固定 Jetson 内核模块的行为已经移除。

### IMU 串口

查看实际串口：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

若设备是 `/dev/ttyUSB0` 且当前用户无读写权限，可临时授权该串口：

```bash
bash FAST-LIO-Localization/imu_drive/net.sh --serial /dev/ttyUSB0 --grant-serial
```

重新插拔或重启后临时权限可能失效。长期权限可由小车管理员加入 dialout 组并重新登录。设备不存在时，先检查线缆和 USB 芯片驱动；必要时显式加载对应驱动：

```bash
bash FAST-LIO-Localization/imu_drive/net.sh --load-module ch341
```

只有实际芯片需要 cp210x 时，才把 ch341 换成 cp210x。不再使用写死内核版本的 `.ko` 绝对路径。

### 雷达网卡

本包默认雷达型号 `RSHELIOS_16P`，UDP 端口 6699 / 7788。先确认雷达设备的 IP、目标主机 IP、接线和小车上的专用雷达网卡。

仅当确认需要添加静态地址时，按实际设备修改下面参数：

```bash
bash FAST-LIO-Localization/imu_drive/net.sh --interface eth1 --address 192.168.1.102/24
```

这里的 eth1 和地址只是原实验配置示例。选择连接雷达的网卡，不要选择承载 ToDesk 连接的网卡。脚本只添加地址，不清空旧地址、不改默认路由；但错误网段仍可能影响网络。已配置正确时不用执行这一条。

两个需要核对的配置文件：

- `RS_SDK/src/rslidar_sdk/config/config.yaml`：雷达型号、UDP 端口；已修正 `ts_first_point:true`。
- `FAST-LIO-Localization/FAST-LIO-Localization-QN-master/src/third_party/FAST_LIO/config/velodyne.yaml`：话题、线数和外参；已修正秒单位 `timestamp_unit:0` 和默认 16 线。外参仍保留原值，实际安装位置、方向不同时须重新标定。

## 6 分三个终端启动驱动和建图

三个终端均在小车 Ubuntu 桌面打开，并保留运行。下面每个脚本会自行定位并加载自己的工作区。

终端 A，启动雷达：

```bash
cd "$HOME/robot_labs/slam_experiment"
bash scripts/run_lidar.sh
```

终端 B，启动 IMU：

```bash
cd "$HOME/robot_labs/slam_experiment"
bash scripts/run_imu.sh /dev/ttyUSB0 921600
```

串口与波特率不同时，替换最后两个参数。

另开诊断终端，先确认输入数据持续更新：

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /rslidar_points
```

观察后在这个诊断终端 Ctrl+C，再依次检查：

```bash
rostopic hz /velodyne_points
```

```bash
rostopic hz /imu
```

有话题名称但无持续数据不算通过。停止诊断命令不会停止 A、B 终端中的驱动。

终端 C，启动建图并导出定位地图：

```bash
cd "$HOME/robot_labs/slam_experiment"
bash scripts/run_mapping.sh
```

首次默认输出：

```text
maps/map.bag    定位用的关键帧地图
maps/map.pcd    查看和保存用的全局点云
```

再次建图需要换一个未使用的名称：

```bash
bash scripts/run_mapping.sh "$HOME/robot_labs/slam_experiment/maps/map_02.bag"
```

脚本会同时使用 `map_02.pcd`，已有 bag、pcd 或正在写入的文件会被拒绝覆盖。RViz 不需要时可使用 `RVIZ=false bash scripts/run_mapping.sh`。

刚开始让小车保持静止，待 IMU 初始化、点云和里程计正常，再按实验原有方式移动小车。新代码会输出 `Saved keyframe N`。这里没有新增底盘运动控制程序。

地图导出器使用 FAST-LIO 的同步 `/Odometry` 与 `/cloud_registered`，把世界点云转回对应 body 坐标，成对保存 `/keyframe_pcd` 与 `/keyframe_pose`。它保存的是 FAST-LIO 里程计地图，没有额外实现回环优化；长距离建图仍可能有漂移。

## 7 正常停止并检查地图

在终端 C 按 Ctrl+C，等待终端返回提示符和保存日志。不要直接关闭远程窗口或强制断电。

```bash
cd "$HOME/robot_labs/slam_experiment"
ls -lh maps/map.bag maps/map.pcd
source /opt/ros/noetic/setup.bash
rosbag info maps/map.bag
pcl_viewer maps/map.pcd
```

`rosbag info` 应包含 `/keyframe_pcd` 和 `/keyframe_pose`，消息数相等且非零。看到 `Map saved:` 和 `Point cloud saved to`，并检查实际文件后，再进入定位。

运行中会有 `.bag.active` 和 `.bag.lock` 文件。零关键帧或记录失败时不会发布正式 `.bag`；异常断电也可能留下临时文件。不要把临时文件直接改名当成有效地图；先确认驱动输入与日志，重新建图时另取地图名称。PCD 与 bag 分别保存，需分别确认结果。

当前 `pcd_save.interval:-1` 会把点云累积到内存，短程实验后及时保存；很长时间或大范围建图应另行设置分段保存策略，分段文件也不能代替定位用的关键帧 bag。

## 8 加载刚才的地图进行定位

保持终端 A、B 的驱动运行。先确认建图终端已经完全退出，然后让小车回到建图起点附近，初始朝向也尽量接近，保证当前扫描与地图有充分重叠。

在终端 C：

```bash
cd "$HOME/robot_labs/slam_experiment"
bash scripts/run_localization.sh "$HOME/robot_labs/slam_experiment/maps/map.bag"
```

定位脚本检查地图文件，定位节点校验话题、类型、数量、时间戳、位姿和点云。地图有问题会明确报错并停止，不能用 `.pcd` 或普通原始传感器录包代替。

正常状态顺序：

1. 显示 `Loaded map`，说明地图读取成功。
2. 等待当前扫描和地图配准。
3. 显示 `Initial map registration accepted` 后，才开始发布全局 `/pose_stamped` 与 `map -> robot` TF。

可在诊断终端查看输出：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /pose_stamped
```

初始匹配尚未成功时没有全局位置输出，这是修复后的明确行为。此时检查起点、场景重叠、时间同步和外参，不要把 RViz 有画面当作定位已经成功。

不要同时启动建图脚本和定位脚本，因为它们都会启动 FAST-LIO。定位模式关闭了额外 PCD 累积，避免覆盖建图点云并减少内存占用。

## 常见错误的处理

| 提示或现象 | 检查与处理 |
| --- | --- |
| catkin: command not found | 回到依赖安装和环境检查；不要先 source 不存在的 devel |
| teaserppConfig.cmake 找不到 | 安装 TEASER++，或提供正确的 teaserpp_DIR |
| cc1plus 被 killed / 内存不足 | 用 JOBS=1 编译，检查内存和交换空间 |
| Serial port not found | 核对实际 /dev/ttyUSB*、USB 线和驱动 |
| Serial permission missing | 使用指定串口的 --grant-serial，或处理 dialout 权限 |
| 雷达持续 MSOP/DIFOP timeout | 查雷达型号、网卡地址、设备目标 IP、端口和接线 |
| 一直无点云/轨迹 | 分别检查 /velodyne_points 和 /imu 的数据频率和时间戳 |
| output already exists | 换新的地图名称，脚本不会覆盖原图 |
| 地图两个话题缺失或数量不同 | 使用本包建图导出的已完成 bag；检查保存日志 |
| No accepted initial map match | 回到建图起点附近，检查朝向、场景重叠、时间和外参 |
| RViz 图形或远程显示异常 | 可先用 RVIZ=false 启动，再检查数据；不是自动证明算法失败 |

## 修改与验证记录

已修复编译入口文本、先 source 后构建、定位空地图和越界、头文件多重定义、消息生成依赖、点云时间单位与首点时间、固定网卡/内核路径、地图保存路径、定位输出前的配准状态等问题。

新增地图导出器与按步骤启动脚本；清理交付包中的旧 build/devel、旧编译日志、Git 元数据和示例 gps bag，保留所需源码与许可证。Livox SDK 改为使用包内源码，不再在 CMake 中删除后重新下载。

已执行 10 项点云坐标变换/异常输入/文件保存保护测试，以及 Bash、XML、YAML、Python 和 C++ 语法扫描；构建流程还做了模拟失败传播测试。这些结果不能替代小车上真实 ROS 编译、地图精度评估和硬件联调。具体记录见包内 `VALIDATION.md`。
