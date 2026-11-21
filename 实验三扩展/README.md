# 实验三扩展：机器人终端三步运行

已发现的避障问题、算法对照及后续改进顺序见 [项目交接](HANDOFF.md)。真机现有参数和代码位置见 [方法说明](方法说明.md)。

当前版本：ROS Navfn 路径规划 + 官方 DWA 局部避障。根据前方和两侧点云持续规划，支持原地转向和短距离倒车；没有可行轨迹时减速停车并重新规划。通信或定位故障仍停车并显示具体原因。

行驶上限 0.5 m/s，起步转向上限 0.7 rad/s，进入目标 20 cm 范围停车，不调整最终朝向。原实验三保留。

在机器人 Ubuntu 的同一个终端依次执行。

**1. 准备实验并打开 RViz 点云（不运动）**

```bash
cd /home/nvidia/lab3-xy-extension && export ROBOT_PYTHON=/usr/bin/python3 && bash robot.sh prepare
```

等待“准备完成”，确认机器人桌面点云显示正常。
prepare 同时执行原实验的零速度 CAN 模式初始化。check 会检查底盘模式为 1、遥控器未接管及无底盘故障；不满足时直接说明原因，不进入 go。
网页的“准备实验”和命令行 prepare 都会重启雷达驱动、点云转换、IMU 和 FAST-LIO。准备流程核对新的进程号，并等待 IMU、雷达、定位与配准点云各自收到至少三条时间递增的新消息。IMU 无数据时自动重试一次。每次准备都会重建定位地图，须在新地图中重新选点。

**2. 检查位置与目标（不运动）**

```bash
bash robot.sh check 2 1
```

应显示当前位置、坐标系和目标距离。将 `2 1` 换成目标 X、Y（米），这是当前 SLAM 绝对坐标，单次距离最多 30 米。

**3. 运行实验三扩展，前往同一个目标**

```bash
bash robot.sh go 2 1
```

运行 Python 目标程序，调用已部署的 ROS/C++ 导航控制器。**Ctrl+C 停车**。更换目标时先停车，再用新坐标执行 check 和 go。

日志实时显示在终端，并保存在 `/home/nvidia/lab3-xy-extension/logs/`。

## 暂停、停止和关闭

在导航终端按 **Ctrl+C**：结束当前导航并发送停车指令，保留雷达、定位和点云窗口。

另开机器人终端，按需执行其中一条：

```bash
bash /home/nvidia/lab3-xy-extension/robot.sh pause     # 暂停，保留目标
bash /home/nvidia/lab3-xy-extension/robot.sh resume    # 继续当前目标
bash /home/nvidia/lab3-xy-extension/robot.sh stop      # 结束导航，等同 Ctrl+C
bash /home/nvidia/lab3-xy-extension/robot.sh shutdown  # 结束导航，关闭控制服务、实验传感器与 RViz
```

暂停时导航进程仍须保持运行；当前任务总超时为 180 秒（包含暂停）。stop 后不能 resume，需要重新 go。shutdown 不关闭 Ubuntu、不切断电源；重新实验需从 prepare 开始。命令只管理通过本入口启动的导航进程，不能代替遥控器急停；发出停车指令后应确认车体停止。若 Windows 控制台后台仍在运行，应先关闭，避免其重新连接控制器。

**check 超时：不要执行 go。** 关闭 Windows 网页控制台后台和其他导航程序（只关闭浏览器不够），它们会占用唯一控制连接，再执行 prepare、check。准备失败时看 `/home/nvidia/lab26/logs/console_service.log`、`sensors.log`；点云窗口问题看同目录 `rviz.log`。

部署目录：`/home/nvidia/lab3-xy-extension`。Python 源码为 `goto_xy.py`，命令入口为 `robot.sh`。DWA 的配置、Python 适配程序和日志在 `/home/nvidia/lab3-dwa/`，分别为 `navigation.yaml`、`bridge.py`、`logs/navigation.log`。底层接收命令在 `/home/nvidia/lab26/catkin_ws/src/basic_control/src/basic_control.cpp`，DWA 接口在同包的 `include/basic_control/dwa_client.h`。原实验三仍使用 `/home/nvidia/lab3/src/navigation.cpp`。

前端刷新后选择“实验三扩展”，准备完成再选点启动。准备会检查 DWA 是否就绪。常规 DWA 短退每轮最多约 20 cm，再前进 15 cm 才重新允许短退。

遇到死路时，控制器先检查当前能否转入另一条通路，再按本次任务的实走轨迹逐段返回。来路可以是直线、弯道，也可以包含原地转弯。回退到仍有可选方向的岔口时，车会停稳复核、转入支路，继续前往原目标。未找到出口时可一直回到连续轨迹的起点。控制器每周期检查当前后方回波和制动距离，回退线速度上限为 0.10 m/s。后方有障碍或机械臂遮挡导致观测不足时，车会停车等待。控制台会显示“沿刚驶过的路线回退”和“停车重新选路”。

受阻接管时也会检查正前方能否直行一段。刚起步缺少来路历史时，如果后方实测通道足以退到一个能转出的地点，可以先直退腾位，停稳后再用新扫描核实出口。停车提示中的前、后净距从含留量的车身边缘起算。“本帧无通道内回波”表示这一帧没有测到该方向的障碍距离。
