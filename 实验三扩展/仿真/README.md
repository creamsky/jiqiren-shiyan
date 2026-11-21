# 在浏览器运行 SCOUT MINI 仿真

**当前已关机。** 新版控制输入已切换到 ROS 激光里程计。专用 Ubuntu 尚缺 Cartographer ROS 软件包，独立定位和本版行驶仍待验收。接手时先看算法问题和定位阻塞。[实验三扩展交接](../HANDOFF.md)

1. 双击项目根目录的 **启动控制台.cmd**。在“本地仿真”页点 **启动仿真**。首次冷启动需要加载厂家网格，等到状态显示在线。
2. 主页面默认在轻量点云图中选桩桶、路障或终点，单位为米。点 **独立打开原生 Gazebo** 可另开虚拟机内的 `gzclient` 画面，用鼠标调整观察角度。[独立原生窗口](http://127.0.0.1:8765/simulation/gazebo/)
3. 点 **开始导航**。控制台会先确认布置已写入 Gazebo，再发送目标。可用 **停止运动** 结束本轮，停稳后用 **车辆复位** 返回起点。**导出场景与运行记录** 可保存当前数据。

两种模式共用本机 8765 端口和主题。打开首页只加载网页。切换到 **真实机器人** 后点 **连接真机**，用现场 SLAM 点云选点。模式切换会检查停稳反馈。结束整套实验可双击根目录的 **关闭控制台.cmd**。

## 环境与源程序

| 位置 | 内容 |
| --- | --- |
| Windows `仿真/runtime/Runtime.ps1` | 管理专用 VMware 虚拟机、部署到 Ubuntu、建立 ROS 和原生显示的独立 SSH 转发 |
| Ubuntu `/home/sim/scout-lab/sim/` | ROS 工程和 `start-gazebo.sh`。Gazebo 日志在 `/home/sim/scout-lab/gazebo.log` |
| `ros/scout_description/` | 厂家 SCOUT MINI 的车体、轮胎和关节 |
| `ros/scout_lab/` | 仿真世界、中央 16 线雷达、控制适配和规划配置 |
| `web/` | 原生 Gazebo 画面、点云选点和实验控制 |
| `验收/` | 实际运行数据和界面截图 |

专用虚拟机使用 Ubuntu 20.04、ROS Noetic 和 Gazebo Classic 11。Windows 端可在项目根目录用 PowerShell 7 单独查询或停止仿真。把示例中的 `Status` 换成 `Launch` 启动、`StopLab` 停止 ROS 和原生视窗、`Stop` 关闭专用虚拟机。新建环境需依次执行一次 `Prepare` 和 `Provision`，依赖使用清华镜像。

```powershell
& 'C:\Users\asus\AppData\Local\Microsoft\WindowsApps\pwsh.exe' -NoProfile -File '.\实验三扩展\仿真\runtime\Runtime.ps1' -Action Status
```

Ubuntu 内可直接运行 `cd /home/sim/scout-lab/sim && bash start-gazebo.sh`。独立 ROS master 使用 11431，Gazebo master 使用 11445。浏览器通过控制台访问本机转发的 9091（ROS）与 9092（原生显示）。交互式原生窗口从控制台主机打开。

## 如何理解结果

车辆使用松灵公开的四轮滑移转向模型。规划的长宽按实验三 Word 图 4.1 取 **0.65 × 0.55 m**。顶部 VLP-16 用于模拟实验车的 16 线雷达，安装基座高于 `base_link` 0.175 m，仿真坐标变换给出的扫描中心离地 **0.369 m**。厂家模型的质量、轮胎摩擦和实车载荷仍需实测标定。[厂家原始模型](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/27633a956c845903ee630538afeb17fe70afdd84/scout/scout_description)

障碍进入代价地图的依据是有限距离的激光回波。旧对照将 Gazebo 物理位姿用于导航。新版要求 ROS 只从激光观测估计位置，通过 `/scout_lab/odom` 供导航使用。独立记录的世界真值可用于事后评估定位误差。
