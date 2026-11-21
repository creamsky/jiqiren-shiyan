# 本地验证记录

验证日期：2026年10月21日。验证环境：Windows，GNU Bash 5.2、Python 3.12。目标运行环境：小车上的 Ubuntu 20.04 / ROS 1 Noetic。

已完成：

- 10 项独立测试通过：坐标平移与旋转往返、四元数归一化、无效位姿、非有限/空点云、平移和旋转关键帧、地图文件原子发布、重复关闭、不覆盖已有地图、零关键帧失败处理。
- 12 个新增或修改的 Bash 脚本语法检查通过，文件为 UTF-8 无 BOM / LF。
- 交付源码中的 32 个 launch/package XML 文件解析通过。
- 9 个 YAML 配置解析通过。
- 2 个新增 Python 文件语法检查通过。
- 7 个修改的 C++ 源码/头文件做 Tree-sitter 语法扫描。定位相关 6 个文件未检出语法错误；FAST-LIO laserMapping.cpp 存在 1 处与原附件相同的宏/语法解析限制，没有新增解析错误。此扫描不检查 C++ 类型、符号链接或 ROS 接口，不能当作真实编译。
- 模拟构建检查通过：含空格路径、默认低并发构建顺序、Quatro 失败后立即停止、错误 JOBS / 缺 ROS 提前退出。
- 网络脚本模拟检查通过：默认无修改、完整参数校验、重复 IP 不再添加；本机未实际改网卡/驱动。
- 独立复查了 runner → launch → 参数/文件路径、ExactTime 消息时间、body 点云与世界位姿的地图契约，以及 Python ROS 安装入口。
- 与原附件逐项核对，除明确排除的旧 build/devel/install、logs、Git 元数据、编辑器配置、Python 缓存和示例 GPS bag 外，所需原始源码文件没有遗漏。包内 MANIFEST.sha256 可用于检查文件完整性。

在小车上可重跑纯 Python 测试：

```bash
python3 -m unittest discover -s tests -v
```

尚未完成：真实 ROS/CMake/C++ 构建、TEASER++ 在目标机的安装验证、雷达 UDP 和 IMU 串口联调、时间/外参校准、实际地图保存与定位精度评估。未连接小车，也没有生成可冒充实测结果的地图或日志。请按 README_运行说明.md 在目标小车逐步验证，出现首个真实错误时保留 build_robot.log 和对应终端输出。
