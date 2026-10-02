# ROS 2 + Gazebo + Nav2 + MoveIt 2 移动操作复合机器人全流程仿真工程套件 (diffbot_nav2)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Jazzy-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20Fortress-orange?logo=gazebo)
![Nav2](https://img.shields.io/badge/Nav2-RPP%20Controller-green)
![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Manipulation-purple)
![License](https://img.shields.io/badge/License-Apache_2.0-yellow)

本工程是一个面向 **两轮差速自主移动底盘** 及 **上装 4 轴机械臂（移动操作复合机器人，Mobile Manipulator）** 的生产级开源仿真套件。系统采用现代 Gazebo Sim、`ros_gz_bridge`、`robot_localization` (EKF)、Nav2 与 MoveIt 2 构建，已预调优并杜绝了坐标系跳变、物理模型打滑发散等仿真常见缺陷。

---

## 目录
1. [系统整体架构与数据流](#1-系统整体架构与数据流)
2. [坐标系规范 (TF Tree)](#2-坐标系规范-tf-tree)
3. [工程目录结构与文件清单](#3-工程目录结构与文件清单)
4. [环境准备与依赖安装](#4-环境准备与依赖安装)
5. [编译与安装](#5-编译与安装)
6. [运行与操作实操](#6-运行与操作实操)
   - [场景 A：自主巡航与避障 (Nav2)](#场景-a自主巡航与避障-nav2)
   - [场景 B：复合机器人抓取转运全闭环 (Nav2 + MoveIt 2)](#场景-b复合机器人抓取转运全闭环-nav2--moveit-2)
7. [关键设计与核心避坑要点](#7-关键设计与核心避坑要点)
8. [常用调试与诊断指令](#8-常用调试与诊断指令)

---

## 1. 系统整体架构与数据流

整个系统基于 ROS 2 节点生命周期与异步 Action 通信构建，实现了“物理仿真 $\leftrightarrow$ 状态估计 $\leftrightarrow$ 运动规划 $\leftrightarrow$ 顶层调度”的分层解耦：

```text
               +-------------------------------------------+
               |        Gazebo Sim (Fortress/Harmonic)     |
               |  - 差速物理动力学 (含轮地摩擦与惯量模型)        |
               |  - 2D 激光雷达 (GPU Lidar, 15Hz)           |
               |  - 6 轴 IMU 仿真传感器 (含高斯噪波, 100Hz)    |
               +--------------------+----------------------+
                                    | 
                                    | 内存级双向桥接 (ros_gz_bridge)
                                    v
+-------------------------------------------------------------------------+
|                                ROS 2 生态                               |
|                                                                         |
|  [/clock] 仿真时钟 ----------------------------------> 所有 ROS 2 节点   |
|                                                                         |
|  [/odom/unfiltered] + [/imu/data]                                       |
|          |                                                              |
|          v                                                              |
|  +------------------------------+                                       |
|  | robot_localization (EKF 节点)|                                       |
|  +--------------+---------------+                                       |
|                 | (滤波平滑)                                            |
|                 v                                                       |
|        [/odometry/filtered] + 发布 TF (odom -> base_footprint)          |
|                 |                                                       |
|                 +----------------------------+                          |
|                 |                            |                          |
|                 v                            v                          |
|  +------------------------------+  +---------------------------------+  |
|  |       Nav2 (自主导航系统)     |  |       MoveIt 2 (机械臂规划)      |  |
|  |  - Costmap 2D (动静态与膨胀)  |  |  - Planning Group: arm/gripper  |  |
|  |  - RPP 纯追踪局部控制器      |  |  - 自碰撞免检矩阵 (ACM)          |  |
|  |  - AMCL 定位 (map->odom)     |  |  - FollowJointTrajectory        |  |
|  +--------------+---------------+  +----------------+----------------+  |
|                 |                                   |                   |
|                 +-----------------+-----------------+                   |
|                                   |                                     |
|                                   v                                     |
|            +----------------------------------------------+             |
|            | mobile_manipulation_demo.py (7 步任务状态机)  |             |
|            | 巡航待命 -> 低重心防翻折叠 -> 展开抓取 -> 返航转运 |             |
|            +----------------------------------------------+             |
+-------------------------------------------------------------------------+
```

---

## 2. 坐标系规范 (TF Tree)

系统严格遵循 **ROS REP-105** 规范，建立唯一的连通无环有向树：

$$\text{map} \xrightarrow[\text{AMCL / SLAM}]{\text{30Hz (绝对位姿校正)}} \text{odom} \xrightarrow[\text{EKF (robot\_localization)}]{\text{50Hz (滤波推算)}} \text{base\_footprint} \xrightarrow[\text{robot\_state\_pub}]{\text{Static (贴地投影)}} \text{base\_link}$$

各子分支坐标系：
* $\text{base\_link} \xrightarrow{\text{Z=+0.12m}} \text{laser\_link}$（2D 激光雷达扫描基准）
* $\text{base\_link} \xrightarrow{\text{Z=+0.05m}} \text{imu\_link}$（几何旋转质心处）
* $\text{base\_link} \xrightarrow{\text{Z=+0.12m}} \text{arm\_base\_link} \rightarrow \dots \rightarrow \text{wrist\_pitch\_link} \rightarrow \text{gripper\_base\_link}$（上装机械臂链条）

> **关键准则**：`odom -> base_footprint` 的广播权独家赋予 `ekf_node`，Gazebo 驱动插件内部的 TF 广播被显式关闭，彻底消除双源冲突引发的模型抖动。

---

## 3. 工程目录结构与文件清单

功能包 `diffbot_nav2` 包含 10 个工业标准文件，目录布局如下：

```text
diffbot_nav2/
├── CMakeLists.txt                  # ament_cmake 构建与资源安装定义
├── package.xml                     # 依赖包声明元数据清单
├── urdf/
│   ├── diffbot.urdf.xacro          # 底盘核心刚体、轮系、惯量宏与传感器 link
│   ├── diffbot_gazebo.xacro        # Gazebo Sim 差速驱动插件、激光雷达与 IMU 配置
│   └── diffbot_arm.urdf.xacro      # 4 轴机械臂 + 滑轨双指夹爪 + ros2_control 硬件标签
├── config/
│   ├── ros_gz_bridge.yaml          # Gazebo 与 ROS 2 话题双向桥接配置表
│   ├── ekf.yaml                    # 参考 linorobot2 的 2D 状态估计滤波参数
│   ├── nav2_params.yaml            # Nav2 核心参数 (RPP 控制器、Costmap 2D、AMCL)
│   ├── diffbot_arm.srdf            # MoveIt 2 规划组、收折姿态与自碰撞免检矩阵 (ACM)
│   └── moveit_controllers.yaml     # MoveIt 2 与 ros2_control 动作服务映射
├── launch/
│   └── diffbot_sim.launch.py       # 编排拉起物理世界、Spawn 模型、EKF、Nav2 与 RViz2
└── scripts/
    └── mobile_manipulation_demo.py # 复合移动抓取端到端任务调度 Python 脚本
```

---

## 4. 环境准备与依赖安装

### 4.1 支持环境
* **操作系统**：Ubuntu 22.04 LTS (Jammy) 或 Ubuntu 24.04 LTS (Noble)
* **ROS 2**：Humble Hawksbill / Jazzy Jalisco
* **Gazebo**：Gazebo Sim Fortress / Harmonic

### 4.2 安装核心二进制依赖包
以 ROS 2 Humble 为例（若使用 Jazzy，将命令中的 `humble` 替换为 `jazzy`）：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-ros-gz \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-moveit \
  ros-humble-moveit-ros-move-group
```

---

## 5. 编译与安装

### 5.1 获取源码（二选一）

#### 选项 A：使用工作台一键部署脚本（秒级自动生成）
在 Web 工作台右上角点击 **「⚡ 一键部署脚本」**，或直接在终端粘贴执行：
```bash
# 自动建立目录并一次性写入全部 10 个文件
mkdir -p ~/diffbot_ws/src
cd ~/diffbot_ws/src
# 将工作台生成的 bash 脚本粘贴运行
```

#### 选项 B：解压 ZIP 包
在 Web 工作台点击 **「📦 下载工程 ZIP」**，将解压后的 `diffbot_nav2` 放入 `~/diffbot_ws/src/`。

### 5.2 编译工作空间
```bash
cd ~/diffbot_ws
colcon build --symlink-install --packages-select diffbot_nav2

# 激活环境变量（建议写入 ~/.bashrc）
source install/setup.bash
```

---

## 6. 运行与操作实操

### 场景 A：自主巡航与避障 (Nav2)

启动物理仿真、EKF 滤波融合与 Nav2 导航全栈：
```bash
ros2 launch diffbot_nav2 diffbot_sim.launch.py
```

**操作步骤**：
1. **位姿初始化**：在自动弹出的 RViz2 顶部工具栏中点击 **`2D Pose Estimate`**，在地图原点处单击并沿车头朝向拖动箭头；
2. **观察粒子收敛**：激光雷达扫描红点与静态地图黑色墙体完全对齐；
3. **下发目标点**：点击顶部 **`Nav2 Goal`**，在走廊任意通行区域点击并拖拽设定终点朝向；
4. **自主导航**：全局规划器生成黄色可行路径，局部 RPP 控制器驱动底盘平稳起步、遇障减速并精准停靠目标位。

---

### 场景 B：复合机器人抓取转运全闭环 (Nav2 + MoveIt 2)

在 Launch 启动后，另开一个新终端执行上装作业调度：
```bash
source ~/diffbot_ws/install/setup.bash
ros2 run diffbot_nav2 mobile_manipulation_demo.py
```

**任务时序执行流程**：
1. **[Step 1] 收拢防侧翻**：机械臂自动运动至 `transport` 姿态（极度收缩贴近车体上表面，重心降至最低），夹爪开启；
2. **[Step 2] Nav2 长途巡航**：底盘高速前往料台工位（坐标：$X=2.0\text{m}, Y=1.0\text{m}$），到达后触发静止制动；
3. **[Step 3] 展开准备**：MoveIt 控制机械臂升起至 `ready` 姿态；
4. **[Step 4] 抓取工件**：手臂下探至 `pick` 姿态，双指滑轨夹爪闭合（闭合至 $0.005\text{m}$，施加抓取力）；
5. **[Step 5] 再次折叠收拢**：携带工件回缩至 `transport` 姿态，重新锁定系统重心；
6. **[Step 6] Nav2 转运返航**：底盘携带工件运送至卸料目标台；
7. **[Step 7] 放置与复位**：张开夹爪卸下工件，机械臂恢复折叠待命。

---

## 7. 关键设计与核心避坑要点

| 痛点问题 | 故障表现 | 工程机理 | 本套件解决方案 |
| :--- | :--- | :--- | :--- |
| **万向支撑轮摩擦锁死** | 机器人无法转弯，原地剧烈抖动打滑 | Gazebo 默认接触摩擦系数 $\mu_1=\mu_2=1.0$，产生巨大侧向阻力矩 | `diffbot_gazebo.xacro` 中将万向轮 `<mu1>` 和 `<mu2>` 强制归零。 |
| **惯量张量非正定** | 机器人刚在仿真中生成即被炸飞升空 | 随意填 0 或违反刚体三角形不等式，物理引擎积分出无穷大加速度 | URDF 中使用规范长方体与圆柱体转动惯量数学解析宏。 |
| **仿真时钟不同步** | TF 报错 "extrapolation into the future" | 仿真时钟 `/clock` 与系统真实时间偏差数十亿秒，队列失效 | Launch 文件中所有 Node 与 Nav2 全局注入 `use_sim_time: True`。 |
| **EKF 双源 TF 竞争冲突** | 机器人在 RViz 中疯狂跳跃瞬移 | Gazebo 插件与 EKF 同时发布 `odom -> base_footprint` | 禁用 Gazebo 驱动插件内部 TF 发布，**由 EKF 独占广播**。 |
| **RPP 路径跟踪蛇形震荡** | 底盘直道行驶走 S 形，到点频繁自转 | 前视距离过小或转弯未减速引起超调 | 增大 `lookahead_dist` 至 $0.6\text{m}$，开启曲率动态降速。 |
| **复合机器人自碰撞锁死** | MoveIt 启动即报错拒绝执行任何规划 | 机械臂基座与底盘外壳紧贴，被规划引擎判定为初始发生自碰撞 | 在 SRDF 的 ACM 中将 `arm_base_link` 与 `base_link` 声明为免检。 |

---

## 8. 常用调试与诊断指令

```bash
# 1. 验证仿真时钟是否正常步进 (必须每秒累加)
ros2 topic echo /clock --once

# 2. 检查传感器原始输入频率
ros2 topic hz /scan               # 期望: ~15 Hz
ros2 topic hz /imu/data           # 期望: ~100 Hz
ros2 topic hz /odom/unfiltered    # 期望: ~50 Hz

# 3. 检查 EKF 滤波状态估计输出
ros2 topic hz /odometry/filtered  # 期望: 50 Hz

# 4. 生成并导出完整 TF 树拓扑图
ros2 run tf2_tools view_frames
# 当前终端目录下将生成 frames.pdf，用以确认是否存在双根节点或环路

# 5. 监听底层驱动轮速控制指令
ros2 topic echo /cmd_vel
```

---

## 9. 扩展与二次开发路线

1. **升级局部路径控制器**：将 `nav2_params.yaml` 中的控制器插件由 RPP 升级为 **Nav2 MPPI**，实现非平稳障碍物高动态避让；
2. **三维立体感知拓展**：在 `base_link` 正前上方增配 RGB-D 深度相机（如 RealSense D435i），在 Nav2 中引入 **STVL (Spatio-Temporal Voxel Layer)** 规避悬空障碍物；
3. **精密自主对接**：引入 `opennav_docking` 任务服务器，结合 AprilTag 视觉标记实现毫米级自动回充对准。