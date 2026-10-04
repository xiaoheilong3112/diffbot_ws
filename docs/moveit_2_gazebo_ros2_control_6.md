# 6 自由度机械臂 MoveIt 2 与 Gazebo (ros2_control) 仿真集成实践指南

本指南面向 ROS 2（推荐 Humble 或 Jazzy）平台，详细演示如何将一个标准 6 自由度（DOF）机械臂从 URDF 建模、`ros2_control` 硬件抽象、Gazebo 动力学仿真到 MoveIt 2 运动规划进行全流程打通。

---

## 1. 总体架构与数据流图

在仿真系统中，各模块的数据流和控制闭环如下：

```
[MoveIt 2 (move_group)]
       │
       │ FollowJointTrajectory Action (目标关节轨迹点: 位置/速度/时间戳)
       ▼
[ros2_control (JointTrajectoryController)]
       │
       │ 关节位置命令 / Joint Position Command
       ▼
[Gazebo 仿真硬件插件 (gz_ros2_control / gazebo_ros2_control)]
       │
       │ 物理引擎动力学求解 (求解力矩、惯性、重力与碰撞)
       ▼
[Gazebo 仿真世界 (真实关节状态)]
       │
       │ 反馈关节位置与速度 (Joint States)
       ▼
[ros2_control (JointStateBroadcaster)] ──> /joint_states ──> [robot_state_publisher & MoveIt]
```

---

## 2. 步骤一：URDF/Xacro 中集成 `ros2_control` 与 Gazebo 插件

机械臂的描述文件中不仅需要连杆（Link）和关节（Joint），还需要定义 **硬件抽象接口** 和 **Gazebo 仿真插件**。

### 2.1 机械臂 ros2_control 硬件接口定义 (`arm_ros2_control.xacro`)

在 URDF 中为 6 个旋转关节（`joint_1` ~ `joint_6`）配置命令接口与状态反馈接口：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="arm_ros2_control" params="name">
    <ros2_control name="${name}" type="system">
      <!-- 声明硬件插件为 Gazebo 仿真环境 -->
      <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        <!-- 若使用 Gazebo Classic (Gazebo 11)，则为:
        <plugin>gazebo_ros2_control/GazeboSystem</plugin> -->
      </hardware>

      <!-- 为 6 个活动关节逐一配置接口 -->
      <xacro:macro name="configure_joint" params="joint_name">
        <joint name="${joint_name}">
          <command_interface name="position">
            <param name="min">-3.14159</param>
            <param name="max">3.14159</param>
          </command_interface>
          <state_interface name="position"/>
          <state_interface name="velocity"/>
        </joint>
      </xacro:macro>

      <xacro:configure_joint joint_name="joint_1"/>
      <xacro:configure_joint joint_name="joint_2"/>
      <xacro:configure_joint joint_name="joint_3"/>
      <xacro:configure_joint joint_name="joint_4"/>
      <xacro:configure_joint joint_name="joint_5"/>
      <xacro:configure_joint joint_name="joint_6"/>

    </ros2_control>
  </xacro:macro>

</robot>
```

### 2.2 Gazebo 插件绑定 (`arm_gazebo.xacro`)

在机器人顶层 Xacro 中挂载 Gazebo 控制插件，并指向控制器参数文件：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 加载 gz_ros2_control 插件 -->
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find my_arm_bringup)/config/ros2_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- 连杆动力学与物理材质设置示例 -->
  <gazebo reference="link_1">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
  </gazebo>

</robot>
```

---

## 3. 步骤二：配置 `ros2_controllers.yaml`

控制器配置文件决定了 `controller_manager` 如何驱动关节。我们需要两个关键控制器：
1. **`joint_state_broadcaster`**：发布 `/joint_states`。
2. **`joint_trajectory_controller`**：接收轨迹并插值下发位置命令。

创建文件 `my_arm_bringup/config/ros2_controllers.yaml`：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz 控制回路频率

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    # 轨迹插值与容差设置
    open_loop_control: false
    allow_integration_in_goal_trajectories: true
    state_publish_rate: 100.0
    action_monitor_rate: 20.0

    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.5
      joint_1: { trajectory: 0.1, goal: 0.05 }
      joint_2: { trajectory: 0.1, goal: 0.05 }
      joint_3: { trajectory: 0.1, goal: 0.05 }
      joint_4: { trajectory: 0.1, goal: 0.05 }
      joint_5: { trajectory: 0.1, goal: 0.05 }
      joint_6: { trajectory: 0.1, goal: 0.05 }
```

---

## 4. 步骤三：使用 MoveIt Setup Assistant 生成配置包

使用官方配置向导自动生成 MoveIt 运动学与碰撞矩阵配置：

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

### 关键配置步骤：
1. **Load URDF**：导入解析后的机械臂完整 URDF。
2. **Self-Collisions**：调整采样分辨率（如 10,000 次以上），生成自碰撞矩阵（ACM）。将永远不会碰到的连杆（如相邻连杆、基座与地面）禁用碰撞检测以提升规划性能。
3. **Virtual Joints**：若机械臂固定在地面，不需要额外定义虚拟关节；若挂载在底盘上，需将其 `base_link` 连接到系统的根坐标系。
4. **Planning Groups**：
   * 组名：`arm`
   * Kinematic Solver：推荐选择 `KDLKinematicsPlugin`（通用解析），求解超时设置建议为 $0.005\text{ s}$。如果对规划实时性要求极高，可后续更换为 `PickIK` 或 `IKFast`。
   * 添加链（Add Kinematic Chain）：`base_link` $\rightarrow$ `tool0`。
5. **Robot Poses**：
   * 预设 `home` 姿态：所有关节为 $0$。
   * 预设 `ready` 姿态：臂略微弯曲，避开奇异点（Singularity）。
6. **MoveIt Controllers**：
   * 选择 **ROS 2 Controllers**。
   * 手动或自动添加 `arm_controller`，Action 接口类型为 `FollowJointTrajectory`。
7. **生成配置包**：输出至 `my_arm_moveit_config` 目录。

---

## 5. 步骤四：打通 MoveIt 2 与 ros2_control

MoveIt 2 必须知道如何向 `ros2_control` 发送目标轨迹。检查生成的 `my_arm_moveit_config/config/moveit_controllers.yaml`：

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller

  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
```

> **注意**：Action 的完整 ROS 2 话题服务路径为 `/arm_controller/follow_joint_trajectory`。必须确保 `ros2_controllers.yaml` 中的控制器名字与此处一致。

---

## 6. 步骤五：编写一体化启动文件 (`simulation.launch.py`)

将 Gazebo、控制器生成器（Spawners）、MoveIt 2 以及 RViz 整合到一个统一的 Launch 流程中。

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_arm_bringup')
    pkg_moveit_config = get_package_share_directory('my_arm_moveit_config')

    # 1. 解析 Xacro 生成机器人描述
    xacro_file = os.path.join(pkg_bringup, 'urdf', 'my_arm.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_raw}

    # 2. 启动 Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf'}.items()
    )

    # 3. 在 Gazebo 中生成机器人实体
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_arm', '-z', '0.0'],
        output='screen'
    )

    # 4. 发布 TF 坐标变换
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 5. 加载 ros2_control 控制器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]
    )

    # 6. 包含 MoveIt 2 move_group 启动文件
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 7. 包含 RViz 启动文件
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_moveit_config, 'launch', 'moveit_rviz.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 依赖顺序：确保关节状态控制器加载完成后，再激活手臂轨迹控制器
    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_pub,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        move_group,
        moveit_rviz
    ])
```

---

## 7. 步骤六：系统联调与运行验证

### 7.1 启动仿真并检查核心节点
打开终端执行：
```bash
ros2 launch my_arm_bringup simulation.launch.py
```

### 7.2 检查控制器状态
另起终端，确认控制器均处于 `active` 状态：
```bash
ros2 control list_controllers
```
输出应包含：
```text
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
arm_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### 7.3 在 RViz 中规划并观察 Gazebo
1. 在 RViz 的 **MotionPlanning** 插件中，选择 `Planning Group: arm`。
2. 将 **Goal State** 设为预设的 `ready` 姿态（或直接拖动末端执行器的交互球/Interactive Marker）。
3. 点击 **Plan & Execute**。
4. 观察画面：
   * RViz 中轨迹展示橙色阴影插补动画。
   * Gazebo 中的机械臂平滑运动至目标位姿，且没有剧烈抖动或坍塌。

---

## 8. 高频坑点与排错检查清单 (Troubleshooting)

| 典型现象 | 根因分析 | 针对性排查与解决方案 |
| :--- | :--- | :--- |
| **机械臂在 Gazebo 中瞬间瘫倒或炸飞** | 惯性参数（Inertial）或 PID 配置异常 | 1. 检查各 link 的 `mass` 和 `inertia`，严禁全部填 0。<br>2. 若控制器为位置接口，检查物理引擎阻尼（damping）是否过小。 |
| **MoveIt 规划成功，但点击 Execute 后超时无反应** | Action 名称或话题命名空间不匹配 | 运行 `ros2 action list`，确认是否存在 `/arm_controller/follow_joint_trajectory`。检查 `moveit_controllers.yaml` 中的名字是否完全吻合。 |
| **机械臂动作缓慢，RViz 报警 "TF_OLD_DATA"** | 节点间时间戳不同步 | 必须确保每一个节点都传入了 `use_sim_time: true`，特别是 `robot_state_publisher` 和 `move_group`。 |
| **轨迹执行中途报错 "Trajectory drift threshold exceeded"** | 关节未能按时跟上期望轨迹 | 机械臂电机出力不够或容差要求太严格。在 `ros2_controllers.yaml` 的 `constraints` 下适当放宽 `trajectory` 阈值，或增大关节的最大速度/力矩（Effort limit）。 |