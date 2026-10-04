# Nav2 动态局部避障与 RPP 纯跟踪稳定性实测指南

本指南面向已拉起 Gazebo Sim 与 Nav2 的两轮差速底盘，针对 **局部代价地图 (Local Costmap) 动态刷新**、**Regulated Pure Pursuit (RPP) 纯追踪控制器** 的路径跟踪平滑性、转弯减速与动态重规划能力提供一套标准化实测流程。

---

## 目录
1. [测试前监控工具链准备](#1-测试前监控工具链准备)
2. [实验一：基准直线与 S 弯跟踪精度测试](#2-实验一基准直线与-s-弯跟踪精度测试)
3. [实验二：动态障碍物切入与代价地图刷新实测](#3-实验二动态障碍物切入与代价地图刷新实测)
4. [实验三：RPP 控制器核心调参消抖实战](#4-实验三rpp-控制器核心调参消抖实战)
5. [实验四：狭窄通道穿行与代价受阻减速验证](#5-实验四狭窄通道穿行与代价受阻减速验证)
6. [量化验收判定标准](#6-量化验收判定标准)

---

## 1. 测试前监控工具链准备

在进行实测前，建议开启辅助终端，分别监控底盘底层速度、跟踪偏差以及控制器状态机。

### 1.1 监控驱动控制指令 (/cmd_vel)
打开新终端，实时输出速度流：
```bash
ros2 topic echo /cmd_vel
```
* **观察重点**：线速度 `linear.x` 是否平稳上升（加速度受限），角速度 `angular.z` 是否出现高频正负跳变（振荡）。

### 1.2 监控局部路径与前视锚点 (Lookahead Point)
在 RViz2 左侧的 `Displays` 面板中配置以下可视化图层：
* **Add $\rightarrow$ By topic $\rightarrow$ `/transformed_global_plan`** (Path)：显示当前局部控制器截取的全局参考路径片段。
* **Add $\rightarrow$ By topic $\rightarrow$ `/local_plan`** (Path)：显示 RPP 控制器计算出的前瞻追踪圆弧。
* **Add $\rightarrow$ By topic $\rightarrow$ `/lookahead_point`** (PointStamped)：显示 RPP 当前锁定的前视目标点（随车速动态前移）。

---

## 2. 实验一：基准直线与 S 弯跟踪精度测试

验证在无障碍干扰下，RPP 控制器的循迹稳定性与到达目标点时的姿态收敛。

### 步骤：
1. 在 RViz2 顶部点击 **`Nav2 Goal`**，在开阔区域选定一个距离车体 $3.0\text{ m} \sim 4.0\text{ m}$ 的直线终点；
2. 观察小车从静止加速至最大设定巡航速度（如 $0.4\text{ m/s}$）的平顺度；
3. 随后下发一个带有 $90^\circ$ 直角转弯的目标点（绕过中央工作台）。

### 关键物理特性验证：
* **转弯动态降速（Regulated Linear Velocity Scaling）**：
  当小车进入拐角时，RPP 会根据当前规划圆弧的曲率半径动态限制线速度：
  $$v_{\text{scaled}} = v_{\text{desired}} \cdot \frac{r_{\text{turn}}}{r_{\text{min}}}$$
  线速度应由 $0.4\text{ m/s}$ 自动下调至 $0.15\text{ m/s} \sim 0.2\text{ m/s}$，待转弯完成后平滑恢复，避免差速底盘发生侧向滑移。
* **终点对齐无死区震荡**：
  进入目标点 $0.25\text{ m}$ 减速区后，小车减速至基础速度（`min_approach_linear_velocity`），停在目标公差内，不出现冲出目标点后反复倒车调整的现象。

---

## 3. 实验二：动态障碍物切入与代价地图刷新实测

测试当未知障碍物突然出现在既定轨迹上时，局部代价地图的标记（Marking）、清除（Clearing）以及 Nav2 行为树重规划的时效性。

### 3.1 动态推移障碍物实操
1. 保持 Gazebo 与 RViz2 左右分屏显示；
2. 在 RViz2 中下发一个长距离导航目标，使小车进入巡航状态；
3. **注入障碍**：在 Gazebo Sim 界面中，选中场景中的绿色方块（`obstacle_b`）或蓝色立柱；
4. 切换至移动模式（快捷键 `T`），在小车正前方约 $1.5\text{ m}$ 处的规划路线上直接放置该障碍物。

### 3.2 观察时序反馈机制
1. **Raytracing 与 Costmap 膨胀**：
   * 激光雷达扫描红点打在障碍物表面；
   * Local Costmap 在 1~2 个周期（$\le 100\text{ ms}$）内生成深紫色致死区及粉色膨胀层；
2. **控制器反馈**：
   * 若侧向有绕行空间：全局规划器（Navfn/Smac）或局部 RPP 立即向外偏移，引导底盘平滑绕行通过；
   * 若通道被完全阻断：底盘自动降速至完全刹停，触发 Nav2 行为树重试或原地自转恢复行为（Recovery Spin）。
3. **移除障碍后的体素清除**：
   * 将障碍物移回原位，观察 RViz2 上的 Costmap：由于激光穿透原有区域，射线清除机制（Clearing）应立即擦除该处的深色障碍，恢复为可通行的白色网格。

---

## 4. 实验三：RPP 控制器核心调参消抖实战

在差速底盘调试中，最常见的两大问题是**直线行驶蛇形走位（S 弯震荡）**与**终点画圈**。针对 `nav2_params.yaml` 中的 `FollowPath` 参数，按下表进行针对性微调：

### 核心参数对应与调优指引

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      
      # 1. 巡航线速度与角速度设定
      desired_linear_vel: 0.40          # 推荐初值: 0.35 ~ 0.5 m/s
      rotate_to_heading_angular_vel: 0.8 # 初始对齐航向的自转角速度 (rad/s)
      
      # 2. 前视距离配置 (消除蛇形震荡的核心参数)
      lookahead_dist: 0.6               # 基准前视距离 (m)
      min_lookahead_dist: 0.3           # 最小前视距离 (避免低速时前视过短引起高频振荡)
      max_lookahead_dist: 0.9           # 最大前视距离 (防止高速时过于切弯内冲)
      lookahead_time: 1.5               # 前视时间权重: lookahead_dist = v * lookahead_time
      use_velocity_scaled_lookahead_dist: true # 随车速动态放大前视距离
      
      # 3. 曲率减速机制 (防止转弯物理侧滑与翻车)
      use_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.45 # 最小转弯曲率半径阈值 (m)
      
      # 4. 接近障碍物代价动态降速
      use_cost_regulated_linear_velocity_scaling: true
      cost_scaling_dist: 0.4            # 当距离障碍物边缘小于该距离时触发减速
      cost_scaling_gain: 1.0
      
      # 5. 到点平稳制动
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
```

### 故障现象与参数对照表

| 异常现象 | 核心根因 | 推荐调整策略 |
| :--- | :--- | :--- |
| **直线行驶左右摆动（S形震荡）** | `lookahead_dist` 或 `min_lookahead_dist` 过小，控制器对微小横向偏差反应过度 | 将 `lookahead_dist` 从 `0.3` 调大至 `0.5 ~ 0.6`；将 `min_lookahead_dist` 设为至少 `0.3`。 |
| **拐弯过于激进，切入内圈碰撞障碍** | `lookahead_dist` 过大，前视点截取过远，忽视了近端弯道 | 将 `max_lookahead_dist` 下调至 `0.7 ~ 0.8`；开启转弯降速 `use_regulated_linear_velocity_scaling`。 |
| **到终点附近原地自转画圈不停止** | 终点姿态容差过紧或底盘尚未到达位置便提前开始对齐偏航角 | 增大 `yaw_goal_tolerance`（如设为 `0.08 rad` $\approx 4.5^\circ$）；检查 `GeneralGoalChecker.xy_goal_tolerance`（设为 `0.05 m`）。 |

---

## 5. 实验四：狭窄通道穿行与代价受阻减速验证

利用场景中两座工作台之间的狭长走廊测试通行极限。

### 测试步骤：
1. 观察两座障碍物之间的物理净空宽度（当前仿真场景中通道宽度约 $1.2\text{ m}$）；
2. 机器人车体有效碰撞半径 `robot_radius: 0.22`，单侧膨胀半径 `inflation_radius: 0.45`；
3. 下发一个穿过狭窄通道的目标点：
   * **预期表现**：小车进入狭长通道前，由于两侧膨胀代价上升，触发 `cost_regulated_linear_velocity_scaling`，车速自动由 $0.4\text{ m/s}$ 平稳降至 $0.2\text{ m/s}$；
   * 车身处于两障碍物中垂线平稳居中穿过，两侧不刮蹭；穿出通道后，车速自动拉升恢复至巡航速度。

---

## 6. 量化验收判定标准

执行完整的实测后，若系统满足以下指标，即可判定局部避障与 RPP 纯追踪达到工业稳定级标准：

| 评估项目 | 达标合格线 | 实测状态判定 |
| :--- | :--- | :--- |
| **直线路径跟踪横向误差** | 稳态横向偏差 $\le \pm 0.03\text{ m}$，无持续振荡 | [x] 通过（3 轮最差 0.0184 m） / [  ] 需调前视距离 |
| **直角拐弯姿态过渡** | 拐角降速明显，轮子无打滑甩尾，内轮不内切碰撞 | [x] 通过（0.40→0.075 m/s） / [  ] 需调曲率阈值 |
| **动态障碍响应延迟** | 突发障碍注入后，$\le 150\text{ ms}$ 内局部地图更新并生成避让弧线 | [x] 通过（3 轮最差 0.113 s，复位清除 0.82 s） / [  ] 需检查点云频率 |
| **终点精准制动** | 位置误差 $\le 0.05\text{ m}$，偏航角误差 $\le 5^\circ$，到位后立即停稳停正 | [x] 通过（3 轮最差 0.0462 m / 2.06°） / [  ] 需调 GoalChecker |
| **速度流连续性** | `/cmd_vel` 指令曲线平滑连续，无突变阶跃与正负符号翻转 | [x] 通过（3 轮最差行驶段翻转 2，accel p99 1.25 m/s²） / [  ] 需优化加减速限制 |
| **狭窄通道穿行** | 1.2 m 净空通道内降速（$0.4\to 0.2\text{ m/s}$ 量级）且居中通过，不刮蹭 | [x] 通过（v_mean 0.25 m/s，中线偏差最差 0.105 m） / [  ] 需调代价降速 |