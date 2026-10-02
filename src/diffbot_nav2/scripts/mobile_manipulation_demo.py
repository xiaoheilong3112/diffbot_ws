#!/usr/bin/env python3
"""
ROS 2 Nav2 + MoveIt 2 移动复合机器人端到端作业控制脚本
功能：巡航至料台 -> 机械臂展开 -> 抓取工件 -> 折叠收拢 (低重心防翻车) -> 导航至放置台 -> 卸货返航
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class MobileManipulatorMission(Node):
    def __init__(self):
        super().__init__('mobile_manipulator_mission')
        
        # 1. 初始化 MoveIt 2 控制器 Action 客户端
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        self.get_logger().info('等待 MoveIt 2 控制器 Action 服务上线...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('MoveIt 2 控制服务已就绪！')

    def send_arm_joint_goal(self, joint_angles, duration_sec=3.0):
        """下发机械臂 4 轴目标关节角度并同步等待执行完成"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4']
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = int(duration_sec)
        goal.trajectory.points.append(point)

        self.get_logger().info(f'>> 规划执行机械臂姿态: {joint_angles}')
        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('机械臂动作目标被拒绝！')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('机械臂姿态执行完成。')
        return True

    def set_gripper(self, position, max_effort=15.0):
        """控制平行双指夹爪开合 (0.035m 打开, 0.005m 闭合)

        左右指同值同步: 两关节 axis 方向相反, 相同 joint value 即对称开合.
        (Gazebo 物理引擎不支持 <mimic>, 故由轨迹控制器显式双关节驱动)
        max_effort 保留仅为调用兼容, 开环位置控制下不使用力限.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = 1
        goal.trajectory.points.append(point)

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('夹爪动作目标被拒绝！')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'夹爪行程到位: {position} m')
        return True

def main():
    rclpy.init()

    # 1. 实例化任务节点与 Nav2 Navigator
    mission = MobileManipulatorMission()
    navigator = BasicNavigator()

    # 预设关节姿态 [j1, j2, j3, j4]
    TRANSPORT_POSE = [0.0, 1.30, -1.45, 0.15] # 紧凑收折 (低重心，防倾覆)
    READY_POSE     = [0.0, 0.45,  0.50, -0.95] # 展开待命
    PICK_POSE      = [0.0, 0.70,  0.60, -1.30] # 俯身抓取料台工件

    print("\n=======================================================")
    print("    ROS 2 移动复合机器人: Nav2 + MoveIt 2 协同演示")
    print("=======================================================\n")

    # ---------------------------------------------------------
    # 步骤 1: 航行前收拢机械臂 (防止急刹车翘尾翻车)
    # ---------------------------------------------------------
    mission.get_logger().info('[Step 1] 收折机械臂至 Transport 低重心姿态...')
    mission.send_arm_joint_goal(TRANSPORT_POSE, duration_sec=2.5)
    mission.set_gripper(0.035) # 预先打开夹爪

    # ---------------------------------------------------------
    # 步骤 2: Nav2 自主导航至取料工作台
    # ---------------------------------------------------------
    mission.get_logger().info('[Step 2] Nav2 启动: 规划航线前往取料工位 (X=2.0m, Y=1.0m)...')
    pick_goal = PoseStamped()
    pick_goal.header.frame_id = 'map'
    pick_goal.header.stamp = navigator.get_clock().now().to_msg()
    pick_goal.pose.position.x = 2.0
    pick_goal.pose.position.y = 1.0
    pick_goal.pose.orientation.w = 1.0 # 航向朝前

    navigator.goToPose(pick_goal)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'   底盘巡航中... 剩余距离: {feedback.distance_remaining:.2f} m', end='\r')
        time.sleep(0.3)

    nav_result = navigator.getResult()
    if nav_result != TaskResult.SUCCEEDED:
        mission.get_logger().error('底盘未能成功抵达工位，任务终止！')
        return

    mission.get_logger().info('\n[Step 2 完成] 已平稳停靠料台前，底盘制动！')

    # ---------------------------------------------------------
    # 步骤 3: 机械臂展开并抓取工件
    # ---------------------------------------------------------
    mission.get_logger().info('[Step 3] MoveIt 2: 机械臂展开至 Ready 位...')
    mission.send_arm_joint_goal(READY_POSE, duration_sec=3.0)

    mission.get_logger().info('[Step 4] MoveIt 2: 伸向工件并闭合夹爪...')
    mission.send_arm_joint_goal(PICK_POSE, duration_sec=2.0)
    mission.set_gripper(0.005) # 闭合夹紧工件
    time.sleep(0.5)

    # ---------------------------------------------------------
    # 步骤 4: 提起工件并再次深度收折 (保证转运安全)
    # ---------------------------------------------------------
    mission.get_logger().info('[Step 5] 提起工件并回缩至 Transport 姿态...')
    mission.send_arm_joint_goal(TRANSPORT_POSE, duration_sec=3.0)

    # ---------------------------------------------------------
    # 步骤 5: Nav2 运送工件至目标下料点
    # ---------------------------------------------------------
    mission.get_logger().info('[Step 6] Nav2 启动: 携带工件转运至放置台 (X=0.0m, Y=0.0m)...')
    home_goal = PoseStamped()
    home_goal.header.frame_id = 'map'
    home_goal.header.stamp = navigator.get_clock().now().to_msg()
    home_goal.pose.position.x = 0.0
    home_goal.pose.position.y = 0.0
    home_goal.pose.orientation.w = 1.0

    navigator.goToPose(home_goal)
    while not navigator.isTaskComplete():
        time.sleep(0.3)

    # ---------------------------------------------------------
    # 步骤 6: 放置工件与任务收尾
    # ---------------------------------------------------------
    mission.get_logger().info('[Step 7] 卸载工件...')
    mission.send_arm_joint_goal(READY_POSE, duration_sec=2.5)
    mission.set_gripper(0.035) # 松开夹爪
    time.sleep(0.5)
    mission.send_arm_joint_goal(TRANSPORT_POSE, duration_sec=2.0)

    mission.get_logger().info('>>> 全流程移动抓取闭环任务顺利完成！<<<')
    rclpy.shutdown()

if __name__ == '__main__':
    main()