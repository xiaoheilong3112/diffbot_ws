import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_diffbot_nav2 = get_package_share_directory('diffbot_nav2')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(pkg_diffbot_nav2, 'maps', 'room.yaml'))
    nav2_params_path = LaunchConfiguration('params_file', default=os.path.join(pkg_diffbot_nav2, 'config', 'nav2_params.yaml'))
    world_path = LaunchConfiguration('world', default=os.path.join(pkg_diffbot_nav2, 'world', 'diffbot_world.sdf'))

    # 1. 解析 Xacro 生成 Robot Description (底盘 + 4轴机械臂 + IMU + 激光雷达)
    xacro_file = os.path.join(pkg_diffbot_nav2, 'urdf', 'diffbot_arm.urdf.xacro')
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    # 1.1 MoveIt 2 配置 (SRDF / IK / OMPL / 控制器映射)
    moveit_config = (
        MoveItConfigsBuilder('diffbot_arm', package_name='diffbot_nav2')
        .robot_description(file_path='urdf/diffbot_arm.urdf.xacro')
        .robot_description_semantic(file_path='config/diffbot_arm.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    # 2. 节点: robot_state_publisher (发布机器各 link TF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # 3. 启动 Gazebo Sim 场景世界并自动运行 (-r)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 4. 在 Gazebo 中生成 (Spawn) 机器人模型
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'diffbot',
                   '-z', '0.1']
    )

    # 4.1 机器人生成完毕后加载 ros2_control 控制器 (状态广播 + 机械臂 + 夹爪)
    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['arm_controller', '--controller-manager-timeout', '120'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['gripper_controller', '--controller-manager-timeout', '120'],
                    output='screen'
                ),
            ]
        )
    )

    # 5. ros_gz_bridge: 双向桥接 /clock, /scan, /cmd_vel, /odom/unfiltered, /imu/data
    bridge_config = os.path.join(pkg_diffbot_nav2, 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 6. EKF 状态估计节点 (参考 linorobot2 架构：融合轮速与 IMU，发布 odom -> base_footprint)
    ekf_config = os.path.join(pkg_diffbot_nav2, 'config', 'ekf.yaml')
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    # 7. 引入 Nav2 Bringup (输入滤波后的 /odometry/filtered)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items()
    )

    # 7.1 MoveIt 2 move_group (机械臂运动规划/执行, 供 RViz MotionPlanning 面板与场景B调度)
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': use_sim_time,
                'allow_trajectory_execution': True,
                'moveit_manage_controllers': False,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
            },
        ],
    )

    # 8. 启动可视化 RViz2 (Nav2 视图 + MoveIt MotionPlanning 面板)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_diffbot_nav2, 'rviz', 'diffbot.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    # 8.1 gz 侧传感器 frame_id 为带模型前缀的作用域名，与 robot_state_publisher 的
    #     URDF 裸帧名不一致，这里补两条静态 TF 打通 (位姿取自 URDF 安装尺寸)
    #     lidar: base_link->laser_link (0.12,0,chassis_height+0.02) + base_footprint->base_link z=wheel_radius
    #     imu:   base_link->imu_link (0,0,0.05) + base_footprint->base_link z=wheel_radius
    gz_sensor_tfs = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gz_lidar_frame_tf',
            arguments=['--x', '0.12', '--y', '0', '--z', '0.22',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'diffbot/base_footprint/diffbot_lidar']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gz_imu_frame_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0.13',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'diffbot/base_footprint/diffbot_imu']
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('map', default_value=map_yaml_path, description='Full path to map yaml'),
        DeclareLaunchArgument('params_file', default_value=nav2_params_path, description='Nav2 params'),
        DeclareLaunchArgument('world', default_value=world_path, description='Gazebo world file'),
        
        node_robot_state_publisher,
        *gz_sensor_tfs,
        gz_sim,
        spawn_entity,
        spawn_controllers,
        ros_gz_bridge,
        node_ekf,
        nav2_bringup,
        move_group,
        rviz_node
    ])