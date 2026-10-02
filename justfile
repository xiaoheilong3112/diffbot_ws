# diffbot_ws 工作区入口
# 依赖: ROS 2 Jazzy (/opt/ros/jazzy, 外置盘 loop 挂载) + systemd user unit diffbot-sim.service

default: list

# 列出所有 recipe
list:
    @just --list

# 构建工作区 (symlink install)
build:
    source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

# 只构建 diffbot_nav2
build-pkg:
    source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select diffbot_nav2

# 启动/重启仿真 (Gazebo + Nav2 + MoveIt + RViz)
sim:
    systemctl --user restart diffbot-sim.service

# 停止仿真
stop:
    systemctl --user stop diffbot-sim.service

# 查看 unit 状态
status:
    systemctl --user status diffbot-sim.service --no-pager

# 跟踪仿真日志
log:
    tail -n 100 -f {{env_var("HOME")}}/.ros/diffbot-sim.log

# 彻底清理: SIGKILL unit + 清 FastDDS SHM (防僵尸 participant 导致新节点创建死锁)
clean:
    systemctl --user kill -s SIGKILL diffbot-sim.service || true
    sleep 2
    systemctl --user reset-failed diffbot-sim.service || true
    rm -f /dev/shm/fastrtps_*
    @echo "cleaned (restart with: just sim)"

# 健康检查: unit + 控制器 + 关节状态 + 规划接口
check:
    #!/usr/bin/env bash
    echo "== unit: $(systemctl --user is-active diffbot-sim.service)"
    source /opt/ros/jazzy/setup.bash
    source "{{justfile_directory()}}/install/setup.bash"
    export ROS_DOMAIN_ID=0
    echo "== controllers:"
    timeout -s KILL 25 ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers 2>/dev/null \
        | grep -oE "name='[^']+', state='[^']+'" || echo "(controller_manager 不可达)"
    echo "== joint_states:"
    timeout -s KILL 20 ros2 topic echo /joint_states --once --field name 2>/dev/null || echo "(无 joint_states)"
    echo "== move_group:"
    timeout -s KILL 20 ros2 action list 2>/dev/null | grep -E "move_action|execute_trajectory" || echo "(move_group 不可达)"
