#!/usr/bin/env python3
"""MoveIt 2 + ros2_control 系统联调冒烟验证 (docs/moveit_2_gazebo_ros2_control_6.md §7/§8).

检查项:
  A. 前置: /move_action 与 /arm_controller/follow_joint_trajectory 可用, 控制器 active,
     节点时钟为 sim time (§8 TF_OLD_DATA 前置条件)
  B. §7.3 等价: 对 SRDF 预设位姿依次 MoveGroup 规划+执行, 到位误差/平滑性量化
  C. 可选 --gripper: 夹爪 open/close 循环 (双指同值同步)
  D. §8 排错: 执行错误码, 轨迹容差, 关节速度峰值 (剧烈抖动检测)

用法:
  python3 moveit_smoke.py                      # arm: ready -> transport -> ready
  python3 moveit_smoke.py --pose ready --gripper
  python3 moveit_smoke.py --out /tmp/moveit_smoke.json
"""
import argparse
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             MoveItErrorCodes, PlanningOptions)
from sensor_msgs.msg import JointState

ERR_NAMES = {
    1: "SUCCESS", -1: "PLANNING_FAILED", -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED", -4: "CONTROL_FAILED", -5: "UNABLE_TO_ACHIEVE_GOAL",
    -6: "INVALID_START_STATE", -7: "INVALID_GOAL_STATE", -8: "INVALID_ROBOT_STATE",
    -10: "INVALID_GOAL_CONSTRAINTS", -11: "PATH_TOLERANCE_VIOLATION",
    -12: "GOAL_TOLERANCE_VIOLATION", 99999: "FAILURE",
}


def load_srdf_group_states():
    """解析安装的 SRDF, 返回 {group: {pose: {joint: value}}}。"""
    path = os.path.join(get_package_share_directory("diffbot_nav2"),
                        "config", "diffbot_arm.srdf")
    root = ET.parse(path).getroot()
    out = {}
    for gs in root.findall("group_state"):
        out.setdefault(gs.get("group"), {})[gs.get("name")] = {
            j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
    return out


def joint_constraints(joint_vals, tol):
    cs = Constraints()
    for name, val in joint_vals.items():
        c = JointConstraint()
        c.joint_name = name
        c.position = val
        c.tolerance_above = tol
        c.tolerance_below = tol
        c.weight = 1.0
        cs.joint_constraints.append(c)
    return cs


class Smoke(Node):
    def __init__(self):
        super().__init__("moveit_smoke", parameter_overrides=[rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])
        self.joints = {}          # name -> (pos, vel, t)
        self.joint_hist = []      # (t, {name: pos})
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)

    def _on_js(self, m):
        t = self.get_clock().now().nanoseconds / 1e9
        d = dict(zip(m.name, m.position))
        self.joints = {n: (m.position[i], m.velocity[i] if i < len(m.velocity) else 0.0)
                       for i, n in enumerate(m.name)}
        self.joint_hist.append((t, d))
        if len(self.joint_hist) > 60000:
            del self.joint_hist[:30000]

    def wait_settle(self, targets, tol=0.02, timeout=20.0, stable=0.5):
        """等待所有目标关节进入容差并保持 stable 秒 -> (ok, settle_s, errs)。"""
        t0 = time.time()
        since_ok = None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            errs = {n: abs(self.joints.get(n, (0, 0))[0] - v)
                    for n, v in targets.items()}
            if errs and max(errs.values()) <= tol:
                if since_ok is None:
                    since_ok = time.time()
                elif time.time() - since_ok >= stable:
                    return True, time.time() - t0, errs
            else:
                since_ok = None
        errs = {n: abs(self.joints.get(n, (0, 0))[0] - v)
                for n, v in targets.items()}
        return False, time.time() - t0, errs


def main():
    ap = argparse.ArgumentParser(description="MoveIt2+ros2_control 联调冒烟 (doc §7/§8)")
    ap.add_argument("--pose", default="ready,transport,ready",
                    help="arm 组位姿序列 (SRDF group_state, 逗号分隔)")
    ap.add_argument("--gripper", action="store_true", help="附带夹爪 open->close->open")
    ap.add_argument("--scale-vel", type=float, default=0.3)
    ap.add_argument("--scale-acc", type=float, default=0.2)
    ap.add_argument("--tol", type=float, default=0.02, help="到位判定容差 (rad/m)")
    ap.add_argument("--planning-time", type=float, default=5.0)
    ap.add_argument("--out", default="", help="结果 JSON 输出路径")
    args = ap.parse_args()

    srdf = load_srdf_group_states()
    rclpy.init()
    node = Smoke()
    chk, runs = {}, []

    # ---- A. 前置检查 (§5/§8) ----
    mg = ActionClient(node, MoveGroup, "move_action")
    chk["move_action 可用"] = mg.wait_for_server(timeout_sec=30.0)
    fjt = ActionClient(node, FollowJointTrajectory, "arm_controller/follow_joint_trajectory")
    chk["arm FollowJT action 存在 (§8-2)"] = fjt.wait_for_server(timeout_sec=5.0)
    fjt_g = ActionClient(node, FollowJointTrajectory, "gripper_controller/follow_joint_trajectory")
    chk["gripper FollowJT action 存在"] = fjt_g.wait_for_server(timeout_sec=5.0)

    lc = node.create_client(ListControllers, "/controller_manager/list_controllers")
    ctl_ok = False
    if lc.wait_for_service(timeout_sec=10.0):
        fut = lc.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        if fut.result():
            active = [c.name for c in fut.result().controller if c.state == "active"]
            want = {"joint_state_broadcaster", "arm_controller", "gripper_controller"}
            ctl_ok = want.issubset(set(active))
            chk[f"控制器 active (§7.2) {sorted(want & set(active))}"] = ctl_ok
    else:
        chk["控制器 active (§7.2)"] = False

    t_sim = node.get_clock().now().nanoseconds / 1e9
    use_st = node.get_parameter("use_sim_time").get_parameter_value().bool_value
    # sim time: 参数开启 且 时钟远小于壁钟 epoch (2026 ~1.79e9); gz 世界时钟从 0 起步
    chk["节点使用 sim time (§8-3)"] = bool(use_st and t_sim < 1e8)
    print(f"[pre] sim time = {t_sim:.1f} (use_sim_time={use_st})")

    for k, v in chk.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
    if not all(chk.values()):
        print("[abort] 前置检查未全过")
        rclpy.shutdown()
        sys.exit(2)

    # ---- B. arm 规划+执行序列 (§7.3) ----
    def run_group(group, pose):
        targets = srdf[group][pose]
        req = MotionPlanRequest()
        req.group_name = group
        req.planner_id = "ompl"
        req.num_planning_attempts = 10
        req.allowed_planning_time = args.planning_time
        req.max_velocity_scaling_factor = args.scale_vel
        req.max_acceleration_scaling_factor = args.scale_acc
        req.pipeline_id = "ompl"
        req.start_state.is_diff = True          # 从当前状态出发
        # 夹爪行程仅 0~0.035, 容差须远小于行程, 否则规划器在带内"合法"取偏值
        gtol = 0.003 if group == "gripper" else 0.005
        req.goal_constraints.append(joint_constraints(targets, gtol))
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        h0 = len(node.joint_hist)
        t_send = time.time()
        fut = mg.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        handle = fut.result() if fut.done() else None
        if not handle or not handle.accepted:
            runs.append({"group": group, "pose": pose, "error": "goal rejected"})
            print(f"[{group}/{pose}] goal rejected")
            return
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=90.0)
        t_done = time.time() - t_send
        code = res_fut.result().result.error_code.val if res_fut.done() else -99999
        stol = 0.005 if group == "gripper" else args.tol
        ok_settle, t_settle, errs = node.wait_settle(targets, tol=stol, timeout=20.0)

        seg = node.joint_hist[h0:]
        # 关节角速度峰值 (位置差分, 抖动/剧烈运动检测 §8-4)
        step_max = 0.0
        for i in range(1, len(seg)):
            dt = seg[i][0] - seg[i - 1][0]
            if dt <= 1e-4:
                continue
            for n in targets:
                dq = abs(seg[i][1].get(n, 0.0) - seg[i - 1][1].get(n, 0.0))
                step_max = max(step_max, dq / dt)
        run = {"group": group, "pose": pose,
               "error_code": code, "error": ERR_NAMES.get(code, str(code)),
               "exec_s": round(t_done, 2), "settle_s": round(t_settle, 2),
               "final_err": {k: round(v, 4) for k, v in errs.items()},
               "max_joint_rate": round(step_max, 3)}
        run["ok"] = (code == MoveItErrorCodes.SUCCESS and ok_settle)
        runs.append(run)
        print(f"[{group}/{pose}] {run['error']} exec={run['exec_s']}s "
              f"settle={run['settle_s']}s max_err={max(errs.values()):.4f} "
              f"rate={step_max:.3f}/s ok={run['ok']}")

    for pose in args.pose.split(","):
        run_group("arm", pose.strip())
    if args.gripper:
        for pose in ("open", "close", "open"):
            run_group("gripper", pose)

    # ---- 汇总 ----
    all_ok = all(r.get("ok") for r in runs) and all(chk.values())
    print("\n== MoveIt 联调冒烟 (doc §7/§8) ==")
    for k, v in chk.items():
        print(f"  [{'x' if v else ' '}] {k}")
    for r in runs:
        print(f"  [{'x' if r.get('ok') else ' '}] {r['group']}/{r['pose']}: "
              f"{r.get('error')} exec={r.get('exec_s')}s err_max="
              f"{max(r['final_err'].values()) if r.get('final_err') else float('nan')}")
    print(f"合计: {'PASS' if all_ok else 'FAIL'}")

    if args.out:
        with open(args.out, "w") as f:
            json.dump({"checklist": chk, "runs": runs, "pass": all_ok}, f,
                      indent=1, ensure_ascii=False)
        print(f"-> {args.out}")

    rclpy.shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
