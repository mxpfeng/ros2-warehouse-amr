#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
到达目标后，计算 目标位姿(/goal_pose) 与 真值位姿(/gazebo/model_states)
之间的 x / y / yaw 差值。

触发方式：监听 /rosout，捕获
  controller_server : "Reached the goal!"
  bt_navigator      : "Goal succeeded"
两者都出现后，用到达时刻附近的真值位姿做一次对比并打印。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from rcl_interfaces.msg import Log


def quat_to_yaw(q) -> float:
    """四元数 -> yaw (rad)，范围 (-pi, pi]"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a: float) -> float:
    """归一化到 (-pi, pi]"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class GoalErrorChecker(Node):
    def __init__(self):
        super().__init__('goal_error_checker')

        # ---- 参数 ----
        self.declare_parameter('model_name', 'warehouse_amr')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('reached_window_sec', 2.0)   # 两条 INFO 允许的最大间隔
        self.declare_parameter('print_on_goal_pose', True)  # 每次收到新目标也打印

        self.model_name = self.get_parameter('model_name').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.ms_topic = self.get_parameter('model_states_topic').value
        self.window = self.get_parameter('reached_window_sec').value
        self.print_on_goal = self.get_parameter('print_on_goal_pose').value

        # ---- 状态 ----
        self.goal_xy = None        # (x, y, yaw)
        self.gt_xy = None          # (x, y, yaw)
        self.gt_stamp = None       # 最近一帧真值时间
        self.reached_stamp = None  # controller_server 报告到达的时间(秒)
        self.reported = False      # 本次目标已输出过，避免重复刷屏

        # ---- 订阅 ----
        qos_best = QoSProfile(depth=10)
        qos_best.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_best.durability = DurabilityPolicy.VOLATILE

        self.sub_goal = self.create_subscription(
            PoseStamped, self.goal_topic, self.cb_goal, 10)
        self.sub_ms = self.create_subscription(
            ModelStates, self.ms_topic, self.cb_model_states, qos_best)
        self.sub_log = self.create_subscription(
            Log, '/rosout', self.cb_log, 10)

        self.get_logger().info(
            'goal_error_checker started: model=%s goal=%s ms=%s'
            % (self.model_name, self.goal_topic, self.ms_topic))

    # ---------------- 回调 ----------------
    def cb_goal(self, msg: PoseStamped):
        yaw = quat_to_yaw(msg.pose.orientation)
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.reported = False
        if self.print_on_goal:
            self.get_logger().info(
                'NEW GOAL  x=%.4f y=%.4f yaw=%.4f' % self.goal_xy)

    def cb_model_states(self, msg: ModelStates):
        if self.model_name not in msg.name:
            return
        i = msg.name.index(self.model_name)
        p = msg.pose[i].position
        q = msg.pose[i].orientation
        self.gt_xy = (p.x, p.y, quat_to_yaw(q))
        self.gt_stamp = self.get_clock().now().nanoseconds / 1e9

        # 如果已经收到“到达”且还没输出，就用最新真值算一次
        if self.reached_stamp is not None and not self.reported:
            # 只使用“到达时刻之后”的真值，避免用到旧数据
            if self.gt_stamp >= self.reached_stamp:
                self.try_report(trigger='gt')

    def cb_log(self, msg: Log):
        if msg.name == 'controller_server' and 'Reached the goal' in msg.msg:
            self.reached_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            self.get_logger().info('[detect] controller_server: Reached the goal!')
        if (msg.name == 'bt_navigator'
                and 'Goal succeeded' in msg.msg
                and self.reached_stamp is not None
                and not self.reported):
            now = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            if now - self.reached_stamp <= self.window:
                self.try_report(trigger='bt')

    # ---------------- 计算与打印 ----------------
    def try_report(self, trigger: str = ''):
        if self.goal_xy is None or self.gt_xy is None:
            return
        gx, gy, gyaw = self.goal_xy
        ax, ay, ayaw = self.gt_xy
        dx = gx - ax
        dy = gy - ay
        dyaw = norm_angle(gyaw - ayaw)
        dist = math.hypot(dx, dy)

        line = (
            "\n================== GOAL REACHED (GT) ==================\n"
            "expected goal : x=%9.4f y=%9.4f yaw=%7.4f\n"
            "actual  (gt)  : x=%9.4f y=%9.4f yaw=%7.4f\n"
            "error(exp-act): dx=%+8.4f dy=%+8.4f dyaw=%+7.4f dist=%.4f m\n"
            "======================================================="
            % (gx, gy, gyaw, ax, ay, ayaw, dx, dy, dyaw, dist)
        )
        self.get_logger().info(line)
        print(line, flush=True)
        self.reported = True


def main(args=None):
    rclpy.init(args=args)
    node = GoalErrorChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()