#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
"""
视觉跟随控制主脑节点

设计目标：
1. 订阅 detect_node.py 当前已经发布的 ball_info 话题
2. 在红 / 黄 / 黑三种球中按照优先级：
      红 > 黄 > 黑
   选择一个当前要跟踪的目标
3. 根据目标在图像中的横向偏差，发布角速度 angular.z 调整航向
4. 根据目标“远近”发布线速度 linear.x，使小车停在约 0.5m 附近
5. 目标完全丢失时，原地自旋寻找

与当前 detect_node.py 的接口兼容说明：
- 现有 BallInfo.msg 只有：
      geometry_msgs/Point[] positions
      string[] classes
- 其中 detect_node.py 当前写法里：
      point.x = 目标框中心 x
      point.y = 目标框底边 bottom
      point.z = 0.0

也就是说：
- 现在并没有直接发布 bbox 面积 area
- 但是 point.y 越大，通常代表球越靠近画面下方，也往往越近
- 所以本节点做了“双模式兼容”：
  1. 如果未来你的 detect_node 把 point.z 填成 bbox 面积，那么本节点优先用面积控制距离
  2. 如果 point.z 还是 0，则退化为使用 point.y（bbox 底边）作为单目距离代理量

这样你现在不用改原 detect_node.py，也能先跑通整套跟随闭环。
"""

import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from ball_interfaces.msg import BallInfo
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass
class TargetCandidate:
    """保存单个候选球的视觉信息。"""

    class_name: str
    x: float
    y: float
    area: float
    size_metric: float


class BallTrackLogic(Node):
    """根据视觉检测结果输出 /cmd_vel 的跟随控制节点。"""

    def __init__(self) -> None:
        super().__init__('ball_track_logic')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ball_info_topic', 'ball_info'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('control_period', 0.05),
                ('image_width', 640.0),
                ('image_height', 480.0),
                ('target_center_x', -1.0),
                ('target_area', 12000.0),            # 若未来 detect_node 提供面积，可在此设成 0.5m 对应面积
                ('target_bottom_y', 340.0),          # 现有 detect_node 下，0.5m 对应的 bbox 底边像素位置
                ('area_deadband', 1200.0),
                ('bottom_y_deadband', 12.0),
                ('angular_kp', 0.0060),              # 像素误差 -> 角速度(rad/s)
                ('linear_kp_area', 0.00004),         # 面积误差 -> 线速度(m/s)
                ('linear_kp_bottom_y', 0.0030),      # bottom_y 误差 -> 线速度(m/s)
                ('heading_deadband_px', 12.0),
                ('forward_heading_limit_px', 180.0), # 偏差过大时先原地对准，不前进
                ('max_linear_speed', 0.28),
                ('max_angular_speed', 1.80),
                ('lost_timeout', 0.35),
                ('search_angular_speed', 1.00),
                ('use_area_if_available', True),
                ('priority_order', ['red', 'yellow', 'black']),
            ],
        )

        self.ball_info_topic = self.get_parameter('ball_info_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_period = float(self.get_parameter('control_period').value)
        self.image_width = float(self.get_parameter('image_width').value)
        self.image_height = float(self.get_parameter('image_height').value)
        self.target_center_x = float(self.get_parameter('target_center_x').value)
        if self.target_center_x < 0.0:
            self.target_center_x = self.image_width * 0.5
        self.target_area = float(self.get_parameter('target_area').value)
        self.target_bottom_y = float(self.get_parameter('target_bottom_y').value)
        self.area_deadband = float(self.get_parameter('area_deadband').value)
        self.bottom_y_deadband = float(self.get_parameter('bottom_y_deadband').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.linear_kp_area = float(self.get_parameter('linear_kp_area').value)
        self.linear_kp_bottom_y = float(self.get_parameter('linear_kp_bottom_y').value)
        self.heading_deadband_px = float(self.get_parameter('heading_deadband_px').value)
        self.forward_heading_limit_px = float(
            self.get_parameter('forward_heading_limit_px').value
        )
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.use_area_if_available = bool(self.get_parameter('use_area_if_available').value)

        priority_order = list(self.get_parameter('priority_order').value)
        self.priority_map: Dict[str, int] = {
            class_name: index for index, class_name in enumerate(priority_order)
        }

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(BallInfo, self.ball_info_topic, self._ball_info_callback, 10)
        self.create_timer(self.control_period, self._control_timer_callback)

        self._current_target: Optional[TargetCandidate] = None
        self._last_target: Optional[TargetCandidate] = None
        self._last_seen_time = 0.0
        self._last_search_direction = 1.0

        self.get_logger().info(
            'ball_track_logic started: '
            f'ball_info_topic={self.ball_info_topic}, cmd_vel_topic={self.cmd_vel_topic}, '
            f'priority={priority_order}'
        )

    def _ball_info_callback(self, msg: BallInfo) -> None:
        """接收视觉节点输出，从所有球中选出当前最该跟踪的那个。"""
        candidates = self._extract_candidates(msg)
        if not candidates:
            return

        selected = self._select_target(candidates)
        self._current_target = selected
        self._last_target = selected
        self._last_seen_time = time.monotonic()
        self._last_search_direction = 1.0 if selected.x <= self.target_center_x else -1.0

    def _extract_candidates(self, msg: BallInfo) -> List[TargetCandidate]:
        """
        从 BallInfo 中提取红/黄/黑三类球。

        当前 detect_node 没有显式发布面积，因此这里约定：
        - 若 point.z > 0，则把它视作 bbox 面积
        - 否则使用 point.y 作为“近大远小”的代理量
        """
        candidates: List[TargetCandidate] = []

        for point, class_name in zip(msg.positions, msg.classes):
            if class_name not in self.priority_map:
                continue

            area = float(point.z) if float(point.z) > 0.0 else 0.0
            size_metric = area if area > 0.0 else float(point.y)

            candidates.append(
                TargetCandidate(
                    class_name=class_name,
                    x=float(point.x),
                    y=float(point.y),
                    area=area,
                    size_metric=size_metric,
                )
            )

        return candidates

    def _select_target(self, candidates: List[TargetCandidate]) -> TargetCandidate:
        """
        目标选择策略：
        1. 先按颜色优先级排序：红 > 黄 > 黑
        2. 同一优先级下，选择“更大/更近”的那个
           - 若有真实 area，选 area 最大
           - 若没有 area，则退化为选 y 最大（更靠近画面下方）
        """
        return sorted(
            candidates,
            key=lambda item: (
                self.priority_map[item.class_name],
                -item.size_metric,
            ),
        )[0]

    def _control_timer_callback(self) -> None:
        """固定频率发布 /cmd_vel，形成连续控制。"""
        now = time.monotonic()
        cmd = Twist()

        if self._current_target is not None and (now - self._last_seen_time) <= self.lost_timeout:
            cmd = self._build_tracking_cmd(self._current_target)
        else:
            # 目标完全丢失，原地自旋搜索
            cmd.linear.x = 0.0
            cmd.angular.z = self.search_angular_speed * self._last_search_direction

        self.cmd_pub.publish(cmd)

    def _build_tracking_cmd(self, target: TargetCandidate) -> Twist:
        """
        根据目标位置生成跟随速度。

        转向逻辑：
        - error_x = image_center_x - target_x
        - 当球在图像左边时，target_x < center_x，因此 error_x > 0
        - 输出正 angular.z，表示车体左转
        - 差速到底盘上就是“右履带更快，左履带更慢”

        这正好符合你的要求：
        - 小球偏左 -> 输出正角速度
        - 小球偏右 -> 输出负角速度

        距离逻辑：
        - 若 area 可用：面积越小表示越远，因此 area 小于目标面积时向前
        - 若 area 不可用：使用 bbox bottom_y 作为替代，bottom_y 越小表示目标越远
        """
        cmd = Twist()

        error_x = self.target_center_x - target.x
        if abs(error_x) <= self.heading_deadband_px:
            angular_z = 0.0
        else:
            angular_z = self.angular_kp * error_x

        angular_z = self._clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)

        linear_x = self._compute_forward_speed(target, abs(error_x))
        linear_x = self._clamp(linear_x, -self.max_linear_speed, self.max_linear_speed)

        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        return cmd

    def _compute_forward_speed(self, target: TargetCandidate, abs_heading_error_px: float) -> float:
        """
        计算前进线速度。

        一个很重要的工程细节：
        - 如果目标偏离画面中心太多，先转向，暂时不前进
        - 这样能避免履带车边冲边拐，把球甩出视野
        """
        if abs_heading_error_px > self.forward_heading_limit_px:
            return 0.0

        if self.use_area_if_available and target.area > 0.0:
            # 面积小 -> 远 -> 往前走
            area_error = self.target_area - target.area
            if area_error <= self.area_deadband:
                return 0.0
            return self.linear_kp_area * area_error

        # 当前 detect_node 兼容模式：使用 bbox 底边 y 估计远近
        # bottom_y 小 -> 球更远 -> 往前走
        y_error = self.target_bottom_y - target.y
        if y_error <= self.bottom_y_deadband:
            return 0.0
        return self.linear_kp_bottom_y * y_error

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def destroy_node(self) -> bool:
        """退出前补发一次零速度，让底盘停稳。"""
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallTrackLogic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ball_track_logic stopped by keyboard')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
