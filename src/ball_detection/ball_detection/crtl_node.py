#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from ball_interfaces.msg import BallInfo


class VisionServoController(Node):
    """
    基于 ball_info 的简易视觉伺服控制节点。

    控制目标：
    - 图像分辨率按 640x480 处理
    - 目标期望位置为 (320, 384)
    - 只跟踪类别为 red 的目标
    - 使用 P 控制器输出 cmd_vel
    """

    def __init__(self):
        super().__init__('crtl_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_class', 'red'),
                ('image_center_x', 320.0),
                ('target_y', 384.0),
                ('kp_linear', 0.001),
                ('kp_angular', 0.001),
                ('max_linear', 0.3),
                ('max_angular', 0.5),
                ('control_period', 0.05),
                ('target_timeout', 0.2),
            ],
        )

        self.target_class = self.get_parameter('target_class').value
        self.image_center_x = float(self.get_parameter('image_center_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.control_period = float(self.get_parameter('control_period').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(BallInfo, 'ball_info', self.ball_info_callback, 10)
        self.create_timer(self.control_period, self.control_loop)

        self.current_target = None
        self.last_target_time = None

        self.get_logger().info(
            'crtl_node started: target_class=%s, center_x=%.1f, target_y=%.1f'
            % (self.target_class, self.image_center_x, self.target_y)
        )

    def ball_info_callback(self, msg: BallInfo):
        """
        从当前检测结果中选出 red 目标。

        由于 BallInfo 当前只提供 positions/classes，不含面积信息，
        这里采用“离期望控制点最近”的 red 目标作为控制对象。
        """
        red_targets = []
        for point, class_name in zip(msg.positions, msg.classes):
            if class_name == self.target_class:
                red_targets.append(point)

        if not red_targets:
            self.current_target = None
            self.last_target_time = None
            return

        self.current_target = min(
            red_targets,
            key=lambda p: abs(p.x - self.image_center_x) + abs(p.y - self.target_y),
        )
        self.last_target_time = self.get_clock().now()

    def control_loop(self):
        """
        视觉伺服主循环。

        误差定义：
        - error_x = image_center_x - target_x
          当目标在画面左侧时，error_x > 0，输出正角速度
        - error_y = target_y - target_y_measured
          当目标高于期望位置时，说明通常更远，输出正线速度前进
        """
        cmd = Twist()

        if self.current_target is None or self.last_target_time is None:
            self.cmd_vel_pub.publish(cmd)
            return

        dt = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if dt > self.target_timeout:
            self.current_target = None
            self.last_target_time = None
            self.cmd_vel_pub.publish(cmd)
            return

        error_x = self.image_center_x - float(self.current_target.x)
        error_y = self.target_y - float(self.current_target.y)

        cmd.angular.z = self.clamp(
            self.kp_angular * error_x,
            -self.max_angular,
            self.max_angular,
        )
        cmd.linear.x = self.clamp(
            self.kp_linear * error_y,
            -self.max_linear,
            self.max_linear,
        )

        self.get_logger().info(
            f"追踪红球 -> 线速度: {cmd.linear.x:.2f}, 角速度: {cmd.angular.z:.2f}"
        )
        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))


def main(args=None):
    rclpy.init(args=args)
    node = VisionServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
