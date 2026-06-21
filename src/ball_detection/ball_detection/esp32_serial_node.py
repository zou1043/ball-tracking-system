#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
"""
ROS2 -> ESP32 串口桥接节点

功能概述：
1. 订阅标准速度话题 /cmd_vel（geometry_msgs/msg/Twist）
2. 按履带式差速底盘的运动学模型，将线速度/角速度换算为左右履带速度
3. 再将左右履带线速度换算成 ESP32 所需的 pps（pulse per second）
4. 通过串口向 ESP32 发送：
      m a <left_pps>\n
      m b <right_pps>\n

特别说明：
- 这里把履带车等效为“差速驱动底盘”，其左右两侧履带中心线间距为 track_width。
- ESP32 端已经做了 PID 闭环和电机极性修正，所以本节点只负责把 ROS2 的速度指令
  换算成左右履带目标脉冲速度。
- 编码器“每转脉冲数”无法从现有文件中自动推断，因此做成参数
  encoder_pulses_per_wheel_rev，默认值给了一个常见参考值 390，请按实测标定。
"""

import math
import queue
import threading
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from serial import Serial, SerialException
from std_msgs.msg import Int32


class ESP32SerialNode(Node):
    """将 /cmd_vel 转换为 ESP32 双电机 pps 串口指令的 ROS2 节点。"""

    def __init__(self) -> None:
        super().__init__('esp32_serial_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('cmd_vel_topic', '/cmd_vel'),
                ('port', '/dev/ttyUSB0'),
                ('baudrate', 115200),
                ('serial_timeout', 0.05),
                ('reconnect_interval', 2.0),
                ('command_period', 0.05),             # 20Hz 周期性下发，避免底盘保持旧速度
                ('cmd_vel_timeout', 0.50),            # 超时后自动发 0，防止 ROS2 上位机卡住后继续跑车
                ('track_width', 0.115),               # 履带中心距，单位 m
                ('wheel_diameter', 0.042),            # 主动轮直径，单位 m
                ('encoder_pulses_per_wheel_rev', 390.0),
                ('servo1_topic', '/servo1_angle'),
                ('servo2_topic', '/servo2_angle'),
                ('left_command_sign', 1),
                ('right_command_sign', 1),
                ('max_abs_pps', 4000),
                ('print_received_serial', False),
            ],
        )

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.command_period = float(self.get_parameter('command_period').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.encoder_pulses_per_wheel_rev = float(
            self.get_parameter('encoder_pulses_per_wheel_rev').value
        )
        self.servo1_topic = self.get_parameter('servo1_topic').value
        self.servo2_topic = self.get_parameter('servo2_topic').value
        self.left_command_sign = int(self.get_parameter('left_command_sign').value)
        self.right_command_sign = int(self.get_parameter('right_command_sign').value)
        self.max_abs_pps = int(self.get_parameter('max_abs_pps').value)
        self.print_received_serial = bool(
            self.get_parameter('print_received_serial').value
        )

        if self.wheel_diameter <= 0.0:
            raise ValueError('wheel_diameter 必须大于 0')
        if self.track_width <= 0.0:
            raise ValueError('track_width 必须大于 0')
        if self.encoder_pulses_per_wheel_rev <= 0.0:
            raise ValueError('encoder_pulses_per_wheel_rev 必须大于 0')

        self._latest_linear = 0.0
        self._latest_angular = 0.0
        self._last_cmd_vel_time = time.monotonic()

        self._last_sent_command: Optional[Tuple[int, int]] = None
        self._command_queue: queue.Queue[Tuple[int, int]] = queue.Queue(maxsize=1)
        self._raw_command_queue: queue.Queue[bytes] = queue.Queue(maxsize=8)
        self._serial: Optional[Serial] = None
        self._serial_lock = threading.Lock()
        self._stop_event = threading.Event()

        self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._cmd_vel_callback,
            10,
        )
        self.create_subscription(
            Int32,
            self.servo1_topic,
            self._servo1_callback,
            10,
        )
        self.create_subscription(
            Int32,
            self.servo2_topic,
            self._servo2_callback,
            10,
        )
        self.create_timer(self.command_period, self._command_timer_callback)

        self._serial_thread = threading.Thread(
            target=self._serial_worker,
            name='esp32-serial-worker',
            daemon=True,
        )
        self._serial_thread.start()

        self.get_logger().info(
            'esp32_serial_node started: '
            f'cmd_vel_topic={self.cmd_vel_topic}, port={self.port}, baudrate={self.baudrate}, '
            f'track_width={self.track_width:.3f}m, wheel_diameter={self.wheel_diameter:.3f}m, '
            f'encoder_pulses_per_wheel_rev={self.encoder_pulses_per_wheel_rev:.1f}'
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """接收 ROS2 速度指令，保存最新线速度和角速度。"""
        self._latest_linear = float(msg.linear.x)
        self._latest_angular = float(msg.angular.z)
        self._last_cmd_vel_time = time.monotonic()

    def _command_timer_callback(self) -> None:
        """
        定时根据最近一次 /cmd_vel 计算左右 pps。

        为什么不在回调里直接写串口：
        - 串口写入可能阻塞
        - ROS2 回调线程应尽量短小
        - 因此这里仅计算并把最新命令塞到队列，由独立线程收发串口
        """
        now = time.monotonic()
        if now - self._last_cmd_vel_time > self.cmd_vel_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self._latest_linear
            angular = self._latest_angular

        left_pps, right_pps = self._twist_to_pps(linear, angular)
        self._enqueue_latest_command((left_pps, right_pps))

    def _servo1_callback(self, msg: Int32) -> None:
        """杞彂舵満 1 瑙掑害鍛戒护锛?s 1 <angle>銆?"""
        self._enqueue_raw_serial_command(self._build_servo_packet(1, int(msg.data)))

    def _servo2_callback(self, msg: Int32) -> None:
        """杞彂舵満 2 瑙掑害鍛戒护锛?s 2 <angle>銆?"""
        self._enqueue_raw_serial_command(self._build_servo_packet(2, int(msg.data)))

    def _twist_to_pps(self, linear_x: float, angular_z: float) -> Tuple[int, int]:
        """
        将底盘速度 (v, w) 逆解为左右履带 pps。

        1. 差速底盘左右两侧线速度公式
           v_left  = v - w * B / 2
           v_right = v + w * B / 2

           其中：
           - v 是底盘前进线速度（m/s）
           - w 是底盘绕 z 轴角速度（rad/s）
           - B 是左右履带中心距，也就是 track_width（m）

        2. 履带线速度 -> 主动轮转速
           每转一圈前进的线距离 = 轮周长 = pi * D
           rev_per_sec = v_side / (pi * D)

           其中 D 是驱动轮直径（m）

        3. 主动轮转速 -> 编码器脉冲速度
           pps = rev_per_sec * encoder_pulses_per_wheel_rev

           注意：
           encoder_pulses_per_wheel_rev 指“主动轮转一圈，对应编码器计数多少个脉冲”。
           这个值与编码器线数、减速比、ESP32 使用 HalfQuad/FullQuad 的计数方式都有关，
           现场一定要按实物标定。
        """
        half_track = self.track_width * 0.5
        left_linear = linear_x - angular_z * half_track
        right_linear = linear_x + angular_z * half_track

        wheel_circumference = math.pi * self.wheel_diameter
        left_rev_per_sec = left_linear / wheel_circumference
        right_rev_per_sec = right_linear / wheel_circumference

        left_pps = left_rev_per_sec * self.encoder_pulses_per_wheel_rev
        right_pps = right_rev_per_sec * self.encoder_pulses_per_wheel_rev

        left_pps = int(round(left_pps * self.left_command_sign))
        right_pps = int(round(right_pps * self.right_command_sign))

        left_pps = max(-self.max_abs_pps, min(self.max_abs_pps, left_pps))
        right_pps = max(-self.max_abs_pps, min(self.max_abs_pps, right_pps))

        return left_pps, right_pps

    def _enqueue_latest_command(self, command: Tuple[int, int]) -> None:
        """
        队列只保留“最新”一条串口命令，避免串口线程落后时积压老指令。
        """
        try:
            while True:
                self._command_queue.get_nowait()
        except queue.Empty:
            pass

        try:
            self._command_queue.put_nowait(command)
        except queue.Full:
            # 理论上前面已经清空，仍保底处理一次。
            pass

    def _enqueue_raw_serial_command(self, packet: bytes) -> None:
        """鍏ュ垪涓嶄笌 /cmd_vel 鍚堝苟鐨勫師濮嬩覆鍙ｅ懡浠ゃ€?"""
        try:
            self._raw_command_queue.put_nowait(packet)
        except queue.Full:
            try:
                self._raw_command_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._raw_command_queue.put_nowait(packet)
            except queue.Full:
                pass

    @staticmethod
    def _build_servo_packet(servo_id: int, angle: int) -> bytes:
        safe_angle = max(0, min(180, angle))
        return f's {servo_id} {safe_angle}\n'.encode('utf-8')

    def _serial_worker(self) -> None:
        """独立线程：负责串口连接、重连、写入最新 pps 指令和读取下位机回显。"""
        while not self._stop_event.is_set():
            if self._serial is None:
                self._try_connect_serial()
                if self._serial is None:
                    time.sleep(self.reconnect_interval)
                    continue

            try:
                command = self._fetch_latest_command(timeout=self.serial_timeout)
                if command is not None:
                    self._write_motor_command(command)

                self._write_pending_raw_commands()
                self._read_serial_feedback()
            except SerialException as exc:
                self.get_logger().error(f'串口通信异常，准备重连: {exc}')
                self._close_serial()
                time.sleep(self.reconnect_interval)
            except Exception as exc:  # 保底，避免线程意外退出
                self.get_logger().error(f'串口线程异常，准备重连: {exc}')
                self._close_serial()
                time.sleep(self.reconnect_interval)

        self._close_serial()

    def _try_connect_serial(self) -> None:
        """尝试建立串口连接。"""
        try:
            with self._serial_lock:
                self._serial = Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.serial_timeout,
                    write_timeout=self.serial_timeout,
                )
            self.get_logger().info(f'ESP32 serial connected: {self.port} @ {self.baudrate}')
        except SerialException as exc:
            self._serial = None
            self.get_logger().warn(f'串口连接失败 {self.port}: {exc}')

    def _fetch_latest_command(self, timeout: float) -> Optional[Tuple[int, int]]:
        """
        获取待发送命令，并在多条积压时只发送最后一条。
        """
        try:
            command = self._command_queue.get(timeout=timeout)
        except queue.Empty:
            return None

        try:
            while True:
                command = self._command_queue.get_nowait()
        except queue.Empty:
            return command

    def _write_motor_command(self, command: Tuple[int, int]) -> None:
        """按 ESP32 协议发送左右履带 pps。"""
        if self._serial is None:
            return

        left_pps, right_pps = command
        if self._last_sent_command == command:
            # 虽然上层定时器一直在发最新命令，但串口线程没有必要重复写完全相同的数据。
            return

        packet_left = f'm a {left_pps}\n'.encode('utf-8')
        packet_right = f'm b {right_pps}\n'.encode('utf-8')

        with self._serial_lock:
            self._serial.write(packet_left)
            self._serial.write(packet_right)
            self._serial.flush()

        self._last_sent_command = command

    def _write_pending_raw_commands(self) -> None:
        """鍙戦€佹墍鏈夌Н鍘嬬殑鍘熷涓插彛鍛戒护锛屼富瑕佺敤浜庤垫満銆?"""
        if self._serial is None:
            return

        packets = []
        try:
            while True:
                packets.append(self._raw_command_queue.get_nowait())
        except queue.Empty:
            pass

        if not packets:
            return

        with self._serial_lock:
            for packet in packets:
                self._serial.write(packet)
            self._serial.flush()

    def _read_serial_feedback(self) -> None:
        """可选读取 ESP32 的串口回显，主要用于调试。"""
        if self._serial is None:
            return

        with self._serial_lock:
            if self._serial.in_waiting <= 0:
                return
            raw = self._serial.readline()

        if not raw:
            return

        line = raw.decode('utf-8', errors='ignore').strip()
        if line and self.print_received_serial:
            self.get_logger().info(f'ESP32: {line}')

    def _close_serial(self) -> None:
        """关闭串口资源。"""
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
        self._last_sent_command = None

    def destroy_node(self) -> bool:
        """
        退出节点前，尽量给下位机发一次 0 速，避免 ROS2 退出后底盘保持旧速度。
        """
        try:
            self._enqueue_latest_command((0, 0))
            time.sleep(min(0.10, self.command_period))
        except Exception:
            pass

        self._stop_event.set()
        if self._serial_thread.is_alive():
            self._serial_thread.join(timeout=1.0)
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ESP32SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('esp32_serial_node stopped by keyboard')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
