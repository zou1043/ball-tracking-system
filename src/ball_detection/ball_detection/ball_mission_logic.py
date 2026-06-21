#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from ball_interfaces.msg import BallInfo
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32


@dataclass
class VisionCandidate:
    class_name: str
    x: float
    y: float
    size_metric: float


class BallMissionLogic(Node):
    STARTUP_FORWARD = 'startup_forward'
    SEARCH_BALL = 'search_ball'
    APPROACH_BALL = 'approach_ball'
    GRAB_BALL = 'grab_ball'
    SEARCH_SAFE = 'search_safe'
    GO_SAFE = 'go_safe'
    VERIFY_SAFE = 'verify_safe'
    PLACE_SAFE = 'place_safe'
    COMPLETE = 'complete'

    def __init__(self) -> None:
        super().__init__('ball_mission_logic')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ball_info_topic', 'ball_info'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('servo1_topic', '/servo1_angle'),
                ('servo2_topic', '/servo2_angle'),
                ('control_period', 0.05),
                ('image_width', 640.0),
                ('target_center_x', -1.0),
                ('team_color', 'red'),
                ('ball_priority_order', ['yellow', 'black', 'red', 'blue']),
                ('same_size_bin_px', 10.0),
                ('heading_deadband_px', 18.0),
                ('angular_kp', 0.010),
                ('approach_max_linear_speed', 2.0),
                ('approach_max_angular_speed', 4.0),
                ('approach_target_y', 430.0),
                ('approach_y_deadband', 10.0),
                ('near_grab_y', 190.0),
                ('near_grab_max_linear_speed', 0.5),
                ('approach_align_before_forward', True),
                ('approach_alignment_px', 20.0),
                ('approach_forward_heading_limit_px', 300.0),
                ('grab_heading_deadband_px', 20.0),
                ('grab_trigger_y', 250.0),
                ('blind_forward_speed', 0.4),
                ('blind_forward_seconds', 1.0),
                ('grab_push_heading_deadband_px', 8.0),
                ('grab_push_angular_kp', 0.004),
                ('grab_push_max_angular_speed', 0.4),
                ('grab_stable_cycles', 1),
                ('grip_hold_seconds', 1.2),
                ('ball_search_angular_speed', 1.2),
                ('safe_search_angular_speed', 4.0),
                ('safe_max_linear_speed', 1.2),
                ('safe_max_angular_speed', 4.0),
                ('safe_target_y', 240.0),
                ('safe_verify_y', 130.0),
                ('safe_verify_frames', 10),
                ('safe_verify_pass_votes', 8),
                ('safe_verify_match_x_px', 120.0),
                ('safe_verify_match_y_px', 80.0),
                ('safe_verify_timeout_seconds', 1.5),
                ('safe_reject_cooldown_seconds', 3.0),
                ('safe_y_deadband', 16.0),
                ('safe_forward_heading_limit_px', 80.0),
                ('safe_heading_deadband_px', 40.0),
                ('safe_open_before_place_seconds', 0.5),
                ('safe_place_speed', 0.6),
                ('safe_place_seconds', 1.5),
                ('safe_release_wait_seconds', 0.3),
                ('lost_timeout', 1.5),
                ('ignore_ball_in_safe_x_px', 250.0),
                ('ignore_ball_in_safe_y_px', 80.0),
                ('servo1_open_angle', 100),
                ('servo2_open_angle', 150),
                ('servo1_close_angle', 60),
                ('servo2_close_angle', 180),
                ('release_in_safe_zone', True),
                ('open_gripper_on_startup', False),
                ('startup_open_delay', 1.0),
                ('startup_forward_speed', 1.0),
                ('startup_forward_seconds', 3.0),
                ('startup_forward_angular_z', -0.05),
                ('post_release_reverse_speed', -1.2),
                ('post_release_reverse_seconds', 1.5),
                ('post_release_turn_angular_speed', 3.14),
                ('post_release_turn_seconds', 3.0),
            ],
        )

        self.ball_info_topic = self.get_parameter('ball_info_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.servo1_topic = self.get_parameter('servo1_topic').value
        self.servo2_topic = self.get_parameter('servo2_topic').value
        self.control_period = float(self.get_parameter('control_period').value)
        self.image_width = float(self.get_parameter('image_width').value)
        self.target_center_x = float(self.get_parameter('target_center_x').value)
        if self.target_center_x < 0.0:
            self.target_center_x = self.image_width * 0.5

        self.team_color = str(self.get_parameter('team_color').value)
        self.safe_class = 'redsafe' if self.team_color == 'red' else 'bluesafe'
        self.opponent_color = 'blue' if self.team_color == 'red' else 'red'
        self.ball_priority_order = [
            str(class_name)
            for class_name in self.get_parameter('ball_priority_order').value
        ]
        self.same_size_bin_px = float(self.get_parameter('same_size_bin_px').value)

        self.heading_deadband_px = float(self.get_parameter('heading_deadband_px').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.approach_max_linear_speed = float(
            self.get_parameter('approach_max_linear_speed').value
        )
        self.approach_max_angular_speed = float(
            self.get_parameter('approach_max_angular_speed').value
        )
        self.approach_target_y = float(self.get_parameter('approach_target_y').value)
        self.approach_y_deadband = float(self.get_parameter('approach_y_deadband').value)
        self.near_grab_y = float(self.get_parameter('near_grab_y').value)
        self.near_grab_max_linear_speed = float(
            self.get_parameter('near_grab_max_linear_speed').value
        )
        self.approach_align_before_forward = bool(
            self.get_parameter('approach_align_before_forward').value
        )
        self.approach_alignment_px = float(
            self.get_parameter('approach_alignment_px').value
        )
        self.approach_forward_heading_limit_px = float(
            self.get_parameter('approach_forward_heading_limit_px').value
        )
        self.grab_heading_deadband_px = float(
            self.get_parameter('grab_heading_deadband_px').value
        )
        self.grab_trigger_y = float(self.get_parameter('grab_trigger_y').value)
        self.blind_forward_speed = float(
            self.get_parameter('blind_forward_speed').value
        )
        self.blind_forward_seconds = float(
            self.get_parameter('blind_forward_seconds').value
        )
        self.grab_push_heading_deadband_px = float(
            self.get_parameter('grab_push_heading_deadband_px').value
        )
        self.grab_push_angular_kp = float(
            self.get_parameter('grab_push_angular_kp').value
        )
        self.grab_push_max_angular_speed = float(
            self.get_parameter('grab_push_max_angular_speed').value
        )
        self.grab_stable_cycles = int(self.get_parameter('grab_stable_cycles').value)
        self.grip_hold_seconds = float(self.get_parameter('grip_hold_seconds').value)
        self.ball_search_angular_speed = float(
            self.get_parameter('ball_search_angular_speed').value
        )
        self.safe_search_angular_speed = float(
            self.get_parameter('safe_search_angular_speed').value
        )
        self.safe_max_linear_speed = float(
            self.get_parameter('safe_max_linear_speed').value
        )
        self.safe_max_angular_speed = float(
            self.get_parameter('safe_max_angular_speed').value
        )
        self.safe_target_y = float(self.get_parameter('safe_target_y').value)
        self.safe_verify_y = float(self.get_parameter('safe_verify_y').value)
        self.safe_verify_frames = int(self.get_parameter('safe_verify_frames').value)
        self.safe_verify_pass_votes = int(
            self.get_parameter('safe_verify_pass_votes').value
        )
        self.safe_verify_match_x_px = float(
            self.get_parameter('safe_verify_match_x_px').value
        )
        self.safe_verify_match_y_px = float(
            self.get_parameter('safe_verify_match_y_px').value
        )
        self.safe_verify_timeout_seconds = float(
            self.get_parameter('safe_verify_timeout_seconds').value
        )
        self.safe_reject_cooldown_seconds = float(
            self.get_parameter('safe_reject_cooldown_seconds').value
        )
        self.safe_y_deadband = float(self.get_parameter('safe_y_deadband').value)
        self.safe_forward_heading_limit_px = float(
            self.get_parameter('safe_forward_heading_limit_px').value
        )
        self.safe_heading_deadband_px = float(
            self.get_parameter('safe_heading_deadband_px').value
        )
        self.safe_open_before_place_seconds = float(
            self.get_parameter('safe_open_before_place_seconds').value
        )
        self.safe_place_speed = float(self.get_parameter('safe_place_speed').value)
        self.safe_place_seconds = float(self.get_parameter('safe_place_seconds').value)
        self.safe_release_wait_seconds = float(
            self.get_parameter('safe_release_wait_seconds').value
        )
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.ignore_ball_in_safe_x_px = float(
            self.get_parameter('ignore_ball_in_safe_x_px').value
        )
        self.ignore_ball_in_safe_y_px = float(
            self.get_parameter('ignore_ball_in_safe_y_px').value
        )
        self.servo1_open_angle = int(self.get_parameter('servo1_open_angle').value)
        self.servo2_open_angle = int(self.get_parameter('servo2_open_angle').value)
        self.servo1_close_angle = int(self.get_parameter('servo1_close_angle').value)
        self.servo2_close_angle = int(self.get_parameter('servo2_close_angle').value)
        self.release_in_safe_zone = bool(self.get_parameter('release_in_safe_zone').value)
        self.open_gripper_on_startup = bool(
            self.get_parameter('open_gripper_on_startup').value
        )
        self.startup_open_delay = float(self.get_parameter('startup_open_delay').value)
        self.startup_forward_speed = float(
            self.get_parameter('startup_forward_speed').value
        )
        self.startup_forward_seconds = float(
            self.get_parameter('startup_forward_seconds').value
        )
        self.startup_forward_angular_z = float(
            self.get_parameter('startup_forward_angular_z').value
        )
        self.post_release_reverse_speed = float(
            self.get_parameter('post_release_reverse_speed').value
        )
        self.post_release_reverse_seconds = float(
            self.get_parameter('post_release_reverse_seconds').value
        )
        self.post_release_turn_angular_speed = float(
            self.get_parameter('post_release_turn_angular_speed').value
        )
        self.post_release_turn_seconds = float(
            self.get_parameter('post_release_turn_seconds').value
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.servo1_pub = self.create_publisher(Int32, self.servo1_topic, 10)
        self.servo2_pub = self.create_publisher(Int32, self.servo2_topic, 10)
        self.create_subscription(BallInfo, self.ball_info_topic, self._ball_info_callback, 10)
        self.create_timer(self.control_period, self._control_timer_callback)

        self._latest_ball_target: Optional[VisionCandidate] = None
        self._latest_safe_target: Optional[VisionCandidate] = None
        self._last_ball_seen_time = 0.0
        self._last_safe_seen_time = 0.0
        self._last_search_direction = 1.0
        self._grab_ready_cycles = 0
        self._completed_grab_cycles = 0
        self._state = self.STARTUP_FORWARD
        self._state_enter_time = time.monotonic()
        self._gripper_closed = False
        self._safe_release_sent = False
        self._safe_target_verified = False
        self._safe_verify_anchor: Optional[VisionCandidate] = None
        self._safe_verify_votes: Dict[str, int] = {'redsafe': 0, 'bluesafe': 0}
        self._safe_verify_samples = 0
        self._rejected_safe_anchor: Optional[VisionCandidate] = None
        self._rejected_safe_until = 0.0
        self._startup_open_timer = None

        self.get_logger().info(
            'ball_mission_logic started: '
            f'ball_info_topic={self.ball_info_topic}, cmd_vel_topic={self.cmd_vel_topic}, '
            f'team_color={self.team_color}, safe_class={self.safe_class}'
        )

        if self.open_gripper_on_startup:
            self._startup_open_timer = self.create_timer(
                self.startup_open_delay, self._startup_open_callback
            )

    def _ball_info_callback(self, msg: BallInfo) -> None:
        ball_candidates: List[VisionCandidate] = []
        safe_candidates: List[VisionCandidate] = []
        current_priority_map = self._get_current_priority_map()
        now = time.monotonic()

        for point, class_name in zip(msg.positions, msg.classes):
            size_metric = float(point.z) if float(point.z) > 0.0 else float(point.y)
            candidate = VisionCandidate(
                class_name=str(class_name),
                x=float(point.x),
                y=float(point.y),
                size_metric=size_metric,
            )

            if class_name == self.safe_class:
                if not self._is_rejected_safe_candidate(candidate, now):
                    safe_candidates.append(candidate)
            elif class_name in current_priority_map:
                ball_candidates.append(candidate)

        if self._state in (
            self.STARTUP_FORWARD,
            self.SEARCH_BALL,
            self.APPROACH_BALL,
            self.GRAB_BALL,
        ):
            if ball_candidates and safe_candidates:
                ball_candidates = self._filter_balls_inside_safe_zone(
                    ball_candidates, safe_candidates
                )

            if ball_candidates:
                best_priority_in_frame = min(
                    current_priority_map[candidate.class_name]
                    for candidate in ball_candidates
                )
                top_priority_balls = [
                    candidate
                    for candidate in ball_candidates
                    if current_priority_map[candidate.class_name] == best_priority_in_frame
                ]
                best_in_frame: Optional[VisionCandidate] = None

                if (
                    self._latest_ball_target is not None
                    and (now - self._last_ball_seen_time) <= self.lost_timeout
                ):
                    old_priority = current_priority_map.get(
                        self._latest_ball_target.class_name, 999
                    )
                    new_priority = best_priority_in_frame

                    if new_priority == old_priority:
                        best_in_frame = min(
                            top_priority_balls,
                            key=lambda candidate: (
                                (candidate.x - self._latest_ball_target.x) ** 2
                                + (candidate.y - self._latest_ball_target.y) ** 2
                            ),
                        )
                    elif new_priority < old_priority:
                        best_in_frame = self._select_leftmost_large_ball(
                            top_priority_balls
                        )
                else:
                    best_in_frame = self._select_leftmost_large_ball(
                        top_priority_balls
                    )

                if best_in_frame is not None:
                    self._latest_ball_target = best_in_frame
                    self._last_ball_seen_time = now
                    self._last_search_direction = (
                        1.0 if best_in_frame.x <= self.target_center_x else -1.0
                    )

        if self._state in (self.SEARCH_SAFE, self.GO_SAFE, self.VERIFY_SAFE):
            if safe_candidates:
                if self._state == self.VERIFY_SAFE and self._safe_verify_anchor is not None:
                    matched_safe = self._match_safe_candidate(
                        safe_candidates, self._safe_verify_anchor
                    )
                    if matched_safe is not None:
                        self._latest_safe_target = matched_safe
                        self._last_safe_seen_time = now
                        self._last_search_direction = (
                            1.0 if matched_safe.x <= self.target_center_x else -1.0
                        )
                        self._safe_verify_votes[matched_safe.class_name] = (
                            self._safe_verify_votes.get(matched_safe.class_name, 0) + 1
                        )
                        self._safe_verify_samples += 1
                else:
                    selected_safe = self._select_safe_candidate(safe_candidates, now)
                    if selected_safe is not None:
                        self._latest_safe_target = selected_safe
                        self._last_safe_seen_time = now
                        self._last_search_direction = (
                            1.0 if selected_safe.x <= self.target_center_x else -1.0
                        )

    def _control_timer_callback(self) -> None:
        now = time.monotonic()
        cmd = Twist()

        if self._state == self.STARTUP_FORWARD:
            elapsed = now - self._state_enter_time
            if not self._gripper_closed:
                self._publish_gripper_close()
            if elapsed < self.startup_forward_seconds:
                cmd.linear.x = self.startup_forward_speed
                cmd.angular.z = self.startup_forward_angular_z
            else:
                self._publish_gripper_open()
                self._set_state(self.SEARCH_BALL)
                return

        elif self._state == self.SEARCH_BALL:
            if self._target_visible(self._last_ball_seen_time, now):
                self._set_state(self.APPROACH_BALL)
                return
            cmd.angular.z = self.ball_search_angular_speed * self._last_search_direction

        elif self._state == self.APPROACH_BALL:
            target = self._latest_ball_target
            if target is None or not self._target_visible(self._last_ball_seen_time, now):
                self._grab_ready_cycles = 0
                self._set_state(self.SEARCH_BALL)
                return

            if self._ready_to_grab(target):
                self._grab_ready_cycles += 1
                cmd = Twist()
                if self._grab_ready_cycles >= self.grab_stable_cycles:
                    self._set_state(self.GRAB_BALL)
            else:
                self._grab_ready_cycles = 0
                approach_heading_limit_px = (
                    self.approach_alignment_px
                    if self.approach_align_before_forward
                    else self.approach_forward_heading_limit_px
                )
                current_max_linear_speed = self.approach_max_linear_speed
                if target.y >= self.near_grab_y:
                    current_max_linear_speed = min(
                        current_max_linear_speed, self.near_grab_max_linear_speed
                    )
                cmd = self._build_follow_cmd(
                    target=target,
                    target_y=self.approach_target_y,
                    y_deadband=self.approach_y_deadband,
                    max_linear_speed=current_max_linear_speed,
                    max_angular_speed=self.approach_max_angular_speed,
                    forward_heading_limit_px=approach_heading_limit_px,
                )

        elif self._state == self.GRAB_BALL:
            cmd = Twist()
            elapsed = now - self._state_enter_time

            if elapsed < self.blind_forward_seconds:
                cmd.linear.x = self.blind_forward_speed
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if not self._gripper_closed:
                    self._publish_gripper_close()

                if elapsed >= self.blind_forward_seconds + self.grip_hold_seconds:
                    self._set_state(self.SEARCH_SAFE)
                    return

        elif self._state == self.SEARCH_SAFE:
            if self._target_visible(self._last_safe_seen_time, now):
                self._set_state(self.GO_SAFE)
                return
            cmd.angular.z = self.safe_search_angular_speed * self._last_search_direction

        elif self._state == self.GO_SAFE:
            target = self._latest_safe_target
            if target is None or not self._target_visible(self._last_safe_seen_time, now):
                self._set_state(self.SEARCH_SAFE)
                return

            if not self._safe_target_verified and target.y >= self.safe_verify_y:
                self._begin_safe_verification(target)
                self._set_state(self.VERIFY_SAFE)
                return

            if self._safe_arrived(target):
                self._set_state(self.PLACE_SAFE)
                return
            else:
                cmd = self._build_follow_cmd(
                    target=target,
                    target_y=self.safe_target_y,
                    y_deadband=self.safe_y_deadband,
                    max_linear_speed=self.safe_max_linear_speed,
                    max_angular_speed=self.safe_max_angular_speed,
                    forward_heading_limit_px=self.safe_forward_heading_limit_px,
                )

        elif self._state == self.VERIFY_SAFE:
            cmd = Twist()
            elapsed = now - self._state_enter_time

            if self._safe_verify_samples >= self.safe_verify_frames:
                own_safe_votes = self._safe_verify_votes.get(self.safe_class, 0)
                if own_safe_votes >= self.safe_verify_pass_votes:
                    self._safe_target_verified = True
                    self._finish_safe_verification()
                    self._set_state(self.GO_SAFE)
                else:
                    self._reject_current_safe_target(now)
                    self._safe_target_verified = False
                    self._latest_safe_target = None
                    self._last_safe_seen_time = 0.0
                    self._set_state(self.SEARCH_SAFE)
                return

            if elapsed >= self.safe_verify_timeout_seconds:
                self._reject_current_safe_target(now)
                self._safe_target_verified = False
                self._latest_safe_target = None
                self._last_safe_seen_time = 0.0
                self._set_state(self.SEARCH_SAFE)
                return

        elif self._state == self.PLACE_SAFE:
            cmd = Twist()
            elapsed = now - self._state_enter_time

            if elapsed < self.safe_open_before_place_seconds:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if self.release_in_safe_zone and not self._safe_release_sent:
                    self._publish_gripper_open()
                    self._safe_release_sent = True
            elif elapsed < self.safe_open_before_place_seconds + self.safe_place_seconds:
                cmd.linear.x = self.safe_place_speed
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self._set_state(self.COMPLETE)
                return

        elif self._state == self.COMPLETE:
            cmd = Twist()
            elapsed = now - self._state_enter_time

            if elapsed < self.post_release_reverse_seconds:
                cmd.linear.x = self.post_release_reverse_speed
                cmd.angular.z = 0.0
            elif elapsed < self.post_release_reverse_seconds + self.post_release_turn_seconds:
                cmd.linear.x = 0.0
                cmd.angular.z = self.post_release_turn_angular_speed
            else:
                self._safe_release_sent = False
                self._latest_ball_target = None
                self._latest_safe_target = None
                self._last_ball_seen_time = 0.0
                self._last_safe_seen_time = 0.0
                self._safe_target_verified = False
                self._finish_safe_verification()
                self._completed_grab_cycles += 1
                self._set_state(self.SEARCH_BALL)
                return

        self.cmd_pub.publish(cmd)

    def _build_follow_cmd(
        self,
        target: VisionCandidate,
        target_y: float,
        y_deadband: float,
        max_linear_speed: float,
        max_angular_speed: float,
        forward_heading_limit_px: float,
    ) -> Twist:
        cmd = Twist()

        heading_error = self.target_center_x - target.x
        if abs(heading_error) <= self.heading_deadband_px:
            angular_z = 0.0
        else:
            angular_z = self.angular_kp * heading_error

        angular_z = self._clamp(angular_z, -max_angular_speed, max_angular_speed)

        if abs(heading_error) > forward_heading_limit_px:
            linear_x = 0.0
        else:
            y_error = target_y - target.y
            if y_error <= y_deadband:
                linear_x = 0.0
            else:
                linear_x = y_error / max(1.0, target_y) * max_linear_speed

        cmd.linear.x = self._clamp(linear_x, -max_linear_speed, max_linear_speed)
        cmd.angular.z = angular_z
        return cmd

    def _ready_to_grab(self, target: VisionCandidate) -> bool:
        return (
            abs(self.target_center_x - target.x) <= self.grab_heading_deadband_px
            and target.y >= self.grab_trigger_y
        )

    def _safe_arrived(self, target: VisionCandidate) -> bool:
        return (
            abs(self.target_center_x - target.x) <= self.safe_heading_deadband_px
            and target.y >= self.safe_target_y - self.safe_y_deadband
        )

    def _target_visible(self, last_seen_time: float, now: float) -> bool:
        return (now - last_seen_time) <= self.lost_timeout

    def _begin_safe_verification(self, target: VisionCandidate) -> None:
        self._safe_verify_anchor = VisionCandidate(
            class_name=target.class_name,
            x=target.x,
            y=target.y,
            size_metric=target.size_metric,
        )
        self._safe_verify_votes = {'redsafe': 0, 'bluesafe': 0}
        self._safe_verify_samples = 0

    def _finish_safe_verification(self) -> None:
        self._safe_verify_anchor = None
        self._safe_verify_votes = {'redsafe': 0, 'bluesafe': 0}
        self._safe_verify_samples = 0

    def _reject_current_safe_target(self, now: float) -> None:
        if self._safe_verify_anchor is not None:
            self._rejected_safe_anchor = VisionCandidate(
                class_name=self._safe_verify_anchor.class_name,
                x=self._safe_verify_anchor.x,
                y=self._safe_verify_anchor.y,
                size_metric=self._safe_verify_anchor.size_metric,
            )
            self._rejected_safe_until = now + self.safe_reject_cooldown_seconds
        self._finish_safe_verification()

    def _is_rejected_safe_candidate(
        self, candidate: VisionCandidate, now: float
    ) -> bool:
        if self._rejected_safe_anchor is None or now >= self._rejected_safe_until:
            return False
        return (
            abs(candidate.x - self._rejected_safe_anchor.x)
            <= self.safe_verify_match_x_px
            and abs(candidate.y - self._rejected_safe_anchor.y)
            <= self.safe_verify_match_y_px
        )

    def _select_safe_candidate(
        self, safe_candidates: List[VisionCandidate], now: float
    ) -> Optional[VisionCandidate]:
        if not safe_candidates:
            return None

        if (
            self._latest_safe_target is not None
            and self._target_visible(self._last_safe_seen_time, now)
        ):
            return min(
                safe_candidates,
                key=lambda candidate: (
                    (candidate.x - self._latest_safe_target.x) ** 2
                    + (candidate.y - self._latest_safe_target.y) ** 2
                ),
            )

        return sorted(safe_candidates, key=lambda item: -item.size_metric)[0]

    def _match_safe_candidate(
        self,
        safe_candidates: List[VisionCandidate],
        anchor: VisionCandidate,
    ) -> Optional[VisionCandidate]:
        matched_candidates = [
            candidate
            for candidate in safe_candidates
            if abs(candidate.x - anchor.x) <= self.safe_verify_match_x_px
            and abs(candidate.y - anchor.y) <= self.safe_verify_match_y_px
        ]
        if not matched_candidates:
            return None
        return min(
            matched_candidates,
            key=lambda candidate: (
                (candidate.x - anchor.x) ** 2 + (candidate.y - anchor.y) ** 2
            ),
        )

    def _filter_balls_inside_safe_zone(
        self,
        ball_candidates: List[VisionCandidate],
        safe_candidates: List[VisionCandidate],
    ) -> List[VisionCandidate]:
        filtered_balls = []
        for ball in ball_candidates:
            is_inside_safe = any(
                abs(ball.x - safe.x) < self.ignore_ball_in_safe_x_px
                and abs(ball.y - safe.y) < self.ignore_ball_in_safe_y_px
                for safe in safe_candidates
            )
            if not is_inside_safe:
                filtered_balls.append(ball)
        return filtered_balls

    def _select_leftmost_large_ball(
        self, candidates: List[VisionCandidate]
    ) -> VisionCandidate:
        max_size = max(candidate.size_metric for candidate in candidates)
        same_size_candidates = [
            candidate
            for candidate in candidates
            if (max_size - candidate.size_metric) <= self.same_size_bin_px
        ]
        return min(same_size_candidates, key=lambda candidate: candidate.x)

    def _get_current_priority_map(self) -> Dict[str, int]:
        if self._completed_grab_cycles < 2:
            return {self.team_color: 0}

        filtered_order: List[str] = []
        for class_name in self.ball_priority_order:
            if class_name == self.opponent_color or class_name == self.safe_class:
                continue
            if class_name not in ('red', 'blue', 'yellow', 'black'):
                continue
            if class_name not in filtered_order:
                filtered_order.append(class_name)

        if self.team_color not in filtered_order:
            filtered_order.append(self.team_color)

        return {
            class_name: index for index, class_name in enumerate(filtered_order)
        }

    def _publish_gripper_close(self) -> None:
        self._publish_servo_angle(self.servo1_pub, self.servo1_close_angle)
        self._publish_servo_angle(self.servo2_pub, self.servo2_close_angle)
        self._gripper_closed = True
        self.get_logger().info(
            f'Close gripper: servo1={self.servo1_close_angle}, servo2={self.servo2_close_angle}'
        )

    def _publish_gripper_open(self) -> None:
        self._publish_servo_angle(self.servo1_pub, self.servo1_open_angle)
        self._publish_servo_angle(self.servo2_pub, self.servo2_open_angle)
        self._gripper_closed = False
        self.get_logger().info(
            f'Open gripper: servo1={self.servo1_open_angle}, servo2={self.servo2_open_angle}'
        )

    def _startup_open_callback(self) -> None:
        self._publish_gripper_open()
        if self._startup_open_timer is not None:
            self._startup_open_timer.cancel()
            self.destroy_timer(self._startup_open_timer)
            self._startup_open_timer = None

    @staticmethod
    def _publish_servo_angle(publisher, angle: int) -> None:
        msg = Int32()
        msg.data = int(angle)
        publisher.publish(msg)

    def _set_state(self, new_state: str) -> None:
        if new_state == self._state:
            return
        old_state = self._state
        self._state = new_state
        self._state_enter_time = time.monotonic()
        self._grab_ready_cycles = 0
        if new_state == self.SEARCH_SAFE:
            self._safe_target_verified = False
        self.get_logger().info(f'State change: {old_state} -> {new_state}')

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def destroy_node(self) -> bool:
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallMissionLogic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ball_mission_logic stopped by keyboard')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
