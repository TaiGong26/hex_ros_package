#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2025-07-08
################################################################
# Modified: cmd_vel key control for hex_device_ros_wrapper chassis
# Topic: /cmd_vel (geometry_msgs/msg/Twist)
################################################################

from __future__ import print_function

import threading
import sys
from select import select
from typing import Dict, Tuple

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


class CmdVelTeleopKeyboard:
    """ROS2 Teleop keyboard control interface for cmd_vel (Twist)"""

    HELP_MSG = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing):
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

Anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

    # 键盘 -> (linear_inc, angular_inc, linear_y_inc)
    # i: 前, ,: 后, j: 左转, l: 右转, k: 停止
    # u/o/m/.: 前左/前右/后左/后右
    # U/I...: 全向移动模式 (如果有需要，这里主要实现差速逻辑，全向可作为扩展)
    MOVE_BINDINGS: Dict[str, Tuple[float, float, float]] = {
        # Differential Drive / Car-like
        'i': (1.0, 0.0, 0.0),
        ',': (-1.0, 0.0, 0.0),
        'j': (0.0, 1.0, 0.0),
        'l': (0.0, -1.0, 0.0),
        'u': (1.0, 1.0, 0.0),
        'o': (1.0, -1.0, 0.0),
        'm': (-1.0, 1.0, 0.0),
        '.': (-1.0, -1.0, 0.0),
        
        # Holonomic (Linear Y) - Shifted keys (Uppercase)
        'I': (1.0, 0.0, 0.0),
        '<': (-1.0, 0.0, 0.0),  # Shift+','
        'J': (0.0, 0.0, 1.0),   # Left Strafe
        'L': (0.0, 0.0, -1.0),  # Right Strafe
        # 可以根据需要补充斜向全向移动
    }

    # 速度缩放因子绑定
    # (linear_scale_factor, angular_scale_factor)
    SPEED_BINDINGS: Dict[str, Tuple[float, float]] = {
        'q': (1.1, 1.1),
        'z': (0.9, 0.9),
        'w': (1.1, 1.0),
        'x': (0.9, 1.0),
        'e': (1.0, 1.1),
        'c': (1.0, 0.9),
    }

    def __init__(self, name: str = "cmd_vel_key_control"):
        # name
        self._name = name

        # ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()

        # parameters
        # 这里的 speed 是指“每次按键增加的速度量”，如果需要持续按住加速，逻辑需改为累加
        # 这里参照原代码逻辑：按键映射增量
        self.__node.declare_parameter('linear_step', 0.5)    # m/s per key press (or increment)
        self.__node.declare_parameter('angular_step', 0.5)   # rad/s per key press
        self.__node.declare_parameter('linear_y_step', 0.5)  # m/s for holonomic
        self.__node.declare_parameter('repeat_rate', 10.0)   # Hz (publish rate)
        self.__node.declare_parameter('key_timeout', 0.5)    # s

        # Get parameters
        self.__linear_step = \
            self.__node.get_parameter('linear_step').get_parameter_value().double_value
        self.__angular_step = \
            self.__node.get_parameter('angular_step').get_parameter_value().double_value
        self.__linear_y_step = \
            self.__node.get_parameter('linear_y_step').get_parameter_value().double_value
        self.__repeat_rate = \
            self.__node.get_parameter('repeat_rate').get_parameter_value().double_value
        self.__key_timeout = \
            self.__node.get_parameter('key_timeout').get_parameter_value().double_value

        # state
        # 保存当前速度状态
        self.__linear_x = 0.0
        self.__linear_y = 0.0
        self.__angular_z = 0.0
        
        # 速度限制 (可选，防止速度过大)
        self.__max_linear = 1.5
        self.__max_angular = 3.0

        self.__status = 0

        # publisher
        qos = QoSProfile(depth=1)
        self.__cmd_vel_pub = self.__node.create_publisher(
            Twist,
            'cmd_vel',  # 标准 cmd_vel 话题
            qos
        )

        # publish thread
        self.__pub_thread = CmdVelPublishThread(
            self.__repeat_rate,
            self.__node
        )

        # spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        # terminal settings
        self.__settings = self.__save_terminal_settings()

        # finish log
        print(f"#### CmdVelTeleopKeyboard init: {self._name} ####")

    def __spin(self):
        rclpy.spin(self.__node)

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        self.__pub_thread.stop()
        self.__restore_terminal_settings(self.__settings)
        self.__node.destroy_node()
        rclpy.shutdown()
        self.__spin_thread.join()

    def __save_terminal_settings(self):
        """Save terminal settings"""
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

    def __restore_terminal_settings(self, old_settings):
        """Restore terminal settings"""
        if sys.platform == 'win32':
            return
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def __get_key(self, timeout: float) -> str:
        """Get key input with timeout"""
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            rlist, _, _ = select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            if self.__settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__settings)
        return key

    def __status_string(self) -> str:
        """Generate status string for velocities."""
        return (
            f"currently:\t"
            f"linear_x={self.__linear_x:.2f}\t"
            f"linear_y={self.__linear_y:.2f}\t"
            f"angular_z={self.__angular_z:.2f}\t"
            f"step: linear={self.__linear_step:.2f} angular={self.__angular_step:.2f}"
        )

    def run(self):
        """Main teleop control loop"""
        try:
            self.__pub_thread.wait_for_subscribers()
            # 初始化发布
            self.__pub_thread.update(self.__linear_x, self.__linear_y, self.__angular_z)

            print(self.HELP_MSG)
            print(self.__status_string())

            while self.ok():
                key = self.__get_key(self.__key_timeout)

                # 移动按键：累加/累减速度
                if key in self.MOVE_BINDINGS.keys():
                    linear_inc, angular_inc, linear_y_inc = self.MOVE_BINDINGS[key]
                    
                    # 累加模式：按一次增加一步，松开不回零（与典型的teleop_twist_keyboard不同，参照了你提供的累加逻辑）
                    self.__linear_x += linear_inc * self.__linear_step
                    self.__angular_z += angular_inc * self.__angular_step
                    self.__linear_y += linear_y_inc * self.__linear_y_step

                    # 限制最大速度
                    self.__linear_x = max(min(self.__linear_x, self.__max_linear), -self.__max_linear)
                    self.__angular_z = max(min(self.__angular_z, self.__max_angular), -self.__max_angular)
                    self.__linear_y = max(min(self.__linear_y, self.__max_linear), -self.__max_linear)

                # 速度步长缩放按键
                elif key in self.SPEED_BINDINGS.keys():
                    linear_factor, angular_factor = self.SPEED_BINDINGS[key]
                    self.__linear_step = min(10.0, self.__linear_step * linear_factor)
                    self.__angular_step = min(10.0, self.__angular_step * angular_factor)
                    
                    print(self.__status_string())
                    if self.__status == 14:
                        print(self.HELP_MSG)
                    self.__status = (self.__status + 1) % 15

                # 停止按键 (参照原代码 'k' 为中心，或者按其他键停止)
                # 这里显式处理 'k' 为归零
                elif key == 'k':
                    self.__linear_x = 0.0
                    self.__linear_y = 0.0
                    self.__angular_z = 0.0
                    print("Stop! Reset velocities to zero.")

                # Z轴控制 (如果是无人机或全向底盘)
                elif key == 't':
                    self.__linear_z = 1.0 # 需要在 update 中添加 linear_z 支持
                elif key == 'b':
                    self.__linear_z = -1.0
                
                # 其他按键：停止或保持
                else:
                    # 超时处理
                    if key == '':
                        # 如果希望松开按键后自动停止，取消下面注释：
                        # self.__linear_x = 0.0
                        # self.__angular_z = 0.0
                        pass
                    
                    if key == '\x03':  # Ctrl-C
                        break

                # 更新发布线程
                self.__pub_thread.update(self.__linear_x, self.__linear_y, self.__angular_z)

        except Exception as e:
            print(f"Error in cmd_vel teleop loop: {e}")
        finally:
            self.shutdown()


class CmdVelPublishThread(threading.Thread):
    """Thread that periodically publishes Twist to /cmd_vel."""

    def __init__(
        self,
        rate: float,
        node: rclpy.node.Node
    ):
        super(CmdVelPublishThread, self).__init__()
        self.node = node

        qos = QoSProfile(depth=1)
        self.publisher = self.node.create_publisher(
            Twist,
            'cmd_vel',
            qos
        )

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        self.condition = threading.Condition()
        self.done = False

        # rate
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if i == 4:
                print(
                    "Waiting for subscriber to connect to {}".format(
                        self.publisher.topic_name
                    )
                )
            rclpy.spin_once(self.node, timeout_sec=0.5)
            i += 1
            i = i % 5
        if not rclpy.ok():
            raise Exception("Got shutdown request before subscribers connected")

    def update(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float
    ):
        self.condition.acquire()
        self.linear_x = float(linear_x)
        self.linear_y = float(linear_y)
        self.angular_z = float(angular_z)
        # 通知发布线程有新数据
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0.0, 0.0, 0.0)
        self.join(timeout=1.0)

    def run(self):
        msg = Twist()

        while not self.done:
            self.condition.acquire()
            # 等待新数据或超时（周期性发布）
            self.condition.wait(self.timeout)

            # 填充 Twist 消息
            msg.linear.x = self.linear_x
            msg.linear.y = self.linear_y
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = self.angular_z

            self.condition.release()

            # 发布
            self.publisher.publish(msg)

        # 退出时发布一次零速度，确保机器人停止
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)


def main(args=None):
    """Main function to start cmd_vel teleop keyboard control"""
    try:
        teleop = CmdVelTeleopKeyboard()
        teleop.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'teleop' in locals():
            teleop.shutdown()


if __name__ == "__main__":
    main()
