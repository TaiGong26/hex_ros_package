#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2025-07-08
################################################################
# Modified: joint_cmd key control for hex_device_ros_wrapper chassis
# Topic: /joint_cmd (sensor_msgs/msg/JointState)
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
from geometry_msgs.msg import TwistStamped

import traceback

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


class CmdVelTeleopKeyboard:
    """ROS2 Teleop keyboard control interface for joint_cmd (JointState)"""

    HELP_MSG = """
Reading from the keyboard and Publishing to JointState!

"""

    # 键盘 -> (joint_index, position_inc)
    # 这里用简单的 6 关节示例，你可以按实际关节数修改
    MOVE_BINDINGS: Dict[str, Tuple[int, int]] = {
        'i':(1,0),
        'k':(-1,0),
        'j':(0,1),
        'l':(0,-1),
        ';':(0,0)
        
    }


    def __init__(self, name: str = "joint_cmd_key_control"):
        # name
        self._name = name

        # ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()

        # parameters
        self.__node.declare_parameter('speed_min_limit', 0)
        self.__node.declare_parameter('speed_max_limit', 4.0)
        self.__node.declare_parameter('repeat_rate', 100.0)     # Hz
        self.__node.declare_parameter('key_timeout', 0.5)       # s

        # Get parameters
        self.__speed_min_limit = \
            self.__node.get_parameter('speed_min_limit').get_parameter_value().double_value
        self.__speed_max_limit = \
            self.__node.get_parameter('speed_max_limit').get_parameter_value().double_value
        self.__repeat_rate = \
            self.__node.get_parameter('repeat_rate').get_parameter_value().double_value
        self.__key_timeout = \
            self.__node.get_parameter('key_timeout').get_parameter_value().double_value

        # state
        self.__speed = 0.5
        self.__vehicle_x = 0.0
        self.__vehicle_y = 0.0
        
        # publish thread
        self.__pub_thread = JointCmdPublishThread(
            self.__repeat_rate,
            self.__node
        )

        # spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        # terminal settings
        self.__settings = self.__save_terminal_settings()

        # finish log
        print(f"#### JointCmdTeleopKeyboard init: {self._name} ####")

    def __spin(self):
        rclpy.spin(self.__node)

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        self.__pub_thread.stop()
        self.__restore_terminal_settings(self.__settings)
        self.__node.destroy_node()
        rclpy.shutdown()
        self.__spin_thread.join(timeout=1.0)
    
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
        
        # 此处发现概率性捕捉不到keyboard，补充raise
        if key == '\x03':
            raise KeyboardInterrupt
    
        return key

    # def __status_string(self) -> str:
    #     """Generate status string for positions / velocities."""
    #     lines = ["currently:"]
    #     for i, name in enumerate(self.__joint_names):
    #         lines.append(
    #             f"  {name}: pos={self.__positions[i]:.3f} "
    #             f"vel_limit={self.__velocity_limit[i]:.3f} "
    #             f"effort_limit={self.__effort_limit[i]:.3f}"
    #         )
    #     return "\n".join(lines)

    def run(self):
        """Main teleop control loop"""
        try:
            self.__pub_thread.wait_for_subscribers()
            self.__pub_thread.update(
                self.__vehicle_x,
                self.__vehicle_y
            )

            print(self.HELP_MSG)

            while self.ok():
                # -------key--------
                key = self.__get_key(self.__key_timeout)

                if key == "u":
                    self.__speed = max(self.__speed_min_limit,self.__speed + 0.2)
                if key == "p":
                    self.__speed = max(self.__speed_min_limit,self.__speed - 0.2)
                
                if key in self.MOVE_BINDINGS:
                    self.__vehicle_x= self.MOVE_BINDINGS[key][0] * self.__speed
                    self.__vehicle_y= self.MOVE_BINDINGS[key][1] * self.__speed
                
                # 更新发布线程
                self.__pub_thread.update(
                    self.__vehicle_x,
                    self.__vehicle_y
                )

        except Exception as e:
            traceback.print_exc()
            print(f"Error in joint_cmd teleop loop: {e}")
        finally:
            self.shutdown()


class JointCmdPublishThread(threading.Thread):
    """Thread that periodically publishes JointState to /joint_cmd."""

    def __init__(
        self,
        rate: float,
        node: rclpy.node.Node
    ):
        super(JointCmdPublishThread, self).__init__()
        self.node = node
        
        qos = QoSProfile(depth=1)
        self.publisher = self.node.create_publisher(
            Twist,
            'cmd_vel',  
            qos
        )

        self.__x = None
        self.__y = None
        
        self.condition = threading.Condition()
        self.done = False

        # rate == 0 表示只在按键触发时更新
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
        x: float,
        y: float
    ):
        
        with self.condition:
            # 内容
            self.__x = x
            self.__y = y
            
            # 通知发布线程有新数据
            self.condition.notify()

    def stop(self):
        self.done = True
        self.update(0.0,0.0)
        self.join(timeout=1.0)

    def run(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        
        while not self.done:
            with self.condition:
                # 等待新数据或超时（周期性发布）
                self.condition.wait(self.timeout)
                msg.linear.x = 0.0 if (self.__x is None or self.__x == 0) else self.__x
                msg.linear.y = 0.0 if (self.__y is None or self.__y == 0) else self.__y

                
            # 发布
            self.publisher.publish(msg)

        # 退出时发布一次零速度/力矩，确保机器人停止
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.publisher.publish(msg)


def main(args=None):
    teleop = None
    try:
        teleop = CmdVelTeleopKeyboard()
        teleop.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if teleop is not None:
            teleop.shutdown()


if __name__ == "__main__":
    main()
