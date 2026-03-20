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

from sensor_msgs.msg import JointState

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


class JointCmdTeleopKeyboard:
    """ROS2 Teleop keyboard control interface for joint_cmd (JointState)"""

    HELP_MSG = """
Reading from the keyboard and Publishing to JointState!
---------------------------
Joint control (increment position):
   u    i    o
   j    k    l
   m    ,    .

Joint index mapping:
   0: joint0
   1: joint1
   2: joint2
   3: joint3
   4: joint4
   5: joint5

Anything else : stop

q/z : increase/decrease position step by 10%
w/x : increase/decrease velocity limit by 10%
e/c : increase/decrease effort limit by 10%

r : reset all joint positions to zero
s : save current positions as "home"
h : move to "home" positions

CTRL-C to quit
"""

    # 键盘 -> (joint_index, position_inc)
    # 这里用简单的 6 关节示例，你可以按实际关节数修改
    MOVE_BINDINGS: Dict[str, Tuple[int, float]] = {
        # joint 0
        'i': (0, 1.0),
        ',': (0, -1.0),
        # joint 1
        'u': (1, 1.0),
        'm': (1, -1.0),
        # joint 2
        'o': (2, 1.0),
        '.': (2, -1.0),
        # joint 3
        'j': (3, 1.0),
        'l': (3, -1.0),
        # joint 4
        'k': (4, 1.0),
        # joint 5
        # 自行扩展，比如 'n' / 'b' 等
    }

    # 速度/力矩缩放因子绑定
    SPEED_BINDINGS: Dict[str, Tuple[float, float]] = {
        'q': (1.1, 1.1),
        'z': (0.9, 0.9),
        'w': (1.1, 1.0),
        'x': (0.9, 1.0),
        'e': (1.0, 1.1),
        'c': (1.0, 0.9),
    }

    def __init__(self, name: str = "joint_cmd_key_control"):
        # name
        self._name = name

        # ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()

        # parameters
        self.__node.declare_parameter('position_step', 0.0)     # rad per key press
        self.__node.declare_parameter('velocity_limit', 0.5)    # rad/s
        self.__node.declare_parameter('effort_limit', 0.0)     # Nm or N
        self.__node.declare_parameter('repeat_rate', 100.0)     # Hz
        self.__node.declare_parameter('key_timeout', 0.5)       # s

        self.__node.declare_parameter('joint_names', [f'joint{i}' for i in range(4)])  # 与你的 URDF / 配置一致
        # self.__node.declare_parameter('joint_names', [f'joint{i}' for i in range(8)])  # 与你的 URDF / 配置一致

        # Get parameters
        self.__position_step = \
            self.__node.get_parameter('position_step').get_parameter_value().double_value
        self.__velocity_limit = \
            self.__node.get_parameter('velocity_limit').get_parameter_value().double_value
        self.__effort_limit = \
            self.__node.get_parameter('effort_limit').get_parameter_value().double_value
        self.__repeat_rate = \
            self.__node.get_parameter('repeat_rate').get_parameter_value().double_value
        self.__key_timeout = \
            self.__node.get_parameter('key_timeout').get_parameter_value().double_value

        self.__joint_names = \
            self.__node.get_parameter('joint_names').get_parameter_value().string_array_value

        # state
        self.__positions = [0.0] * len(self.__joint_names)  # 目标位置
        self.__home_positions = [0.0] * len(self.__joint_names)  # 保存的“home”位置
        self.__status = 0

        # publisher
        qos = QoSProfile(depth=1)
        self.__joint_cmd_pub = self.__node.create_publisher(
            JointState,
            'joint_cmd',  # 对应 hex_device_ros_wrapper 的 /joint_cmd 话题
            qos
        )

        # publish thread
        self.__pub_thread = JointCmdPublishThread(
            self.__repeat_rate,
            self.__node,
            self.__joint_names
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
        """Generate status string for positions / velocities."""
        lines = ["currently:"]
        for i, name in enumerate(self.__joint_names):
            lines.append(
                f"  {name}: pos={self.__positions[i]:.3f} "
                f"vel_limit={self.__velocity_limit:.3f} "
                f"effort_limit={self.__effort_limit:.3f}"
            )
        return "\n".join(lines)

    def run(self):
        """Main teleop control loop"""
        try:
            self.__pub_thread.wait_for_subscribers()
            self.__pub_thread.update(
                self.__positions,
                self.__velocity_limit,
                self.__effort_limit
            )

            print(self.HELP_MSG)
            print(self.__status_string())

            while self.ok():
                key = self.__get_key(self.__key_timeout)

                # 移动按键：调整某个关节的目标位置
                if key in self.MOVE_BINDINGS.keys():
                    joint_index, inc_sign = self.MOVE_BINDINGS[key]
                    if joint_index >= len(self.__positions):
                        self.__logger.warn(
                            f"Joint index {joint_index} out of range "
                            f"[0, {len(self.__positions) - 1}]"
                        )
                        continue

                    # 根据 inc_sign 和 position_step 调整目标位置
                    self.__positions[joint_index] += inc_sign * self.__position_step

                # 速度/力矩缩放按键
                elif key in self.SPEED_BINDINGS.keys():
                    self.__velocity_limit = min(
                        10.0,
                        self.__velocity_limit * self.SPEED_BINDINGS[key][0]
                    )
                    # self.__effort_limit = min(
                    #     100.0,
                    #     self.__effort_limit * self.SPEED_BINDINGS[key][1]
                    # )
                    print(self.__status_string())
                    if self.__status == 14:
                        print(self.HELP_MSG)
                    self.__status = (self.__status + 1) % 15

                # 复位到零位
                elif key == 'r':
                    self.__positions = [0.0 for _ in self.__positions]
                    print("Reset all joint positions to zero.")

                # 保存当前位置为 home
                elif key == 's':
                    self.__home_positions = [pos for pos in self.__positions]
                    print("Saved current positions as home.")

                # 回 home 位
                elif key == 'h':
                    self.__positions = [pos for pos in self.__home_positions]
                    print("Move to home positions.")

                # 其他按键：停止（保持当前位置）
                else:
                    # 超时且已经停止则不重复更新
                    if key == '' and all(
                        abs(pos) < 1e-3 for pos in self.__positions
                    ):
                        continue
                    # 不清零位置，只是不继续增加；这里可以保持当前位置
                    # 若你希望按键释放后回零，可以 self.__positions = [0.0] * N
                    if key == '\x03':  # Ctrl-C
                        break

                # 更新发布线程
                self.__pub_thread.update(
                    self.__positions,
                    self.__velocity_limit,
                    self.__effort_limit
                )

        except Exception as e:
            print(f"Error in joint_cmd teleop loop: {e}")
        finally:
            self.shutdown()


class JointCmdPublishThread(threading.Thread):
    """Thread that periodically publishes JointState to /joint_cmd."""

    def __init__(
        self,
        rate: float,
        node: rclpy.node.Node,
        joint_names
    ):
        super(JointCmdPublishThread, self).__init__()
        self.node = node
        self.joint_names = joint_names

        qos = QoSProfile(depth=1)
        self.publisher = self.node.create_publisher(
            JointState,
            'joint_cmd',  # 与 hex_device_ros_wrapper 中的 /joint_cmd 一致
            qos
        )

        self.positions = [0.0] * len(joint_names)
        self.velocity_limit = 0.0
        self.effort_limit = 0.0
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
        positions,
        velocity_limit: float,
        effort_limit: float
    ):
        self.condition.acquire()
        self.positions = [float(p) for p in positions]
        self.velocity_limit = float(velocity_limit)
        self.effort_limit = float(effort_limit)
        # 通知发布线程有新数据
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update([0.0] * len(self.joint_names), 0.0, 0.0)
        self.join(timeout=1.0)

    def run(self):
        msg = JointState()

        while not self.done:
            self.condition.acquire()
            # 等待新数据或超时（周期性发布）
            self.condition.wait(self.timeout)

            # 填充 JointState 消息
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name = list(self.joint_names)  # 必须与关节顺序一致
            # msg.position = [float(p) for p in self.positions]
            # msg.velocity = [float(self.velocity_limit)] * len(self.joint_names)
            msg.velocity = [float(self.velocity_limit) if index % 2 == 0 else 0.0 
                            for index, name in enumerate(self.joint_names)]
            # msg.effort = [float(self.effort_limit)] * len(self.joint_names)

            self.condition.release()

            # 发布
            self.publisher.publish(msg)

        # 退出时发布一次零速度/力矩，确保机器人停止
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [0.0] * len(self.joint_names)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.publisher.publish(msg)


def main(args=None):
    """Main function to start joint_cmd teleop keyboard control"""
    try:
        teleop = JointCmdTeleopKeyboard()
        teleop.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        teleop.shutdown()


if __name__ == "__main__":
    main()
