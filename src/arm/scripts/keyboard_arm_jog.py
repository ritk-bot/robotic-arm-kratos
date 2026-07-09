#!/usr/bin/env python3

import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class KeyboardArmJog(Node):

    def __init__(self):
        super().__init__("keyboard_arm_jog")

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/arm_cmd_vel",
            10
        )

        self.step = 0.5        # rad/s

        self.msg = Float64MultiArray()
        self.msg.data = [0.0] * 6

        # Publish continuously at 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.publish_callback
        )

        self.running = True

        self.print_help()

    def print_help(self):

        print("""
================ ARM JOG =================

Q / A : Base

W / S : Shoulder

E / D : Elbow

R / F : Wrist Pitch

T / G : Wrist Roll

Y / H : Gripper

SPACE : Stop

X : Exit

==========================================
""")

    def publish_callback(self):

        self.pub.publish(self.msg)

    def stop_all(self):

        self.msg.data = [0.0] * 6

    def handle_key(self, key):

        if key == 'q':
            self.stop_all()
            self.msg.data[0] = self.step

        elif key == 'a':
            self.stop_all()
            self.msg.data[0] = -self.step

        elif key == 'w':
            self.stop_all()
            self.msg.data[1] = self.step

        elif key == 's':
            self.stop_all()
            self.msg.data[1] = -self.step

        elif key == 'e':
            self.stop_all()
            self.msg.data[2] = self.step

        elif key == 'd':
            self.stop_all()
            self.msg.data[2] = -self.step

        elif key == 'r':
            self.stop_all()
            self.msg.data[3] = self.step

        elif key == 'f':
            self.stop_all()
            self.msg.data[3] = -self.step

        elif key == 't':
            self.stop_all()
            self.msg.data[4] = self.step

        elif key == 'g':
            self.stop_all()
            self.msg.data[4] = -self.step

        elif key == 'y':
            self.stop_all()
            self.msg.data[5] = self.step

        elif key == 'h':
            self.stop_all()
            self.msg.data[5] = -self.step

        elif key == ' ':
            self.stop_all()

        elif key == 'x':
            self.running = False
            return

        print(
            "\rVelocity Command : "
            + str([round(x, 2) for x in self.msg.data]),
            end="",
            flush=True
        )


def get_key():

    fd = sys.stdin.fileno()

    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        return sys.stdin.read(1)

    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )


def keyboard_loop(node):

    while node.running:

        key = get_key()

        node.handle_key(key)

    rclpy.shutdown()


def main():

    rclpy.init()

    node = KeyboardArmJog()

    keyboard_thread = threading.Thread(
        target=keyboard_loop,
        args=(node,),
        daemon=True
    )

    keyboard_thread.start()

    rclpy.spin(node)

    node.destroy_node()


if __name__ == "__main__":
    main()