#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener
import sys
import signal

# 初始化ROS节点
rospy.init_node('keyboard_driver', anonymous=True)

# 创建速度和布尔值的发布者
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
keyboard_pub = rospy.Publisher('/keyboard', Bool, queue_size=1)

# 用于跟踪当前按下的键
pressed_keys = set()

def update_move():
    move = Twist()

    if Key.up in pressed_keys:
        move.linear.x = 0.2
    if Key.down in pressed_keys:
        move.linear.x = -0.2
    if Key.left in pressed_keys:
        move.linear.y = 0.2
    if Key.right in pressed_keys:
        move.linear.y = -0.2

    if 'q' in pressed_keys:
        move.angular.z = 1
    if 'e' in pressed_keys:
        move.angular.z = -1
    if 'w' in pressed_keys:
        move.linear.x = 0.3
    if 'a' in pressed_keys:
        move.angular.x = 5.5

    print(move)
    cmd_vel.publish(move)

def on_press(key):
    key_pressed = Bool()

    try:
        if key.char == 'i':
            key_pressed.data = True
            keyboard_pub.publish(key_pressed)
        elif key.char == 'o':
            key_pressed.data = False
            keyboard_pub.publish(key_pressed)
        elif key.char in ['q', 'e', 'w', 'a']:
            pressed_keys.add(key.char)
    except AttributeError:
        if key in [Key.up, Key.down, Key.left, Key.right]:
            pressed_keys.add(key)

    update_move()

def on_release(key):
    try:
        if key.char in ['q', 'e', 'w',  'a']:
            pressed_keys.discard(key.char)
    except AttributeError:
        if key in [Key.up, Key.down, Key.left, Key.right]:
            pressed_keys.discard(key)

    update_move()

    if key == Key.esc:
        return False

def exit_gracefully(signum, frame):
    # Handle any cleanup here
    print("Program exited gracefully")
    sys.exit(0)

def main():
    # Set the signal handler
    signal.signal(signal.SIGINT, exit_gracefully)
    
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

if __name__ == "__main__":
    main()