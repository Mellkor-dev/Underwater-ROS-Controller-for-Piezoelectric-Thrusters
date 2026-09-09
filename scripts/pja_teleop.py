#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

# Keybindings
msg = """
=================================================
CSUR Micro-AUV Custom Teleop Node
=================================================
Moving around:
   w : Forward (Surge +X)
   s : Backward (Surge -X)
   a : Yaw Left (Turn +Yaw)
   d : Yaw Right (Turn -Yaw)

Vertical (Depth Control):
   t : Ascend / Up (+Heave Z)
   b : Descend / Down (-Heave Z)

Speed Adjustments:
   q : Increase Linear Speed by 20%
   z : Decrease Linear Speed by 20%
   e : Increase Angular Speed by 20%
   c : Decrease Angular Speed by 20%

SPACEBAR or 'k' : Emergency Stop (Zero All Speeds)
CTRL-C : Quit
=================================================
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('pja_teleop')
    pub = node.create_publisher(Twist, '/teleop/cmd_vel', 10)

    # Scaled defaults for 85g mass & 50mN PJA thrusters
    speed = 0.1     # Linear scaling (0.1 max effort)
    turn = 0.15     # Angular scaling (0.15 rad/s)
    
    x = 0.0
    z = 0.0
    th = 0.0

    try:
        print(msg)
        print(f"Currently:\tLinear Speed = {speed:.2f}\tAngular Speed = {turn:.2f}")
        while rclpy.ok():
            key = get_key(settings)
            
            if key == 'w':
                x = speed
            elif key == 's':
                x = -speed
            elif key == 'a':
                th = turn
            elif key == 'd':
                th = -turn
            elif key == 't':
                z = speed
            elif key == 'b':
                z = -speed
            elif key == 'q':
                speed = min(1.0, speed * 1.2)
                print(f"Linear Speed increased: {speed:.2f}")
            elif key == 'z':
                speed = max(0.01, speed * 0.8)
                print(f"Linear Speed decreased: {speed:.2f}")
            elif key == 'e':
                turn = min(1.0, turn * 1.2)
                print(f"Angular Speed increased: {turn:.2f}")
            elif key == 'c':
                turn = max(0.01, turn * 0.8)
                print(f"Angular Speed decreased: {turn:.2f}")
            elif key == ' ' or key == 'k':
                x = 0.0
                z = 0.0
                th = 0.0
            else:
                x = 0.0
                z = 0.0
                th = 0.0
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = float(x)
            twist.linear.y = 0.0
            twist.linear.z = float(z)
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(th)

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()