#!/usr/bin/env python3
"""
TurtleBot3 teleop: hold-to-move with velocity ramps.

Hold a key to drive; releasing stops the robot. Combine keys for diagonal
motion (e.g. w+a = forward-left). Ramps on linear and angular velocity
reduce jerk for smoother SLAM data collection.
"""
import os
import select
import sys
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN_VEL = 0.22   # m/s
MAX_ANG_VEL = 0.8    # rad/s

MAX_LIN_ACC = 0.25    # m/s²  — time to full speed: ~0.44 s
MAX_ANG_ACC = 0.75    # rad/s² — time to full turn rate: ~0.53 s

LOOP_HZ = 50
DT = 1.0 / LOOP_HZ
KEY_INITIAL_TIMEOUT = 0.7   # s — covers OS key-repeat initial delay (~500 ms before repeating starts)
KEY_REPEAT_TIMEOUT  = 0.15  # s — covers gap between key-repeat events once repeating (~30 Hz)

msg = """
TurtleBot3 Teleop (hold-to-move)
----------------------------------
         w
    a    s    d
         x

Hold keys to drive — release to stop.
  w / x       : forward / backward
  a / d       : turn left / right
  w+a, w+d    : forward-left / forward-right
  space / s   : emergency stop
  CTRL-C      : quit
"""

KEY_COMMANDS = {
    'w': ( MAX_LIN_VEL,  0.0),
    'x': (-MAX_LIN_VEL,  0.0),
    'a': ( 0.0,           MAX_ANG_VEL),
    'd': ( 0.0,          -MAX_ANG_VEL),
}


def ramp(current, target, max_step):
    diff = target - current
    if abs(diff) <= max_step:
        return target
    return current + max_step * (1.0 if diff > 0 else -1.0)


def publish_twist(pub, ros_distro, lin, ang):
    if ros_distro == 'humble':
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        pub.publish(twist)
    else:
        ts = TwistStamped()
        ts.header.stamp = Clock().now().to_msg()
        ts.twist.linear.x = lin
        ts.twist.angular.z = ang
        pub.publish(ts)


def main():
    settings = None if os.name == 'nt' else termios.tcgetattr(sys.stdin)

    rclpy.init()
    ros_distro = os.environ.get('ROS_DISTRO', '')
    node = rclpy.create_node('teleop_keyboard')
    qos = QoSProfile(depth=10)

    if ros_distro == 'humble':
        pub = node.create_publisher(Twist, 'cmd_vel', qos)
    else:
        pub = node.create_publisher(TwistStamped, 'cmd_vel', qos)

    lin_vel = 0.0
    ang_vel = 0.0
    key_times = {}  # key -> (first_seen, last_seen)

    print(msg)

    try:
        if os.name != 'nt':
            tty.setraw(sys.stdin.fileno())

        while True:
            t_start = time.monotonic()

            # Non-blocking read — relies on OS key-repeat (~30 Hz) to sustain held keys
            if os.name == 'nt':
                key = msvcrt.getch().decode('utf-8', errors='ignore') if msvcrt.kbhit() else ''
            else:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
                key = sys.stdin.read(1) if rlist else ''

            if key == '\x03':
                break
            elif key in (' ', 's'):
                key_times.clear()
                lin_vel = ang_vel = 0.0
            elif key in KEY_COMMANDS:
                if key not in key_times:
                    key_times[key] = (t_start, t_start)
                else:
                    key_times[key] = (key_times[key][0], t_start)

            # Expire keys that have been released.
            # Use the long initial timeout until key-repeat starts, then the short repeat timeout.
            def is_held(entry):
                first, last = entry
                repeating = (last - first) > KEY_REPEAT_TIMEOUT
                timeout = KEY_REPEAT_TIMEOUT if repeating else KEY_INITIAL_TIMEOUT
                return (t_start - last) < timeout

            stale = [k for k, entry in key_times.items() if not is_held(entry)]
            for k in stale:
                del key_times[k]

            # Target velocity is the sum of all held-key contributions
            target_lin = sum((KEY_COMMANDS[k][0] for k in key_times), 0.0)
            target_ang = sum((KEY_COMMANDS[k][1] for k in key_times), 0.0)
            target_lin = max(-MAX_LIN_VEL, min(MAX_LIN_VEL, target_lin))
            target_ang = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, target_ang))

            lin_vel = ramp(lin_vel, target_lin, MAX_LIN_ACC * DT)
            ang_vel = ramp(ang_vel, target_ang, MAX_ANG_ACC * DT)

            publish_twist(pub, ros_distro, lin_vel, ang_vel)

            sleep_time = DT - (time.monotonic() - t_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as exc:
        print(exc)
    finally:
        publish_twist(pub, ros_distro, 0.0, 0.0)
        if os.name != 'nt' and settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
