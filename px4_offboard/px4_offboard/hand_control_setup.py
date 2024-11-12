#!/usr/bin/env python3
import sys
import time

import geometry_msgs.msg
import rclpy
import std_msgs.msg
import leap
import termios
import tty

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

RAD_TO_DEG = 57.2958

msg = """
This node takes hand gestures from the Leap Motion Controller and publishes them as Twist messages. 
Using your right hand you have Mode 2 RC controls.
- Throttle Up
- Throttle Down
- Yaw Left
- Yaw Right
- Pitch Forward
- Pitch Backward
- Roll Left
- Roll Right

Press SPACE to arm/disarm the drone
"""


class MyListener(leap.Listener):
    def __init__(self, pub):
        super().__init__()
        self.pub = pub
        self.linear_threshold = 250.0
        self.angular_threshold = 10.0
        self.last_twist = geometry_msgs.msg.Twist()
        print("MyListener initialized.")

    def on_connection_event(self, event):
        print("Connected to Leap Motion Controller")

    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()
        print(f"Found device {info.serial}")
    
    def on_tracking_event(self, event):
        # print("Tracking event received")
        twist = geometry_msgs.msg.Twist()

        for hand in event.hands:
            if str(hand.type) == "HandType.Right":
                # Throttle Up/Down movements
                if hand.palm.position.y > self.linear_threshold + 50:  # Up
                    twist.linear.z = 1.0  # Up
                elif hand.palm.position.y < self.linear_threshold + 50 and hand.palm.position.y > self.linear_threshold - 50:  # Neutral
                    twist.linear.z = 0.0  # Neutral
                elif hand.palm.position.y < self.linear_threshold - 50:  # Down
                    twist.linear.z = -1.0  # Down

                # Pitch Forward/Backward movements
                if hand.palm.orientation.x * RAD_TO_DEG < -self.angular_threshold:  # Forward
                    twist.linear.y = 1.0  # Forward
                elif hand.palm.orientation.x * RAD_TO_DEG < self.angular_threshold and hand.palm.orientation.x * RAD_TO_DEG > -self.angular_threshold:  # Neutral
                    twist.linear.y = 0.0  # Neutral
                elif hand.palm.orientation.x * RAD_TO_DEG > self.angular_threshold:  # Backward
                    twist.linear.y = -1.0  # Backward
                    
                # Roll Left/Right movements
                if hand.palm.orientation.z * RAD_TO_DEG > self.angular_threshold:  # Left
                    twist.linear.x = 1.0  # Left
                elif hand.palm.orientation.z * RAD_TO_DEG < self.angular_threshold and hand.palm.orientation.z * RAD_TO_DEG > -self.angular_threshold:  # Neutral
                    twist.linear.x = 0.0  # Neutral
                elif hand.palm.orientation.z * RAD_TO_DEG < -self.angular_threshold:  # Right
                    twist.linear.x = -1.0  # Right

                # Yaw Left/Right movements
                if hand.palm.orientation.y * RAD_TO_DEG > self.angular_threshold:  # Rotate CCW
                    twist.angular.z = 0.5  # Rotate CCW
                elif hand.palm.orientation.y * RAD_TO_DEG < self.angular_threshold and hand.palm.orientation.y * RAD_TO_DEG > -self.angular_threshold:  # Neutral
                    twist.angular.z = 0.0  # Neutral
                elif hand.palm.orientation.y * RAD_TO_DEG < -self.angular_threshold:  # Rotate CW
                    twist.angular.z = -0.5  # Rotate CW

                # print(f"Position - \t x:{hand.palm.position.x: .4f} \t y:{hand.palm.position.y: .4f} \t z:{hand.palm.position.z: .4f}")
                # print(f"Orientation - \t x:{hand.palm.orientation.x: .4f} \t y:{hand.palm.orientation.y: .4f} \t z:{hand.palm.orientation.z: .4f} \t w:{hand.palm.orientation.w: .4f}")
            else:
                twist.linear.z = 0.0  # Neutral
                twist.linear.y = 0.0  # Neutral
                twist.linear.x = 0.0  # Neutral
                twist.angular.z = 0.0  # Neutral
        if self.twist_changed(twist):
            self.pub.publish(twist)
            self.last_twist = twist
            print(f"X:{twist.linear.x: .1f} Y:{twist.linear.y: .1f} Z:{twist.linear.z: .1f} Yaw:{twist.angular.z: .1f}")

    def twist_changed(self, new_twist):
        return (
            new_twist.linear.x != self.last_twist.linear.x or
            new_twist.linear.y != self.last_twist.linear.y or
            new_twist.linear.z != self.last_twist.linear.z or
            new_twist.angular.x != self.last_twist.angular.x or
            new_twist.angular.y != self.last_twist.angular.y or
            new_twist.angular.z != self.last_twist.angular.z
        )


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        if key == '\x1b':  # if the first character is \x1b, we might be dealing with an arrow key
            additional_chars = sys.stdin.read(2)  # read the next two characters
            key += additional_chars  # append these characters to the key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )


    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)

    arm_toggle = False
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)
    
    my_listener = MyListener(pub)
    connection = leap.Connection()
    connection.add_listener(my_listener)

    running = True

    try:
        print(msg)
        
        with connection.open():
            connection.set_tracking_mode(leap.TrackingMode.Desktop)
            while running:
                key = getKey(settings)
                if key == ' ':  # ASCII value for space
                    arm_toggle = not arm_toggle  # Flip the value of arm_toggle
                    arm_msg = std_msgs.msg.Bool()
                    arm_msg.data = arm_toggle
                    arm_pub.publish(arm_msg)
                    print(f"Arm toggle is now: {arm_toggle}")
                time.sleep(1)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down...")
    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()