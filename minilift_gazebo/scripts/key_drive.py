#! /usr/bin/env python
"""
Teleoperation using arrow keys
"""
import threading
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key


def set_interval(interval):
    """
    Set Interval
    """
    def decorator(function):
        """
        Decorator
        """
        def wrapper(*args, **kwargs):
            """
            Wrapper
            """
            stopped = threading.Event()

            def loop(): # executed in another thread
                """
                Loop
                """
                while not stopped.wait(interval): # until stopped
                    function(*args, **kwargs)

            thread = threading.Thread(target=loop)
            thread.daemon = True # stop if the program exits
            thread.start()
            return stopped
        return wrapper
    return decorator



class KeyDrive():
    """
    Class to teleoperate the robot
    """

    def __init__(self):

        self.update_rate = 50
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 2.0

        # Twist object
        self.obj = Twist()
        # Publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Start Timer
        self.timer = self.timer_cb()

    def forward(self):
        """
        Move Forward
        """
        self.obj.linear.x = float(self.max_linear_velocity)
        self.obj.angular.z = 0.0
        self.pub.publish(self.obj)

    def backward(self):
        """
        Move Backward
        """
        self.obj.linear.x = float(-self.max_linear_velocity/2)
        self.obj.angular.z = 0.0
        self.pub.publish(self.obj)

    def left(self):
        """
        Move Left
        """
        self.obj.linear.x = 0.0
        self.obj.angular.z = float(self.max_angular_velocity)
        self.pub.publish(self.obj)

    def right(self):
        """
        Move Right
        """
        self.obj.linear.x = 0.0
        self.obj.angular.z = float(-self.max_angular_velocity)
        self.pub.publish(self.obj)

    def brutestop(self):
        """
        Stop the robot
        """
        self.obj = Twist()
        self.pub.publish(self.obj)

    def key_press(self, key):
        """
        Listen for key press
        """
        if key == Key.up:
            self.forward()
        elif key == Key.down:
            self.backward()
        elif key == Key.right:
            self.right()
        elif key == Key.left:
            self.left()
        return False

    def key_release(self, key):
        """
        Listen for key release
        """
        self.brutestop()
        return False

    @set_interval(0.02)   # Update rate
    def timer_cb(self):
        """
        Timed function
        """

        with keyboard.Listener(on_press=self.key_press) \
            as listener_for_key_press:
            listener_for_key_press.join()

        with keyboard.Listener(on_release=self.key_release) \
            as listener_for_key_release:
            listener_for_key_release.join()

    def __del__(self):
        self.timer.set() # Stop Timer


if __name__ == '__main__':

    rospy.init_node('key_teleop', anonymous=True)
    KeyDrive()
    rospy.spin()
