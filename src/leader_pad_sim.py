#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class XboxTeleop:
    def __init__(self):
        rospy.init_node('xbox_teleop')

        self.pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.twist = Twist()

        # Xbox controller axes indices
        self.axis_linear = 1  # Left stick vertical axis
        self.axis_angular = 0  # Right stick horizontal axis

        # Linear and angular speed multipliers
        self.linear_speed = 0.1
        self.angular_speed = 0.6

    def joy_callback(self, data):
        self.twist.linear.x = self.linear_speed * data.axes[self.axis_linear]
        self.twist.angular.z = self.angular_speed * data.axes[self.axis_angular]
        self.pub.publish(self.twist)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    teleop = XboxTeleop()
    teleop.run()
