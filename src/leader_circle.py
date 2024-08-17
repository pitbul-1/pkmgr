#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

def move_in_circle(radius, linear_speed):
    rospy.init_node('turtlebot3_circle', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10) # 10 Hz

    twist = Twist()
    twist.linear.x = linear_speed
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0

    # Calculate the angular velocity based on the linear speed and radius
    twist.angular.z = linear_speed / radius

    start_time = rospy.Time.now()  # Record the start time

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = (current_time - start_time).to_sec()

        if elapsed_time < 20:  # Check if 20 seconds have passed
            cmd_vel_pub.publish(twist)
        else:
            twist.linear.x = 0.0  # Stop the robot
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        # Define the radius and linear speed
        radius = 0.4  # meters
        linear_speed = 0.1  # meters per second

        move_in_circle(radius, linear_speed)
    except rospy.ROSInterruptException:
        pass
