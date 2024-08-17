#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

def move_straight():
    # Initialize the ROS node
    rospy.init_node('turtlebot3_straight', anonymous=True)
    
    # Publisher to the /cmd_vel topic to control the robot's velocity
    vel_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
    
    # Define the rate (10 Hz)
    rate = rospy.Rate(10)  # 10 Hz
    
    # Define the Twist message to set linear and angular velocities
    vel_msg = Twist()

    # Set linear velocity (v) and angular velocity (w) to move in a circle
    duration = 40.0  # seconds
    linear_speed = 0.1  # meters per second (you can adjust this value)
    angular_speed = 0.0  # rad/s, v = r * w

    # Set the velocity in the Twist message
    vel_msg.linear.x = linear_speed
    vel_msg.angular.z = angular_speed

    # Move the robot in a circle for 10 seconds
    start_time = rospy.Time.now().to_sec()
    while start_time == 0:
        start_time = rospy.Time.now().to_sec()
    
    while rospy.Time.now().to_sec() - start_time < duration:
        elapsed_time = rospy.Time.now().to_sec() - start_time
        print("Elapsed time: {:.2f} seconds".format(elapsed_time))
        # Publish the velocity command
        vel_pub.publish(vel_msg)
        # Sleep for a while before the next iteration
        rate.sleep()

    # Stop the robot when Ctrl+C is pressed
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)
    rospy.loginfo("Robot stopped.")


if __name__ == '__main__':
    move_straight()
