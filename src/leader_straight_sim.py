#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_straight():
    # Initialize the ROS node
    rospy.init_node('move_straight', anonymous=True)

    # Create a publisher to the cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10) #/tb3_0/ - sider w symulacji

    # Set the rate to publish the message
    rate = rospy.Rate(10)  # 10 Hz

    # Create a Twist message and set the linear speed
    move_cmd = Twist()
    move_cmd.linear.x = 0.1  # Set linear speed (m/s)
    move_cmd.angular.z = 0.0  # Set angular speed (rad/s)

    # Define the time to move in seconds
    move_time = 10  # Move for 5 seconds
    start_time = rospy.Time.now()

    rospy.loginfo("Robot moving straight...")

    while rospy.Time.now() - start_time < rospy.Duration(move_time):
        # Publish the Twist message
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot after moving for the specified time
    move_cmd.linear.x = 0.0
    cmd_vel_pub.publish(move_cmd)
    rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    try:
        move_straight()
    except rospy.ROSInterruptException:
        pass
