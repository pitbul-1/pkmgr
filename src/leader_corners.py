#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

def move_straight(distance, speed):
    velocity_msg = Twist()
    velocity_msg.linear.x = speed
    velocity_msg.angular.z = 0.0

    duration = distance / speed
    end_time = rospy.Time.now() + rospy.Duration(duration)
    
    while rospy.Time.now() < end_time:
        pub.publish(velocity_msg)
        rate.sleep()

    # Stop the robot after moving straight
    velocity_msg.linear.x = 0.0
    pub.publish(velocity_msg)

def rotate(angle, angular_velocity_rad):
    velocity_msg = Twist()
    velocity_msg.linear.x = 0.0
    
    velocity_msg.angular.z = angular_velocity_rad

    # Convert angle from degrees to radians
    angle_rad = math.radians(angle)

    duration = abs(angle_rad / angular_velocity_rad)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    
    while rospy.Time.now() < end_time:
        pub.publish(velocity_msg)
        rate.sleep()

    # Stop the robot after rotating
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot3_path_controller', anonymous=True)
        pub = rospy.Publisher('lead/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)  # 10 Hz
        
        while rospy.Time.now().to_sec() == 0: # Wait for the clock to sync with the simulation
            pass

        # Parameters
        straight_speed = 0.1  # meters per second
        straight_distance = 1  # meters
        angular_velocity_rad = 0.5  # radians per second

        # Turn angles
        turn_angles = [20, 40, 60, 80]  # Degrees

        # Move and rotate according to the specified path
        for angle in turn_angles:
            move_straight(straight_distance, straight_speed)
            rotate(angle, angular_velocity_rad=angular_velocity_rad)
        
        # Add one more straight move after the last rotate
        move_straight(straight_distance, straight_speed)

        # Stop the robot at the end of the path
        pub.publish(Twist())
        rospy.loginfo("Path completed, robot stopped.")

    except rospy.ROSInterruptException:
        pass
