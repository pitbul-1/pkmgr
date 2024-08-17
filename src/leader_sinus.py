#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist

def move_in_sine_wave():
    # Initialize the ROS node
    rospy.init_node('sine_wave_controller', anonymous=True)

    # Publisher to the /cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Duration to drive in a sine wave
    driving_duration = 60.0  # seconds

    # Define the parameters for the sine wave
    amplitude = 1.0  # Amplitude of the sine wave in meters
    wavelength = 3.0  # Wavelength of the sine wave in meters
    linear_speed = 0.1  # Constant linear speed along the x-axis in m/s
    frequency = linear_speed / wavelength  # Frequency of the sine wave
    angular_speed_amplitude = 2 * math.pi * frequency * amplitude  # Max angular speed to create sine wave

    # Time management
    rate = rospy.Rate(10)  # Control loop rate in Hz (10 Hz)
    start_time = rospy.Time.now().to_sec()
    while start_time == 0:
        start_time = rospy.Time.now().to_sec()

    # Create the Twist message
    twist = Twist()

    while not rospy.is_shutdown():
        print("Running...")
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time
        print("Elapsed time: {:.2f} seconds".format(elapsed_time))
        print("Current time: {:.2f} seconds".format(current_time))
        print("Start time: {:.2f} seconds".format(start_time))
        print("---")

        # Stop the robot after 10 seconds
        if elapsed_time >= driving_duration:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            print("10 seconds elapsed. Stopping the robot.")
            break

        # Calculate the sine wave angular velocity
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed_amplitude * math.cos(2 * math.pi * frequency * elapsed_time)

        # Publish the command
        cmd_vel_pub.publish(twist)

        # Sleep to maintain the loop rate
        rate.sleep()
    print("Node shutting down...")

if __name__ == '__main__':
    try:
        move_in_sine_wave()
    except rospy.ROSInterruptException:
        print("Error ogułemed.")
        pass
