#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import csv
import os

class Follower:
    def __init__(self):
        rospy.init_node('follower_lidar_node', anonymous=True)

        # Parameters for minimal and maximal detection distances
        self.min_detect_distance = rospy.get_param('~min_detect_distance', 0.2)  # Minimum detection distance (in meters)
        self.max_detect_distance = rospy.get_param('~max_detect_distance', 0.8)  # Maximum detection distance (in meters)

        self.lidar_sub = rospy.Subscriber('/tb3_0/scan', LaserScan, self.lidar_callback)
        self.distance = None
        self.angle = None

        # Prepare the CSV file
        self.csv_file = os.path.join(os.path.expanduser('~'), 'follower_lidar.csv')
        with open(self.csv_file, 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'Distance (m)', 'Angle (rad)'])  # Write the header

    def lidar_callback(self, data):
        # Convert the LaserScan data to a numpy array
        ranges = np.array(data.ranges)

        # Replace inf values with a large number
        ranges[ranges == np.inf] = data.range_max

        # Filter ranges based on min and max detection distances
        valid_indices = (ranges >= self.min_detect_distance) & (ranges <= self.max_detect_distance)
        filtered_ranges = ranges[valid_indices]

        # If no valid ranges are found, log a message and return
        if filtered_ranges.size == 0:
            rospy.loginfo("No valid object detected within the specified range.")
            return

        # Find the closest object
        min_distance_index = np.argmin(filtered_ranges)
        self.distance = filtered_ranges[min_distance_index]
        self.angle = data.angle_min + np.where(valid_indices)[0][min_distance_index] * data.angle_increment

        # Get the current time
        current_time = rospy.get_time()

        # Save the data to the CSV file
        with open(self.csv_file, 'a') as file:
            writer = csv.writer(file)
            writer.writerow([current_time, self.distance, self.angle])

        rospy.loginfo("Distance: {:.6f} m, Angle: {:.6f} rad".format(self.distance, self.angle))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = Follower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
