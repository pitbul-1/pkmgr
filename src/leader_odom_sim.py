#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import csv
import tf.transformations as tf

class LeaderPathRecorder:
    def __init__(self):
        self.follower_path = []
        
        self.follower_sub = rospy.Subscriber('/tb3_1/odom', Odometry, self.follower_callback)
        
        self.follower_file = open('leader_path.csv', 'w')
        self.follower_writer = csv.writer(self.follower_file)
        
        self.follower_writer.writerow(['time', 'x', 'y', 'z', 'theta'])

    def follower_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        orientation_q = data.pose.pose.orientation
        timestamp = data.header.stamp.to_sec()

        # Convert quaternion to Euler angles
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, theta = tf.euler_from_quaternion(orientation_list)

        self.follower_writer.writerow([timestamp, x, y, z, theta])
        self.follower_path.append((timestamp, x, y, z, theta))

    def shutdown(self):
        self.follower_file.close()

if __name__ == '__main__':
    rospy.init_node('leader_path_recorder', anonymous=True)
    recorder = LeaderPathRecorder()
    rospy.on_shutdown(recorder.shutdown)
    rospy.spin()