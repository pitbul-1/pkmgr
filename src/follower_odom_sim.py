#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import csv

class FollowerPathRecorder:
    def __init__(self):
        self.follower_path = []
        
        self.follower_sub = rospy.Subscriber('/tb3_1/odom', Odometry, self.follower_callback)
        
        self.follower_file = open('follower_path.csv', 'w')
        self.follower_writer = csv.writer(self.follower_file)
        
        self.follower_writer.writerow(['time', 'x', 'y', 'z'])

    def follower_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        timestamp = data.header.stamp.to_sec()
        self.follower_writer.writerow([timestamp, x, y, z])
        self.follower_path.append((timestamp, x, y, z))

    def shutdown(self):
        self.follower_file.close()

if __name__ == '__main__':
    rospy.init_node('follower_path_recorder', anonymous=True)
    recorder = FollowerPathRecorder()
    rospy.on_shutdown(recorder.shutdown)
    rospy.spin()
