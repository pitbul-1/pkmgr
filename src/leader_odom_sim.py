#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import csv

class LeaderPathRecorder:
    def __init__(self):
        self.leader_path = []
        
        self.leader_sub = rospy.Subscriber('/tb3_0/odom', Odometry, self.leader_callback)
        
        self.leader_file = open('leader_path.csv', 'w')
        self.leader_writer = csv.writer(self.leader_file)
        
        self.leader_writer.writerow(['time', 'x', 'y', 'z'])

    def leader_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        timestamp = data.header.stamp.to_sec()
        self.leader_writer.writerow([timestamp, x, y, z])
        self.leader_path.append((timestamp, x, y, z))

    def shutdown(self):
        self.leader_file.close()

if __name__ == '__main__':
    rospy.init_node('leader_path_recorder', anonymous=True)
    recorder = LeaderPathRecorder()
    rospy.on_shutdown(recorder.shutdown)
    rospy.spin()
