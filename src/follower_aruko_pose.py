#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import csv

class TagPathRecorder:
    def __init__(self):
        self.tag_path = []
        
        self.tag_sub = rospy.Subscriber('/aruco_simple/pose2', Pose, self.tag_callback)
        
        self.tag_file = open('tag_path.csv', 'w')
        self.tag_writer = csv.writer(self.tag_file)
        
        self.tag_writer.writerow(['time', 'x', 'y', 'z'])

    def tag_callback(self, data):
        x = data.position.x
        y = data.position.y
        z = data.position.z
        timestamp = rospy.Time.now().to_sec()

        self.tag_writer.writerow([timestamp, x, y, z])
        self.tag_path.append((timestamp, x, y, z))

    def shutdown(self):
        self.tag_file.close()

if __name__ == '__main__':
    rospy.init_node('tag_path_recorder', anonymous=True)
    recorder = TagPathRecorder()
    rospy.on_shutdown(recorder.shutdown)
    rospy.spin()
