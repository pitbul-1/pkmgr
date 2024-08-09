#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion
import math
import time

class FollowerRobot:
    def __init__(self):
        rospy.init_node('follower_robot', anonymous=True)
        
        # Parameters
        self.target_distance = 0.7  # Target distance from the leader
        self.target_angle = 1.570796327  # Target angle relative to the leader (in radians)
        self.tag_to_leader_offset = 0.07  # Offset of the tag from the center of the leader robot
        self.camera_to_follower_offset = 0.06  # Offset of the camera from the center of the follower robot
        self.max_linear_speed = 0.22  # Maximum linear speed
        self.max_angular_speed = 2.84  # Maximum angular speed
        self.timeout_duration = 1.0  # Timeout duration in seconds
        
        # PID gains for linear speed
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.05
        
        # PID gains for angular speed
        self.kp_angular = 2.0
        self.ki_angular = 0.0
        self.kd_angular = 0.05
        
        # Subscribers and Publishers
        self.pose_sub = rospy.Subscriber('/aruco_simple/pose2', Pose, self.pose_callback)
        self.cmd_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        
        # Variables to store pose data and time
        self.tag_pose = None
        self.last_detection_time = time.time()
        
        # PID error terms
        self.prev_distance_error = 0.0
        self.integral_distance_error = 0.0
        self.prev_angle_error = 0.0
        self.integral_angle_error = 0.0
        self.prev_time = time.time()

        
    def pose_callback(self, data):
        self.tag_pose = data
        self.last_detection_time = time.time()
        
    def compute_control_command(self):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        if self.tag_pose is None or (current_time - self.last_detection_time > self.timeout_duration):
            # Stop the robot if no tag is detected within the timeout duration
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = 0
            return cmd
        
        # Extract position from the leader pose
        leader_x = self.tag_pose.position.x
        leader_y = self.tag_pose.position.z + self.tag_to_leader_offset + self.camera_to_follower_offset

        ## Convert quaternion to euler angles
        #orientation_list = [leader_orientation.x, leader_orientation.y, leader_orientation.z, leader_orientation.w]
        #(roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Compute distance and angle to the leader
        distance = math.sqrt(leader_x**2 + leader_y**2)
        angle_to_leader = math.atan2(leader_y, leader_x)
        
        # Compute the error in distance and angle
        distance_error = distance - self.target_distance
        angle_error = angle_to_leader - self.target_angle
        
        # Integral of the errors
        self.integral_distance_error += distance_error * dt
        self.integral_angle_error += angle_error * dt

        # Derivative of the errors
        distance_error_derivative = (distance_error - self.prev_distance_error) / dt
        angle_error_derivative = (angle_error - self.prev_angle_error) / dt
        
        self.prev_distance_error = distance_error
        self.prev_angle_error = angle_error
        
        # Control law for linear and angular velocities (PID control)
        raw_linear_speed = (self.kp_linear * distance_error +
                            self.ki_linear * self.integral_distance_error +
                            self.kd_linear * distance_error_derivative)
        
        raw_angular_speed = (self.kp_angular * angle_error +
                             self.ki_angular * self.integral_angle_error +
                             self.kd_angular * angle_error_derivative)
        
        # Clamp the raw speeds to their maximum values
        linear_speed = max(min(raw_linear_speed, self.max_linear_speed), -self.max_linear_speed)
        angular_speed = max(min(raw_angular_speed, self.max_angular_speed), -self.max_angular_speed)
        
        # Create and populate the Twist message
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        
        return cmd
    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            cmd = self.compute_control_command()
            if cmd:
                self.cmd_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        follower_robot = FollowerRobot()
        follower_robot.run()
    except rospy.ROSInterruptException:
        pass
