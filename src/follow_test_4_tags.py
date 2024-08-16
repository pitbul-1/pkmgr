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
        self.tag_to_leader_offset = 0.07  # Offset of the tag from the center of the leader robot
        self.camera_to_follower_offset = 0.06  # Offset of the camera from the center of the follower robot
        self.max_linear_speed = 0.22  # Maximum linear speed
        self.max_angular_speed = 2.84  # Maximum angular speed
        self.timeout_duration = 1.0  # Timeout duration in seconds
        
        # PID gains for linear speed
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.05
        
        # PID gains for angular speed (distance-based angle control)
        self.kp_angular = 2.0
        self.ki_angular = 0.0
        self.kd_angular = 0.05

        # PID gains for orientation angle correction
        self.kp_orientation = 1.0
        self.ki_orientation = 0.0
        self.kd_orientation = 0.05
        
        # Subscribers and Publishers
        self.pose_sub_back = rospy.Subscriber('/aruco_front_rear/pose2', Pose, self.pose_callback_back)
        self.pose_sub_front = rospy.Subscriber('/aruco_front_rear/pose', Pose, self.pose_callback_front)
        self.pose_sub_left = rospy.Subscriber('/aruco_sides/pose2', Pose, self.pose_callback_left)
        self.pose_sub_right = rospy.Subscriber('/aruco_simple/pose', Pose, self.pose_callback_right)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Variables to store pose data and time
        self.tag_pose_back = None
        self.tag_pose_front = None
        self.tag_pose_left = None
        self.tag_pose_right = None
        self.last_detection_time_back = time.time()
        self.last_detection_time_front = time.time()
        self.last_detection_time_left = time.time()
        self.last_detection_time_right = time.time()
        
        # PID error terms for distance-based angle control
        self.prev_distance_error = 0.0
        self.integral_distance_error = 0.0
        self.prev_angle_error = 0.0
        self.integral_angle_error = 0.0

        # PID error terms for orientation correction
        self.prev_orientation_error = 0.0
        self.integral_orientation_error = 0.0
        
        self.prev_time = time.time()
        self.last_leader_position = None  # To store last known leader position (x, y)
        self.last_leader_yaw = None       # To store last known leader yaw
        
    def pose_callback_back(self, data):
        self.tag_pose_back = data
        self.last_detection_time_back = time.time()

    def pose_callback_front(self, data):
        self.tag_pose_front = data
        self.last_detection_time_front = time.time()

    def pose_callback_left(self, data):
        self.tag_pose_left = data
        self.last_detection_time_left = time.time()

    def pose_callback_right(self, data):
        self.tag_pose_right = data
        self.last_detection_time_right = time.time()

    def compute_control_command(self):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # Determine which tag is detected and prioritize the control
        tag_pose = None
        last_detection_time = None
        tag_type = None

        if self.tag_pose_back and (current_time - self.last_detection_time_back <= self.timeout_duration):
            tag_pose = self.tag_pose_back
            last_detection_time = self.last_detection_time_back
            tag_type = 'back'
        elif self.tag_pose_left and (current_time - self.last_detection_time_left <= self.timeout_duration):
            tag_pose = self.tag_pose_left
            last_detection_time = self.last_detection_time_left
            tag_type = 'left'
        elif self.tag_pose_right and (current_time - self.last_detection_time_right <= self.timeout_duration):
            tag_pose = self.tag_pose_right
            last_detection_time = self.last_detection_time_right
            tag_type = 'right'
        elif self.tag_pose_front and (current_time - self.last_detection_time_front <= self.timeout_duration):
            tag_pose = self.tag_pose_front
            last_detection_time = self.last_detection_time_front
            tag_type = 'front'
        
        if not tag_pose or (current_time - last_detection_time > self.timeout_duration):
            # Stop the robot if no tag is detected within the timeout duration
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = 0
            return cmd
        
        # Extract position and orientation from the detected tag
        leader_x = tag_pose.position.x
        leader_y = tag_pose.position.z + self.tag_to_leader_offset + self.camera_to_follower_offset
        
        # Separate yaw correction for each tag type
        if tag_type == 'back':
            orientation_list = [tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w]
            _, leader_yaw_fliped, _ = euler_from_quaternion(orientation_list)
            leader_yaw = -leader_yaw_fliped
        elif tag_type == 'front':
            orientation_list = [tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w]
            _, leader_yaw, _ = euler_from_quaternion(orientation_list)
        elif tag_type == 'left':
            orientation_list = [tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w]
            _, leader_yaw_rotated, _ = euler_from_quaternion(orientation_list)
            leader_yaw = leader_yaw_rotated - math.pi / 2
        elif tag_type == 'right':
            orientation_list = [tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w]
            _, leader_yaw_rotated, _ = euler_from_quaternion(orientation_list)
            leader_yaw = leader_yaw_rotated + math.pi / 2
        
        # Memorize the last known position and yaw of the leader
        self.last_leader_position = (leader_x, leader_y)
        self.last_leader_yaw = leader_yaw
        
        # Execute the maneuver based on the detected tag
        if tag_type == 'front':
            return self.maneuver_for_front_tag(leader_x, leader_y, leader_yaw)
        elif tag_type == 'left':
            return self.maneuver_for_left_tag(leader_x, leader_y, leader_yaw)
        elif tag_type == 'right':
            return self.maneuver_for_right_tag(leader_x, leader_y, leader_yaw)
        
        # If back tag is detected or after maneuver, continue normal tracking
        return self.standard_tracking(leader_x, leader_y, leader_yaw, dt)
    
    def standard_tracking(self, leader_x, leader_y, leader_yaw, dt):
        # Compute distance and angle to the leader
        distance = math.sqrt(leader_x**2 + leader_y**2)
        angle_to_leader = math.atan2(leader_y, leader_x)
        
        # Compute the error in distance and angle
        distance_error = distance - self.target_distance
        angle_error = angle_to_leader
        
        # Compute the orientation error
        orientation_error = leader_yaw
        
        # Integral of the distance and angle errors
        self.integral_distance_error += distance_error * dt
        self.integral_angle_error += angle_error * dt

        # Integral of the orientation error
        self.integral_orientation_error += orientation_error * dt

        # Derivative of the distance and angle errors
        distance_error_derivative = (distance_error - self.prev_distance_error) / dt
        angle_error_derivative = (angle_error - self.prev_angle_error) / dt

        # Derivative of the orientation error
        orientation_error_derivative = (orientation_error - self.prev_orientation_error) / dt
        
        self.prev_distance_error = distance_error
        self.prev_angle_error = angle_error
        self.prev_orientation_error = orientation_error
        
        # Control law for linear speed (PID control)
        raw_linear_speed = (self.kp_linear * distance_error +
                            self.ki_linear * self.integral_distance_error +
                            self.kd_linear * distance_error_derivative)
        
        # Control law for angular speed (distance-based angle control)
        raw_angular_speed = (self.kp_angular * angle_error +
                             self.ki_angular * self.integral_angle_error +
                             self.kd_angular * angle_error_derivative)
        
        # Control law for orientation correction
        orientation_correction = (self.kp_orientation * orientation_error +
                                  self.ki_orientation * self.integral_orientation_error +
                                  self.kd_orientation * orientation_error_derivative)
        
        # Combine angular speed from distance-based angle control and orientation correction
        raw_angular_speed += orientation_correction
        
        # Clamp the raw speeds to their maximum values
        linear_speed = max(min(raw_linear_speed, self.max_linear_speed), -self.max_linear_speed)
        angular_speed = max(min(raw_angular_speed, self.max_angular_speed), -self.max_angular_speed)
        
        # Create and populate the Twist message
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        
        return cmd
    
    def maneuver_for_front_tag(self, leader_x, leader_y, leader_yaw):
        cmd = Twist()

        # Turn π/2 - leader_yaw
        cmd.linear.x = 0
        cmd.angular.z = 1.4  # Set angular speed
        self.cmd_pub.publish(cmd)
        time.sleep((math.pi / 2 - leader_yaw) / cmd.angular.z)

        # Move forward by 0.16 meters
        cmd.linear.x = 0.2  # Set linear speed
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd)
        time.sleep(0.16 / cmd.linear.x)

        # Rotate by π/2
        cmd.linear.x = 0
        cmd.angular.z = -1.4  # Set angular speed (counterclockwise)
        self.cmd_pub.publish(cmd)
        time.sleep(math.pi / 2 / abs(cmd.angular.z))

        # Move forward by 0.4 meters
        cmd.linear.x = 0.2  # Set linear speed
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd)
        time.sleep(0.4 / cmd.linear.x)

        # Rotate by π (180 Degrees)
        cmd.linear.x = 0
        cmd.angular.z = 1.4  # Set angular speed
        self.cmd_pub.publish(cmd)
        time.sleep(math.pi / cmd.angular.z)

        # Stop after maneuver
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd)
        
        return cmd
    
    def maneuver_for_left_tag(self, leader_x, leader_y, leader_yaw):
        cmd = Twist()

        # Turn π/2 - leader_yaw
        cmd.linear.x = 0
        cmd.angular.z = 1.4  # Set angular speed
        self.cmd_pub.publish(cmd)
        time.sleep((math.pi / 2 - leader_yaw) / cmd.angular.z)

        # Move forward by 0.3 meters
        cmd.linear.x = 0.2  # Set linear speed
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd)
        time.sleep(0.3 / cmd.linear.x)

        # Rotate by π (180 Degrees)
        cmd.linear.x = 0
        cmd.angular.z = 1.4  # Set angular speed
        self.cmd_pub.publish(cmd)
        time.sleep(math.pi / cmd.angular.z)

        # Stop after maneuver
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd)
        
        return cmd

    def maneuver_for_right_tag(self, leader_x, leader_y, leader_yaw):
        cmd = Twist()

        # Turn π/2 - leader_yaw
        cmd.linear.x = 0
        cmd.angular.z = -1.4  # Set angular speed (clockwise)
        self.cmd_pub.publish(cmd)
        time.sleep((math.pi / 2 - leader_yaw) / abs(cmd.angular.z))

        # Move forward by 0.3 meters
        cmd.linear.x = 0.2  # Set linear speed
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd)
        time.sleep(0.3 / cmd.linear.x)

        # Rotate by π (180 Degrees)
        cmd.linear.x = 0
        cmd.angular.z = 1.4  # Set angular speed
        self.cmd_pub.publish(cmd)
        time.sleep(math.pi / cmd.angular.z)

        # Stop after maneuver
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd)
        
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
