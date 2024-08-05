#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
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
        self.prediction_horizon = 1.0  # Time horizon for prediction in seconds
        
        # PID gains for linear speed
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.05
        
        # PID gains for angular speed (distance-based angle control)
        self.kp_angular = 2.0
        self.ki_angular = 0.0
        self.kd_angular = 0.05

        # PID gains for orientation angle correction
        self.kp_orientation = 0.0 
        self.ki_orientation = 0.0 
        self.kd_orientation = 0.0
        
        # Subscribers and Publishers
        self.pose_sub = rospy.Subscriber('/aruco_simple/pose2', Pose, self.pose_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Variables to store pose data and time
        self.tag_pose = None
        self.prev_tag_pose = None
        self.follower_yaw = None
        self.last_detection_time = time.time()
        
        # PID error terms for distance-based angle control
        self.prev_distance_error = 0.0
        self.integral_distance_error = 0.0
        self.prev_angle_error = 0.0
        self.integral_angle_error = 0.0

        # PID error terms for orientation correction
        self.prev_orientation_error = 0.0
        self.integral_orientation_error = 0.0
        
        self.prev_time = time.time()
        
    def pose_callback(self, data):
        self.prev_tag_pose = self.tag_pose
        self.tag_pose = data
        self.last_detection_time = time.time()
        
    def odom_callback(self, data):
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.follower_yaw = euler_from_quaternion(orientation_list)
        #print("self.follower_yaw:", self.follower_yaw)

    def predict_leader_pose(self, dt):
        if self.prev_tag_pose is None:
            return self.tag_pose
        
        # Calculate velocities
        leader_dx = (self.tag_pose.position.x - self.prev_tag_pose.position.x) / dt
        leader_dz = (self.tag_pose.position.z - self.prev_tag_pose.position.z) / dt
        
        # Predict future position
        predicted_pose = Pose()
        predicted_pose.position.x = self.tag_pose.position.x + leader_dx * self.prediction_horizon
        predicted_pose.position.z = self.tag_pose.position.z + leader_dz * self.prediction_horizon
        predicted_pose.orientation = self.tag_pose.orientation
        
        return predicted_pose

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
        
        if self.follower_yaw is None:
            # If follower yaw is not available yet, wait
            return None
        
        # Predict future leader pose
        predicted_pose = self.predict_leader_pose(dt)
        
        # Extract position and orientation from the predicted leader pose
        leader_x = predicted_pose.position.x
        leader_y = predicted_pose.position.z + self.tag_to_leader_offset + self.camera_to_follower_offset
        
        # Convert quaternion to euler angles for the leader
        orientation_list = [predicted_pose.orientation.x, predicted_pose.orientation.y, predicted_pose.orientation.z, predicted_pose.orientation.w]
        _, leader_yaw_fliped, _ = euler_from_quaternion(orientation_list)
        leader_yaw = -1*leader_yaw_fliped
        print("leader_yaw:", leader_yaw)
        
        # Compute distance and angle to the leader
        distance = math.sqrt(leader_x**2 + leader_y**2)
        angle_to_leader = math.atan2(leader_y, leader_x)
        
        # Compute the error in distance and angle
        distance_error = distance - self.target_distance
        angle_error = angle_to_leader - self.target_angle
        
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
        angular_speed_combined = raw_angular_speed + orientation_correction
        
        # Clamp the raw speeds to their maximum values
        linear_speed = max(min(raw_linear_speed, self.max_linear_speed), -self.max_linear_speed)
        angular_speed = max(min(angular_speed_combined, self.max_angular_speed), -self.max_angular_speed)
        
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
