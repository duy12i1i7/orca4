#!/usr/bin/env python3

"""
Multi-AUV Formation Controller

Basic formation controller that makes AUV2 and AUV3 follow AUV1 in a triangular formation.
AUV1 is the leader, AUV2 and AUV3 are followers positioned behind and to the sides.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math


class FormationController(Node):
    """
    Formation controller that manages a 3-AUV formation.
    """

    def __init__(self):
        super().__init__('formation_controller')
        
        # Formation parameters
        self.formation_distance = 3.0  # meters behind leader
        self.formation_offset = 2.0    # meters to the side
        
        # Current positions
        self.auv1_pose = None
        self.auv2_pose = None
        self.auv3_pose = None
        
        # Control gains
        self.kp_linear = 0.5
        self.kp_angular = 1.0
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        
        # Subscribers to odometry
        self.auv1_odom_sub = self.create_subscription(
            Odometry,
            '/model/auv1/odometry',
            self.auv1_odom_callback,
            10
        )
        
        self.auv2_odom_sub = self.create_subscription(
            Odometry,
            '/model/auv2/odometry',
            self.auv2_odom_callback,
            10
        )
        
        self.auv3_odom_sub = self.create_subscription(
            Odometry,
            '/model/auv3/odometry',
            self.auv3_odom_callback,
            10
        )
        
        # Publishers for velocity commands
        self.auv2_cmd_pub = self.create_publisher(Twist, '/auv2/cmd_vel', 10)
        self.auv3_cmd_pub = self.create_publisher(Twist, '/auv3/cmd_vel', 10)
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_formation)
        
        self.get_logger().info('Formation Controller initialized')
        self.get_logger().info('AUV1 is the leader, AUV2 and AUV3 will follow in formation')

    def auv1_odom_callback(self, msg):
        """Store AUV1 (leader) position"""
        self.auv1_pose = msg.pose.pose

    def auv2_odom_callback(self, msg):
        """Store AUV2 position"""
        self.auv2_pose = msg.pose.pose

    def auv3_odom_callback(self, msg):
        """Store AUV3 position"""
        self.auv3_pose = msg.pose.pose

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_desired_position(self, leader_pose, side_offset):
        """Calculate desired position for follower relative to leader"""
        if leader_pose is None:
            return None, None
            
        # Get leader's position and orientation
        leader_x = leader_pose.position.x
        leader_y = leader_pose.position.y
        leader_yaw = self.quaternion_to_yaw(leader_pose.orientation)
        
        # Calculate desired position behind and to the side of the leader
        desired_x = leader_x - self.formation_distance * math.cos(leader_yaw) + \
                   side_offset * math.sin(leader_yaw)
        desired_y = leader_y - self.formation_distance * math.sin(leader_yaw) - \
                   side_offset * math.cos(leader_yaw)
        
        return desired_x, desired_y

    def calculate_control_command(self, current_pose, desired_x, desired_y):
        """Calculate velocity command to reach desired position"""
        if current_pose is None or desired_x is None or desired_y is None:
            return Twist()
            
        # Current position and orientation
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        current_yaw = self.quaternion_to_yaw(current_pose.orientation)
        
        # Position error
        dx = desired_x - current_x
        dy = desired_y - current_y
        distance_error = math.sqrt(dx*dx + dy*dy)
        
        # Desired heading
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - current_yaw
        
        # Normalize yaw error to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Create velocity command
        cmd = Twist()
        
        # Linear velocity (proportional to distance error)
        cmd.linear.x = min(self.kp_linear * distance_error, self.max_linear_speed)
        
        # Angular velocity (proportional to yaw error)
        cmd.angular.z = max(min(self.kp_angular * yaw_error, self.max_angular_speed), 
                           -self.max_angular_speed)
        
        return cmd

    def control_formation(self):
        """Main control loop for formation"""
        if self.auv1_pose is None:
            return  # Wait for leader position
        
        # Control AUV2 (left follower)
        if self.auv2_pose is not None:
            desired_x, desired_y = self.calculate_desired_position(self.auv1_pose, self.formation_offset)
            cmd2 = self.calculate_control_command(self.auv2_pose, desired_x, desired_y)
            self.auv2_cmd_pub.publish(cmd2)
        
        # Control AUV3 (right follower)
        if self.auv3_pose is not None:
            desired_x, desired_y = self.calculate_desired_position(self.auv1_pose, -self.formation_offset)
            cmd3 = self.calculate_control_command(self.auv3_pose, desired_x, desired_y)
            self.auv3_cmd_pub.publish(cmd3)


def main(args=None):
    rclpy.init(args=args)
    
    formation_controller = FormationController()
    
    try:
        rclpy.spin(formation_controller)
    except KeyboardInterrupt:
        pass
    
    formation_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
