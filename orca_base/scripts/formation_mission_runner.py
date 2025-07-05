#!/usr/bin/env python3

"""
Multi-AUV Formation Mission Runner

Simple mission script that sends waypoints to AUV1 (leader).
AUV2 and AUV3 will automatically follow in formation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from mavros_msgs.msg import State
import time
import math


class FormationMissionRunner(Node):
    """
    Mission runner that sends simple movement commands to the leader AUV.
    """

    def __init__(self):
        super().__init__('formation_mission_runner')
        
        # Publisher for AUV1 (leader) commands
        self.auv1_cmd_pub = self.create_publisher(Twist, '/auv1/cmd_vel', 10)
        
        # Service clients for base controller connections
        self.auv1_conn_client = self.create_client(SetBool, '/auv1/conn')
        self.auv2_conn_client = self.create_client(SetBool, '/auv2/conn')
        self.auv3_conn_client = self.create_client(SetBool, '/auv3/conn')
        
        # MAVROS state subscribers to check connection status
        self.auv1_mavros_state = None
        self.auv2_mavros_state = None
        self.auv3_mavros_state = None
        
        self.auv1_state_sub = self.create_subscription(State, '/auv1/mavros/state', self.auv1_state_callback, 10)
        self.auv2_state_sub = self.create_subscription(State, '/auv2/mavros/state', self.auv2_state_callback, 10)
        self.auv3_state_sub = self.create_subscription(State, '/auv3/mavros/state', self.auv3_state_callback, 10)
        
        self.get_logger().info('Formation Mission Runner initialized')
        self.get_logger().info('Controlling AUV1 (leader), followers will maintain formation')

    def auv1_state_callback(self, msg):
        self.auv1_mavros_state = msg

    def auv2_state_callback(self, msg):
        self.auv2_mavros_state = msg

    def auv3_state_callback(self, msg):
        self.auv3_mavros_state = msg

    def wait_for_connections(self, timeout=30.0):
        """Wait for base controller connection services to be available"""
        self.get_logger().info('Waiting for base controller connection services...')
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if (self.auv1_conn_client.service_is_ready() and 
                self.auv2_conn_client.service_is_ready() and 
                self.auv3_conn_client.service_is_ready()):
                self.get_logger().info('All base controller connection services ready!')
                return True
            time.sleep(0.5)
        
        self.get_logger().warn('Timeout waiting for base controller services - continuing anyway')
        return False

    def enable_base_controllers(self):
        """Enable all base controllers"""
        self.get_logger().info('Enabling base controllers...')
        
        # Create service request
        request = SetBool.Request()
        request.data = True
        
        # Enable AUV1 base controller
        if self.auv1_conn_client.service_is_ready():
            try:
                future = self.auv1_conn_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None:
                    self.get_logger().info(f'AUV1 base controller: {future.result().message}')
                else:
                    self.get_logger().warn('AUV1 base controller service call failed')
            except Exception as e:
                self.get_logger().warn(f'AUV1 base controller service error: {e}')
        else:
            self.get_logger().warn('AUV1 base controller connection service not available')
        
        # Enable AUV2 base controller
        if self.auv2_conn_client.service_is_ready():
            try:
                future = self.auv2_conn_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None:
                    self.get_logger().info(f'AUV2 base controller: {future.result().message}')
                else:
                    self.get_logger().warn('AUV2 base controller service call failed')
            except Exception as e:
                self.get_logger().warn(f'AUV2 base controller service error: {e}')
        else:
            self.get_logger().warn('AUV2 base controller connection service not available')
        
        # Enable AUV3 base controller
        if self.auv3_conn_client.service_is_ready():
            try:
                future = self.auv3_conn_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None:
                    self.get_logger().info(f'AUV3 base controller: {future.result().message}')
                else:
                    self.get_logger().warn('AUV3 base controller service call failed')
            except Exception as e:
                self.get_logger().warn(f'AUV3 base controller service error: {e}')
        else:
            self.get_logger().warn('AUV3 base controller connection service not available')

    def check_mavros_connections(self):
        """Check MAVROS connection status for all AUVs"""
        self.get_logger().info('Checking MAVROS connections...')
        
        # Wait a bit for state messages
        time.sleep(2.0)
        
        if self.auv1_mavros_state:
            self.get_logger().info(f'AUV1 MAVROS - Connected: {self.auv1_mavros_state.connected}, Armed: {self.auv1_mavros_state.armed}, Mode: {self.auv1_mavros_state.mode}')
        else:
            self.get_logger().warn('AUV1 MAVROS state not received')
        
        if self.auv2_mavros_state:
            self.get_logger().info(f'AUV2 MAVROS - Connected: {self.auv2_mavros_state.connected}, Armed: {self.auv2_mavros_state.armed}, Mode: {self.auv2_mavros_state.mode}')
        else:
            self.get_logger().warn('AUV2 MAVROS state not received')
        
        if self.auv3_mavros_state:
            self.get_logger().info(f'AUV3 MAVROS - Connected: {self.auv3_mavros_state.connected}, Armed: {self.auv3_mavros_state.armed}, Mode: {self.auv3_mavros_state.mode}')
        else:
            self.get_logger().warn('AUV3 MAVROS state not received')

    def move_forward(self, speed=0.3, duration=5.0):
        """Move forward at specified speed for given duration"""
        self.get_logger().info(f'Moving forward at {speed} m/s for {duration} seconds')
        
        cmd = Twist()
        cmd.linear.x = speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.auv1_cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop
        cmd.linear.x = 0.0
        self.auv1_cmd_pub.publish(cmd)

    def turn(self, angular_speed=0.3, duration=3.0):
        """Turn at specified angular speed for given duration"""
        self.get_logger().info(f'Turning at {angular_speed} rad/s for {duration} seconds')
        
        cmd = Twist()
        cmd.angular.z = angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.auv1_cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop
        cmd.angular.z = 0.0
        self.auv1_cmd_pub.publish(cmd)

    def stop(self, duration=2.0):
        """Stop for specified duration"""
        self.get_logger().info(f'Stopping for {duration} seconds')
        
        cmd = Twist()
        start_time = time.time()
        while time.time() - start_time < duration:
            self.auv1_cmd_pub.publish(cmd)
            time.sleep(0.1)

    def run_formation_mission(self):
        """Execute a simple formation mission"""
        self.get_logger().info('Starting formation mission!')
        
        # Wait for connections and enable base controllers
        self.wait_for_connections()
        self.enable_base_controllers()
        self.check_mavros_connections()
        
        # Wait a bit for everything to initialize
        self.stop(5.0)
        
        # Mission sequence
        self.move_forward(speed=0.2, duration=8.0)  # Move forward slowly
        self.stop(2.0)
        
        self.turn(angular_speed=0.2, duration=5.0)  # Turn right
        self.stop(2.0)
        
        self.move_forward(speed=0.25, duration=6.0)  # Move forward
        self.stop(2.0)
        
        self.turn(angular_speed=-0.2, duration=5.0)  # Turn left
        self.stop(2.0)
        
        self.move_forward(speed=0.2, duration=8.0)  # Move forward
        self.stop(2.0)
        
        self.turn(angular_speed=0.2, duration=10.0)  # Complete a circle
        self.stop(2.0)
        
        self.get_logger().info('Formation mission completed!')


def main(args=None):
    rclpy.init(args=args)
    
    mission_runner = FormationMissionRunner()
    
    try:
        # Run the formation mission
        mission_runner.run_formation_mission()
        
        # Keep the node alive
        rclpy.spin(mission_runner)
        
    except KeyboardInterrupt:
        mission_runner.get_logger().info('Mission interrupted by user')
    
    mission_runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
