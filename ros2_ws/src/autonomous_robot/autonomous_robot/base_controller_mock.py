#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from datetime import datetime

class BaseControllerMock(Node):
    def __init__(self):
        super().__init__('base_controller_mock')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        # Timer for publishing
        self.timer = self.create_timer(0.05, self.publish_mock_data)  # 20Hz
        
        self.get_logger().info('Base Controller Mock Node Started')
    
    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity commands"""
        self.vx = msg.linear.x
        self.vth = msg.angular.z
        self.get_logger().info(f'Received: linear={self.vx:.2f}, angular={self.vth:.2f}')
    
    def publish_mock_data(self):
        """Publish mock odometry and IMU data"""
        now = self.get_clock().now()
        
        # Update pose (simple integration)
        dt = 0.05
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.vth * dt
        
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q_z = math.sin(self.theta / 2)
        q_w = math.cos(self.theta / 2)
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        
        # Publish IMU (dummy data)
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'imu_link'
        
        # Dummy IMU data
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81
        
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = self.vth
        
        self.imu_pub.publish(imu)
        
        # Publish TF (odom → base_link)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = BaseControllerMock()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
