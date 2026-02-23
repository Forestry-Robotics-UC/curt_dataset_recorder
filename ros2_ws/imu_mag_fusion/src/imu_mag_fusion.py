#!/usr/bin/env python3
import rclpy
import rclpy.node
import numpy as np
from ahrs.filters import Madgwick
from sensor_msgs.msg import Imu, MagneticField


class IMUMagFusionNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('imu_mag_fusion_node')
        
        # Declare Parameters
        self.imu_topic = self.declare_parameter('imu_topic', '/imu/data').value
        self.mag_topic = self.declare_parameter('mag_topic', '/mag').value
        self.fused_topic = self.declare_parameter('fused_topic', '/imu/fused').value
        
        # Subscribers and Publisher
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, self.mag_topic, self.mag_callback, 10)
        self.fused_pub = self.create_publisher(Imu, self.fused_topic, 10)
        
        # Initialize variables for storing sensor data
        self.mag_data = MagneticField()
        self.imu_data = Imu()
        self.imu_fused = Imu()

        self.mag_data_flag = False
        
        # Initialize Madgwick filter
        self.madgwick = Madgwick()
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Initialize quaternion [w, x, y, z]
        
        
    def imu_callback(self, msg):          
        
        # Only run the imu callback if we have received magnetometer data at least once
        if not self.mag_data_flag:
            self.get_logger().warn("Magnetometer data not yet received.")
            return
        
        # Store the latest IMU data
        self.imu_data = msg
                    
        # Extract gyroscope (rad/s)
        gyr = np.array([
            self.imu_data.angular_velocity.x,
            self.imu_data.angular_velocity.y,
            self.imu_data.angular_velocity.z
        ])
        
        # Extract accelerometer (m/s^2)
        acc = np.array([
            self.imu_data.linear_acceleration.x,
            self.imu_data.linear_acceleration.y,
            self.imu_data.linear_acceleration.z
        ])
        
        # Extract magnetometer (Tesla)
        mag = np.array([
            self.mag_data.magnetic_field.x,
            self.mag_data.magnetic_field.y,
            self.mag_data.magnetic_field.z
        ])
        
        # Update quaternion using Madgwick filter pip package
        self.quaternion = self.madgwick.updateMARG(self.quaternion, gyr=gyr, acc=acc, mag=mag)
        
        # Populate fused IMU message
        self.imu_fused.header = self.imu_data.header
        self.imu_fused.angular_velocity = self.imu_data.angular_velocity
        self.imu_fused.linear_acceleration = self.imu_data.linear_acceleration
        
        # Set orientation (Madgwick returns [w, x, y, z])
        self.imu_fused.orientation.w = float(self.quaternion[0])
        self.imu_fused.orientation.x = float(self.quaternion[1])
        self.imu_fused.orientation.y = float(self.quaternion[2])
        self.imu_fused.orientation.z = float(self.quaternion[3])
        
        self.imu_fused.orientation_covariance = self.mag_data.magnetic_field_covariance
        
        # Publish fused IMU data
        self.fused_pub.publish(self.imu_fused)
                
    def mag_callback(self, msg):
        # Store the latest magnetometer data
        self.mag_data = msg
        # Set flag to indicate that magnetometer data has been received
        self.mag_data_flag = True
        
    def spin(self):
        try:
            while rclpy.ok():
                rclpy.spin(self)
        # Handle interrupts gracefully
        except (np.select.error, OSError, KeyboardInterrupt):
            pass
        
        

def main(args=None):
    
    # Initialize the ROS2 node
    rclpy.init(args=args)
    fusion = IMUMagFusionNode()
    fusion.spin()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()