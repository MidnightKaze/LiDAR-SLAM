#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import numpy as np
import struct

class LivoxToPointCloud2(Node):
    def __init__(self):
        super().__init__('livox_to_pointcloud2')
        
        # Parameters
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/pointcloud2')
        self.declare_parameter('frame_id', 'livox_frame')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Subscriber to Livox CustomMsg
        self.subscription = self.create_subscription(
            CustomMsg,
            input_topic,
            self.livox_callback,
            10
        )
        
        # Publisher for PointCloud2
        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )
        
        self.get_logger().info(f'Livox to PointCloud2 converter started')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')

    def livox_callback(self, msg):
        """Convert Livox CustomMsg to PointCloud2"""
        try:
            # Create PointCloud2 message
            pc2_msg = PointCloud2()
            pc2_msg.header.stamp = msg.header.stamp
            pc2_msg.header.frame_id = self.frame_id
            
            # Define point cloud fields
            pc2_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
                PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
            ]
            
            # Point size in bytes
            point_step = 18  # 4*4 + 1 + 1 bytes per point
            pc2_msg.point_step = point_step
            pc2_msg.width = len(msg.points)
            pc2_msg.height = 1  # Unorganized point cloud
            pc2_msg.row_step = pc2_msg.width * point_step
            pc2_msg.is_dense = False
            
            # Convert points to bytes
            data = []
            for point in msg.points:
                # Pack x, y, z as float32
                data.extend(struct.pack('f', point.x))
                data.extend(struct.pack('f', point.y))
                data.extend(struct.pack('f', point.z))
                # Pack intensity as float32 (convert from uint8)
                data.extend(struct.pack('f', float(point.reflectivity)))
                # Pack tag and line as uint8
                data.extend(struct.pack('B', point.tag))
                data.extend(struct.pack('B', point.line))
            
            pc2_msg.data = bytes(data)
            
            # Publish the converted message
            self.publisher.publish(pc2_msg)
            
            self.get_logger().debug(f'Converted {len(msg.points)} points from Livox to PointCloud2')
            
        except Exception as e:
            self.get_logger().error(f'Error converting Livox message: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LivoxToPointCloud2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()