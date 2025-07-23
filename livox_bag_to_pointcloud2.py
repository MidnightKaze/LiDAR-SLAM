#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import numpy as np
import struct
import sys
import os

class LivoxBagConverter(Node):
    def __init__(self):
        super().__init__('livox_bag_converter')
        
        # Parameters
        self.declare_parameter('input_bag_path', '')
        self.declare_parameter('output_bag_path', '')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/pointcloud2')
        self.declare_parameter('frame_id', 'livox_frame')
        
        self.input_bag_path = self.get_parameter('input_bag_path').value
        self.output_bag_path = self.get_parameter('output_bag_path').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        if not self.input_bag_path:
            self.get_logger().error('No input bag path provided! Use: --ros-args -p input_bag_path:=/path/to/input.db3')
            sys.exit(1)
            
        if not self.output_bag_path:
            # Auto-generate output path
            base_name = os.path.splitext(os.path.basename(self.input_bag_path))[0]
            output_dir = os.path.dirname(self.input_bag_path)
            self.output_bag_path = os.path.join(output_dir, f"{base_name}_pointcloud2")
        
        self.get_logger().info(f'Input bag: {self.input_bag_path}')
        self.get_logger().info(f'Output bag: {self.output_bag_path}')
        self.get_logger().info(f'Converting {self.input_topic} -> {self.output_topic}')
        
        # Process the bag
        self.convert_bag()
    
    def convert_bag(self):
        """Convert Livox CustomMsg to PointCloud2 and write to new bag"""
        try:
            # Setup input reader
            reader = rosbag2_py.SequentialReader()
            input_storage_options = rosbag2_py.StorageOptions(
                uri=self.input_bag_path, 
                storage_id='sqlite3'
            )
            input_converter_options = rosbag2_py.ConverterOptions('', '')
            reader.open(input_storage_options, input_converter_options)
            
            # Show available topics
            topic_types = reader.get_all_topics_and_types()
            self.get_logger().info('Available topics in input bag:')
            for topic_metadata in topic_types:
                self.get_logger().info(f'  {topic_metadata.name} ({topic_metadata.type})')
            
            # Setup output writer
            writer = rosbag2_py.SequentialWriter()
            output_storage_options = rosbag2_py.StorageOptions(
                uri=self.output_bag_path,
                storage_id='sqlite3'
            )
            output_converter_options = rosbag2_py.ConverterOptions('', '')
            writer.open(output_storage_options, output_converter_options)
            
            # Create topic info for PointCloud2
            pc2_topic_info = rosbag2_py.TopicMetadata(
                name=self.output_topic,
                type='sensor_msgs/msg/PointCloud2',
                serialization_format='cdr'
            )
            writer.create_topic(pc2_topic_info)
            
            # Copy other topics (optional - you can comment this out if you only want PointCloud2)
            for topic_metadata in topic_types:
                if topic_metadata.name != self.input_topic:
                    writer.create_topic(topic_metadata)
            
            # Process messages
            message_count = 0
            converted_count = 0
            
            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()
                message_count += 1
                
                if topic == self.input_topic:
                    try:
                        # Deserialize CustomMsg
                        msg_type = get_message('livox_ros_driver2/msg/CustomMsg')
                        custom_msg = deserialize_message(data, msg_type)
                        
                        # Convert to PointCloud2
                        pc2_msg = self.convert_livox_to_pointcloud2(custom_msg, timestamp)
                        
                        # Serialize and write PointCloud2
                        serialized_data = serialize_message(pc2_msg)
                        writer.write(self.output_topic, serialized_data, timestamp)
                        
                        converted_count += 1
                        
                        if converted_count % 50 == 0:
                            self.get_logger().info(f'Converted {converted_count} messages...')
                            
                    except Exception as e:
                        self.get_logger().error(f'Error converting message: {e}')
                
                else:
                    # Copy other messages as-is (optional)
                    writer.write(topic, data, timestamp)
            
            self.get_logger().info(f'Conversion complete!')
            self.get_logger().info(f'Total messages processed: {message_count}')
            self.get_logger().info(f'CustomMsg messages converted: {converted_count}')
            self.get_logger().info(f'Output bag saved to: {self.output_bag_path}')
            
        except Exception as e:
            self.get_logger().error(f'Error during conversion: {e}')
            raise
    
    def convert_livox_to_pointcloud2(self, msg, timestamp_ns):
        """Convert Livox CustomMsg to PointCloud2"""
        # Create PointCloud2 message
        pc2_msg = PointCloud2()
        
        # Set header with original timestamp
        pc2_msg.header.stamp.sec = int(timestamp_ns // 1000000000)
        pc2_msg.header.stamp.nanosec = int(timestamp_ns % 1000000000)
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
        return pc2_msg

def main(args=None):
    rclpy.init(args=args)
    
    try:
        converter = LivoxBagConverter()
        # Don't spin - just do the conversion and exit
        converter.destroy_node()
        print("Conversion completed successfully!")
        
    except KeyboardInterrupt:
        print('\nConversion interrupted...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
