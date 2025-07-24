#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
from datetime import datetime

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        
        # Create output directory
        self.output_dir = './saved_maps'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/kiss/local_map',
            self.pointcloud_callback,
            10
        )
        
        self.save_counter = 0
        self.get_logger().info(f"PointCloud saver started. Will save to: {self.output_dir}")

    def pointcloud_callback(self, msg):
        try:
            # Extract points from point cloud
            points = list(pc2.read_points(msg, skip_nans=True))
            
            if not points:
                self.get_logger().warn("Empty point cloud")
                return
                
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"hallway_map_{timestamp}_{self.save_counter:04d}.pcd"
            filepath = os.path.join(self.output_dir, filename)
            
            # Save as PCD format
            self.save_as_pcd(points, filepath)
            
            self.save_counter += 1
            self.get_logger().info(f"Saved {len(points)} points to {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save point cloud: {e}")

    def save_as_pcd(self, points, filename):
        """Save points as PCD file"""
        with open(filename, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            
            for point in points:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")

def main(args=None):
    rclpy.init(args=args)
    saver = PointCloudSaver()
    
    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        pass
    finally:
        saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()