import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class LivoxToPCL(Node):
    def __init__(self):
        super().__init__('livox_to_pcl')
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/points', 10)
        self.get_logger().info("Livox → PointCloud2 converter started.")

    def callback(self, msg: CustomMsg):
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = "livox_frame"

        cloud = []
        for point in msg.points:
            cloud.append((point.x, point.y, point.z, point.reflectivity))

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pointcloud_msg = pc2.create_cloud(header, fields, cloud)
        self.publisher.publish(pointcloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LivoxToPCL()
    rclpy.spin(node)
    rclpy.shutdown()