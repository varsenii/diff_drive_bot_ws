import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy.time
from sensor_msgs.msg import LaserScan
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter

from sensor_fusion.utils.scan_fusor import ScanFusor


class ScanFusionNode(Node):
    def __init__(self):
        super().__init__('scan_fusion_node')

        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.INFO)

        self.declare_parameter('depth_frame', 'camera_link_optical')
        self.depth_frame = self.get_parameter('depth_frame').get_parameter_value().string_value
        use_sim_time = self.get_parameter('use_sim_time').value
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
        self.logger.info(f'use_sim_time: {use_sim_time}')

        self.depth_camera_sub = Subscriber(self, LaserScan, '/camera/scan')
        self.lidar_sub = Subscriber(self, LaserScan, '/scan')

        self.fused_scan_pub = self.create_publisher(LaserScan, '/scan_fused', 5)

        self.sync = ApproximateTimeSynchronizer(
            [self.lidar_sub, self.depth_camera_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.fuse_scans_callback)

        self.scan_fusor = ScanFusor(self)

    
    def fuse_scans_callback(self, lidar_scan, depth_camera_scan):
        fused_scan = self.scan_fusor.fuse_scans(lidar_scan, depth_camera_scan)
        self.fused_scan_pub.publish(fused_scan)
    

def main():
    rclpy.init()

    scan_fusion_node = ScanFusionNode()
    rclpy.spin(scan_fusion_node)

    scan_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()