import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import LaserScan
from rclpy.logging import LoggingSeverity

import numpy as np


class ScanFusionNode(Node):
    def __init__(self):
        super().__init__('scan_fusion_node')

        self.depth_camera_sub = Subscriber(self, LaserScan, '/camera/scan')
        self.lidar_sub = Subscriber(self, LaserScan, '/scan')

        self.fused_scan_pub = self.create_publisher(LaserScan, '/fused_scan', 5)

        self.sync = ApproximateTimeSynchronizer(
            [self.lidar_sub, self.depth_camera_sub],
            queue_size=10,
            slop=0.1
        )

        self.sync.registerCallback(self.merge_scans_callback)

        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.INFO)
    
    def merge_scans_callback(self, lidar_scan, depth_camera_scan):
        self.logger.debug('Synchronized lidar and depth camera scans received')

        fused_ranges = np.array(lidar_scan.ranges)

        lidar_min_angle = lidar_scan.angle_min
        lidar_max_angle = lidar_scan.angle_max
        lidar_angle_increment = lidar_scan.angle_increment

        depth_min_angle = depth_camera_scan.angle_min
        depth_max_angle = depth_camera_scan.angle_max
        depth_angle_increment = depth_camera_scan.angle_increment

        # Determine the start and end indices in the LIDAR data corresponding to the depth camera's FOV
        depth_min_angle_idx = int((depth_min_angle - lidar_min_angle) / lidar_angle_increment)
        depth_max_angle_idx = int((depth_max_angle - lidar_max_angle) / lidar_angle_increment) 
        self.logger.debug(f'Index of depth min angle in LIDAR range array: {depth_min_angle_idx}')
        self.logger.debug(f'Index of depth max angle in LIDAR range array: {depth_max_angle_idx}')

        depth_scan_ranges = np.array(depth_camera_scan.ranges)

        # Resize the depth camera ranges to match the LIDAR resolution if needed
        if lidar_angle_increment != depth_angle_increment:
            num_depth_ranges = int((depth_max_angle - depth_min_angle) / lidar_angle_increment) + 2
            depth_scan_ranges = np.interp(
                np.linspace(depth_min_angle, depth_max_angle, num=num_depth_ranges ),
                np.linspace(depth_min_angle, depth_max_angle, len(depth_camera_scan.ranges)),
                depth_scan_ranges
            )

            self.logger.debug(f'Number of data point to interpolate: {len(fused_ranges[depth_min_angle_idx:depth_max_angle_idx])}')
            self.logger.debug(f'Linearly interpolated {num_depth_ranges}/{len(depth_camera_scan.ranges)} depth scan range data points')

        # Fuse the LIDAR and depth camera data in the overlapping region
        fused_ranges[depth_min_angle_idx:depth_max_angle_idx] = np.minimum(fused_ranges[depth_min_angle_idx:depth_max_angle_idx], depth_scan_ranges)

        # Create a new LaserScan message for the fused data
        fused_scan = LaserScan()
        fused_scan.header.stamp = lidar_scan.header.stamp
        fused_scan.header.frame_id = lidar_scan.header.frame_id
        fused_scan.angle_min = lidar_scan.angle_min
        fused_scan.angle_max = lidar_scan.angle_max
        fused_scan.angle_increment = lidar_scan.angle_increment
        fused_scan.time_increment = lidar_scan.time_increment
        fused_scan.scan_time = lidar_scan.scan_time
        fused_scan.range_min = lidar_scan.range_min
        fused_scan.range_max = lidar_scan.range_max
        fused_scan.ranges = fused_ranges.tolist()

        self.fused_scan_pub.publish(fused_scan)

def main():
    rclpy.init()

    scan_fusion_node = ScanFusionNode()
    rclpy.spin(scan_fusion_node)

    scan_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()