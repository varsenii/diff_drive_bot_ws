import numpy as np
from sensor_msgs.msg import LaserScan

from sensor_fusion.utils.scan_transformer import ScanTransformer


class ScanFusor:
    def __init__(self, node):
        self.node = node
        self.scan_transformer = ScanTransformer(node)
    
    def fuse_scans(self, lidar_scan, depth_camera_scan):
        self.node.logger.debug('Synchronized lidar and depth camera scans received')

        fused_ranges = np.array(lidar_scan.ranges)

        # Transform depth camera ranges into the Lidar's frame if needed
        if lidar_scan.header.frame_id != depth_camera_scan.header.frame_id:
            depth_camera_scan = self.scan_transformer.transform_scan(depth_camera_scan, target_frame=lidar_scan.header.frame_id)

        # Resize the depth camera ranges to match the LIDAR resolution if needed
        if lidar_scan.angle_increment != depth_camera_scan.angle_increment:
            depth_camera_scan = self.scan_transformer.resize_scan(depth_camera_scan, lidar_scan.angle_increment)

        # Determine the start and end indices in the LIDAR data corresponding to the depth camera's FOV
        depth_min_angle_idx = int((depth_camera_scan.angle_min - lidar_scan.angle_min) / lidar_scan.angle_increment)
        depth_max_angle_idx = int((depth_camera_scan.angle_max - lidar_scan.angle_min) / lidar_scan.angle_increment)

        # Fuse the LIDAR and depth camera data in the overlapping region
        overlap = slice(depth_min_angle_idx, depth_max_angle_idx)
        fused_ranges[overlap] = [
            min(d, s) if np.isfinite(d) and np.abs(s - d) > 0.1 else s
            for s, d in zip(fused_ranges[overlap], depth_camera_scan.ranges)
        ]

        # Return a new LaserScan message for the fused data
        return self.build_fused_scan(fused_ranges=fused_ranges, lidar_scan=lidar_scan)
    
    def build_fused_scan(self, fused_ranges, lidar_scan):
        fused_scan = LaserScan()
        # TODO: make it work by using the node's clock
        # fused_scan.header.stamp = self.node.get_clock().now().to_msg()
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
        return fused_scan