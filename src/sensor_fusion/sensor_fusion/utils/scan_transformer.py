import numpy as np
from geometry_msgs.msg import PointStamped, Point
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener


class ScanTransformer:
    def __init__(self, node):
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.depth_camera_lidar_tf = None

    def transform_scan(self, scan, target_frame):
        self.lookup_depth_camera_to_lidar_tf(target_frame, scan.header.frame_id)

        ranges_transformed = []

        for i, r in enumerate(scan.ranges):
            if not np.isfinite(r):
                ranges_transformed.append(r)
                continue
            
            # Convert the polar into cartesian coordintates
            angle = scan.angle_min + i * scan.angle_increment
            point = self.build_point_from_polar_coordinates(radius=r, angle=angle)

            # Transform the point into the Lidar's frame
            transformed_point = self.transform_point(point, source_frame=scan.header.frame_id)

            if transformed_point is None:
                ranges_transformed.append(np.nan)
            else:
                # Re-convert the point into polar coordinates
                range = np.hypot(transformed_point.x, transformed_point.y)
                ranges_transformed.append(range)
        
        scan.ranges = ranges_transformed
        return scan

    def resize_scan(self, scan, angle_increment):
        scan_size = int((scan.angle_max - scan.angle_min) / angle_increment) + 1

        scan.ranges = np.interp(
            np.linspace(scan.angle_min, scan.angle_max, num=scan_size),
            np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges)),
            np.array(scan.ranges)
        ).tolist()

        return scan
    
    def transform_point(self, point: Point, source_frame: str) -> Point:
        if self.depth_camera_lidar_tf:
            point_stamped = PointStamped()
            point_stamped.header.stamp = self.node.get_clock().now().to_msg()
            point_stamped.header.frame_id = source_frame
            point_stamped.point = point
            try:
                transformed_point = do_transform_point(point_stamped, self.depth_camera_lidar_tf)
                return transformed_point.point
            except Exception as e:
                print(f'Failed to transform point: {e}')
        return None
    
    def build_point_from_polar_coordinates(self, radius, angle) -> Point:
        point = Point()
        point.x = radius * np.cos(angle)
        point.y = radius * np.sin(angle)
        point.z = 0.0
        return point
    
    def lookup_depth_camera_to_lidar_tf(self, target_frame, source_frame):
        if self.depth_camera_lidar_tf is not None:
            return

        try:
            self.depth_camera_lidar_tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, self.node.get_clock().now()
            )
            return True
        except Exception as e:
            self.node.logger.error(f'Failed to lookup transform: {e}')
            return False