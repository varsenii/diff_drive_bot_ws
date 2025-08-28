from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
import math
import statistics as st


class ObstacleDetector:
    def __init__(self, node: Node):
        self.node = node
        self.logger = self.node.get_logger()

        self.last_scan = None

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.node.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos
        )

    def has_obstacles_linear(self, longitudinal_dist, traversal_dist):
        try:
            self._check_scan_availability()

            # Compute the arch angle
            ratio = traversal_dist / longitudinal_dist
            angle = int(math.degrees(math.acos(ratio)))
            self.logger.debug(f"Arch angle: {angle}")

            # Determine the scan indexes corresponding to the arch
            # TODO: take into account the angle increment when determining indexes
            # Check whether the arch range measurements are shorter then safe distance

            direction_angle = 180
            idx_min = direction_angle - (angle // 2)
            idx_max = direction_angle + (angle // 2)
            self.logger.debug(f"Range min index: {idx_min}")
            self.logger.debug(f"Range max index: {idx_max}")
            check_measurements = (
                self.last_scan.ranges[idx_min:direction_angle]
                + self.last_scan.ranges[direction_angle:idx_max]
            )

            # self.logger.info(f"Range index 0: {self.last_scan.ranges[0]}")
            # self.logger.info(f"Range index 90: {self.last_scan.ranges[90]}")
            # self.logger.info(f"Range index 180: {self.last_scan.ranges[180]}")
            # self.logger.info(f"Range index 270: {self.last_scan.ranges[270]}")

            # check_measurements = [m for m in check_measurements if m != math.inf]

            for range_measurement in check_measurements:
                if range_measurement < longitudinal_dist:
                    self.logger.warning("Obstacle detected!")
                    return True
            self.logger.info("No obstacles detected")
            return False
        except Exception as e:
            self.logger.error(f"Failed while detecting obstacles: {e}")

    def has_obstacles_rotational(self):
        self._check_scan_availability()

    def _scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def _check_scan_availability(self):
        if not self.last_scan:
            self.logger.error("No scan has been received.")
            raise Exception("No scan has been received.")
