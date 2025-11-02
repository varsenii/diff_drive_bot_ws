#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ros2topic.api import get_msg_class
import statistics
import time

class TopicTSChecker(Node):
    def __init__(self):
        super().__init__('topic_ts_checker')

        # Declare parameters
        self.declare_parameter('topic1', '/diff_cont/odom')
        self.declare_parameter('topic2', '/scan')
        self.declare_parameter('slop', 0.05)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('duration', 10.0)  # seconds

        topic1 = self.get_parameter('topic1').get_parameter_value().string_value
        topic2 = self.get_parameter('topic2').get_parameter_value().string_value
        slop = self.get_parameter('slop').get_parameter_value().double_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value

        # Get message types
        msg1_type = get_msg_class(self, topic1)
        msg2_type = get_msg_class(self, topic2)

        if msg1_type is None or msg2_type is None:
            self.get_logger().error("Cannot determine message type for one of the topics.")
            return

        self.get_logger().info(f"Subscribing to {topic1} [{msg1_type.__name__}] and {topic2} [{msg2_type.__name__}]")
        self.get_logger().info(f"Using slop={slop}s, queue_size={queue_size}, duration={self.duration}s")

        # Create subscribers
        sub1 = Subscriber(self, msg1_type, topic1)
        sub2 = Subscriber(self, msg2_type, topic2)

        # ApproximateTimeSynchronizer
        ats = ApproximateTimeSynchronizer([sub1, sub2], queue_size=queue_size, slop=slop)
        ats.registerCallback(self.callback)

        # Store timestamp differences
        self.deltas = []

        # Start timer to end after duration
        self.start_time = time.time()
        self.timer = self.create_timer(self.duration, self.report_statistics)

    def callback(self, msg1, msg2):
        t1 = msg1.header.stamp.sec + msg1.header.stamp.nanosec * 1e-9
        t2 = msg2.header.stamp.sec + msg2.header.stamp.nanosec * 1e-9
        delta_ms = (t1 - t2) * 1000
        self.deltas.append(delta_ms)
        self.get_logger().info(f"Timestamps: t1={t1:.6f}, t2={t2:.6f}, Δ={delta_ms:.3f} ms")

    def report_statistics(self):
        if not self.deltas:
            self.get_logger().warn("No timestamp differences recorded.")
        else:
            mean = statistics.mean(self.deltas)
            max_delta = max(self.deltas)
            min_delta = min(self.deltas)
            stdev = statistics.stdev(self.deltas) if len(self.deltas) > 1 else 0.0
            self.get_logger().info("=== Timestamp Δ Statistics ===")
            self.get_logger().info(f"Samples collected: {len(self.deltas)}")
            self.get_logger().info(f"Mean Δ: {mean:.3f} ms")
            self.get_logger().info(f"Max Δ: {max_delta:.3f} ms")
            self.get_logger().info(f"Min Δ: {min_delta:.3f} ms")
            self.get_logger().info(f"Standard deviation: {stdev:.3f} ms")
            self.get_logger().info("==============================")

        # Shutdown node after reporting
        rclpy.shutdown()

def main():
    rclpy.init()
    node = TopicTSChecker()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()