#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class
import statistics

class TopicLatencyChecker(Node):
    def __init__(self):
        super().__init__('topic_latency_checker')

        # Parameters
        self.declare_parameter('topic', '/scan')
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('verbose', True)

        self.topic_name = self.get_parameter('topic').get_parameter_value().string_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        # Wait until clock starts if using sim time
        if self.get_parameter('use_sim_time').get_parameter_value().bool_value:
            while self.get_clock().now().nanoseconds == 0:
                self.get_logger().info("Waiting for /clock to start...")
                rclpy.spin_once(self, timeout_sec=0.1)

        # Determine message type
        msg_type = get_msg_class(self, self.topic_name)
        if msg_type is None:
            self.get_logger().error(f"Cannot determine message type for topic {self.topic_name}")
            return

        self.get_logger().info(f"Subscribing to {self.topic_name} [{msg_type.__name__}], running for {self.duration}s")

        # Subscribe
        self.sub = self.create_subscription(msg_type, self.topic_name, self.callback, 10)

        # Store latency measurements
        self.latencies_ms = []

        # Record start time
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # Repeating timer to check elapsed time
        self.create_timer(0.05, self.check_duration)  # 50ms

    def callback(self, msg):
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_time = self.get_clock().now().nanoseconds * 1e-9
        latency_ms = (now_time - msg_time) * 1000
        self.latencies_ms.append(latency_ms)

        if self.verbose:
            self.get_logger().info(f"Message latency: {latency_ms:.3f} ms")

    def check_duration(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time >= self.duration:
            self.report_statistics()
            rclpy.shutdown()

    def report_statistics(self):
        if not self.latencies_ms:
            self.get_logger().warn("No messages received.")
            return

        mean = statistics.mean(self.latencies_ms)
        max_latency = max(self.latencies_ms)
        min_latency = min(self.latencies_ms)
        stdev = statistics.stdev(self.latencies_ms) if len(self.latencies_ms) > 1 else 0.0

        self.get_logger().info("=== Message Latency Statistics ===")
        self.get_logger().info(f"Samples collected: {len(self.latencies_ms)}")
        self.get_logger().info(f"Mean latency: {mean:.3f} ms")
        self.get_logger().info(f"Max latency: {max_latency:.3f} ms")
        self.get_logger().info(f"Min latency: {min_latency:.3f} ms")
        self.get_logger().info(f"Standard deviation: {stdev:.3f} ms")
        self.get_logger().info("=================================")

def main():
    rclpy.init()
    node = TopicLatencyChecker()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
