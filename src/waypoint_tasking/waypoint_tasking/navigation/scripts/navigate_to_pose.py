import time

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main():
    rclpy.init()

    navigator = BasicNavigator()

    navigator.waitUntilNav2Active()

    # Set the goal
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position = Point(x=-1.0, y=0.0, z=0.0)
    pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    navigator.goToPose(pose)

    # Monitor the progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.info(
            f"Estimated remaining time: {Duration.from_msg(feedback.estimated_time_remaining)}"
        )
        navigator.info(
            f"Estimated remaining distance: {feedback.distance_remaining:.2}"
        )

        time.sleep(1)

    # Check the result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.info("Task succeeded")
    elif result == TaskResult.FAILED:
        navigator.error("Task failed")
    elif result == TaskResult.CANCELED:
        navigator.info("Task cancelled")


if __name__ == "__main__":
    main()
