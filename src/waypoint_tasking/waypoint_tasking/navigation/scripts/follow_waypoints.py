import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main():
    rclpy.init()

    route = [
        (-3.0, 0.0),
        (-3.0, -2.0),
        (0.0, -2.0),
    ]

    navigator = BasicNavigator()

    navigator.waitUntilNav2Active()

    # Set the route to navigate
    route_poses = []

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    for position in route:
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position = Point(x=position[0], y=position[1], z=0.0)
        route_poses.append(deepcopy(pose))

    navigator.followWaypoints(route_poses)

    # Monitor the progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.info(
            "Following waypoint {}/{}".format(
                feedback.current_waypoint + 1, len(route_poses)
            )
        )

        time.sleep(5)

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
