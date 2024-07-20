from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage


def marker_to_goal(marker: Marker) -> NavigateToPose.Goal:
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.stamp = marker.header.stamp
    goal_msg.pose.header.frame_id = marker.header.frame_id
    goal_msg.pose.pose.position = marker.pose.position
    goal_msg.pose.pose.orientation = marker.pose.orientation
    return goal_msg

def transform_to_goal(transform: TFMessage) -> NavigateToPose.Goal:
    pose = PoseStamped()
    pose.header.frame_id = transform.header.frame_id
    pose.header.stamp = transform.header.stamp
    
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    
    pose.pose.orientation = transform.transform.rotation
    
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = pose

    return goal_msg