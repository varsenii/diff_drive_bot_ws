from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

import os
from datetime import datetime


class ReportManager:
    def __init__(self):
        pass

    def create_report(self):
        now = datetime.now()
        self.report_file_path = os.path.join(os.path.curdir, 'src', 'waypoint_tasking', 'waypoint_tasking', 'reports', f'{now}.txt')

        with open(self.report_file_path, 'w') as report_file:
            report_file.write('Waypoint Tasks Report\n')
            report_file.write('=====================\n\n')

    def log_task_result(self, waypoint_index, task, result):
        with open(self.report_file_path, 'a') as report_file:
            report_file.write(f'Waypoint {waypoint_index + 1}: "{task}" task result: {result}\n')


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

def serialize_markers(markers):
    return [{
            'header': {
                'stamp': {
                    'sec': marker.header.stamp.sec,
                    'nanosec': marker.header.stamp.nanosec
                },
                'frame_id': marker.header.frame_id
            },
            'id': marker.id,
            'pose': {
                'position': {
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'z': marker.pose.position.z,
                },
                'orientation': {
                    'x': marker.pose.orientation.x,
                    'y': marker.pose.orientation.y,
                    'z': marker.pose.orientation.z,
                    'w': marker.pose.orientation.w,
                },
            }
        } for marker in markers]

def deserialize_markers(waypoints_data):
    markers = []

    for data in waypoints_data:
        marker = Marker()
        marker.header = Header()
        marker.header.stamp.sec = data['header']['stamp']['sec']
        marker.header.stamp.nanosec = data['header']['stamp']['nanosec']
        marker.header.frame_id = data['header']['frame_id']
        marker.id = data['id']
        marker.pose = Pose()
        marker.pose.position.x = data['pose']['position']['x']
        marker.pose.position.y = data['pose']['position']['y']
        marker.pose.position.z = data['pose']['position']['z']
        marker.pose.orientation.x = data['pose']['orientation']['x']
        marker.pose.orientation.y = data['pose']['orientation']['y']
        marker.pose.orientation.z = data['pose']['orientation']['z']
        marker.pose.orientation.w = data['pose']['orientation']['w']
        
        markers.append(marker)
    
    return markers