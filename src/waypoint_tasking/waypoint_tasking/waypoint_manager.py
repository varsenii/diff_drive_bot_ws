import rclpy
from visualization_msgs.msg import MarkerArray
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformListener, Buffer
from visualization_msgs.msg import Marker
import os
import json

from diff_drive_bot_interfaces.srv import SaveWaypoints

from waypoint_tasking.utils import transform_to_goal, serialize_markers, deserialize_markers


class WaypointManager:

    def __init__(self, node):
        self.node = node
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscription to MarkerArray with a callback group for concurrency
        self.waypoint_subscription = self.node.create_subscription(
            MarkerArray,
            '/waypoints',
            self.waypoint_callback,
            10,
            callback_group=self.callback_group
        )

        self.save_waypoints_service = self.node.create_service(SaveWaypoints, '/save_waypoints', self.save_waypoints_callback)
        
        self.logger = self.node.get_logger()
        self.waypoints = []
        self.waypoint_dir = os.path.join(os.path.curdir, 'src', 'waypoint_tasking', 'waypoint_tasking', 'waypoints')

        # The pose before the waypoint navigation
        # May different from the initial one when the map is loaded
        self.starting_pose_goal = None  
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self.node)

    def waypoint_callback(self, msg):
        self.waypoints = [marker for i, marker in enumerate(msg.markers) if i % 3 == 0]
        self.logger.info(f'Received {len(self.waypoints)} waypoints')
    
    def save_waypoints_callback(self, request, response):
        self.logger.info(f'Saving the waypoints for the map {request.map}...')

        path = os.path.join(self.waypoint_dir, f'{request.map}.json')
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)

            with open(path, 'w') as waypoint_file:
                waypoints_data = serialize_markers(self.waypoints)
                json.dump(waypoints_data, waypoint_file, indent=4)
            
            self.logger.info('Waypoints saved successfully')
            response.success = True
        except Exception as e:
            self.logger.error(f'Failed to save the waypoints: {e}')
            response.success = False

        return response

    def load_waypoints_for_map(self, map):
        if map == '':
            self.logger.info('Map isn\'t specified')
            return
        
        self.logger.debug(f'Loading waypoints for the map {map}...')
        
        path = os.path.join(self.waypoint_dir, f'{map}.json')
        try:
            with open(path, 'r') as waypoint_file:
                waypoints_data = json.load(waypoint_file)
                self.waypoints = deserialize_markers(waypoints_data)
                
                self.logger.info(f'Waypoints loaded successfully for the map {map}')
        except Exception as e:
            self.logger.error(f'Failed to load the waypoints: {e}')


    def get_waypoints(self, indexes):
        if len(indexes) == 0:
            return self.waypoints
        return [self.waypoints[i] for i in indexes]
    
    def update_starting_pose(self):
        try:
            now = rclpy.time.Time()
            transform = self.transform_buffer.lookup_transform('map', 'base_link', now)
            self.starting_pose_goal = transform_to_goal(transform)
        except Exception as e:
            self.logger.error(f'Failed to update the starting pose: {e}')