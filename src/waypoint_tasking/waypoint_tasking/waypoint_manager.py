import rclpy
from visualization_msgs.msg import MarkerArray
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformListener, Buffer

from waypoint_tasking.utils import transform_to_goal


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
        
        self.logger = self.node.get_logger()
        self.waypoints = []

        # The pose before the waypoint navigation
        # May different from the initial one when the map is loaded
        self.starting_pose_goal = None  
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self.node)

    def waypoint_callback(self, msg):
        self.logger.info(f'Received {len(msg.markers)} markers')      
        self.waypoints = [marker for i, marker in enumerate(msg.markers) if i % 3 == 0]
        self.logger.info(f'Derived {len(self.waypoints)} waypoints')

        self.update_starting_pose()

    def get_waypoints(self):
        return self.waypoints
    
    def update_starting_pose(self):
        try:
            now = rclpy.time.Time()
            transform = self.transform_buffer.lookup_transform('map', 'base_link', now)
            self.starting_pose_goal = transform_to_goal(transform)
            self.logger.info(f'Staring pose is updated to: {self.starting_pose_goal.pose}')
        except Exception as e:
            self.logger.error(f'Failed to update the starting pose: {e}')