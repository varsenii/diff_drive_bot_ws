from visualization_msgs.msg import MarkerArray
from rclpy.callback_groups import ReentrantCallbackGroup


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

    def waypoint_callback(self, msg):
        self.logger.info(f'Received {len(msg.markers)} markers')      
        self.waypoints = [marker for i, marker in enumerate(msg.markers) if i % 3 == 0]
        self.logger.info(f'Derived {len(self.waypoints)} waypoints')

    def get_waypoints(self):
        return self.waypoints