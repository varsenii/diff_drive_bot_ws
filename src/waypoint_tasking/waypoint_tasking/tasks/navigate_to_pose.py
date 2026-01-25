from waypoint_tasking.clients.navigation import Navigtor
from waypoint_tasking.tasks.utils import PoseManager, TFManager

class NavigateToPose:
    def __init__(self, logger, db_path: str):
        self.logger = logger
        self.navigator = Navigtor(logger=logger)
        self.pose_manager = PoseManager(db_path=db_path)

    def execute(self, command: dict):
        location = command.get('location')
        if not location:
            raise ValueError("Location is required to navigate.")

        # Get the target pose from from databaser
        target = self.pose_manager.get_pose(label=location)

        self.logger.info(f"Navigating to location '{location}': {target['pose']}...")

        # Move the robot to the target pose
        self.navigator.go_to_pose(x =target['pose']['x'],
                                  y=target['pose']['y'],
                                  yaw=target['pose']['yaw'])