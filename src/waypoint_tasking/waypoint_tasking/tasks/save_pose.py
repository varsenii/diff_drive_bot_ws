from waypoint_tasking.tasks.utils import TFManager, PoseManager

class SavePose:
    def __init__(self, logger, db_path:str,  buffer):
       self.logger = logger
       self.tf_manager = TFManager(buffer=buffer)
       self.pose_manager = PoseManager(db_path=db_path)

    def execute(self, command: dict):
        self.logger.info("Executing SavePose task..")

        label = command.get('location')

        if not label:
            raise ValueError("Location label is required to save pose.")
        
        # Get the current pose of the robot
        current_pose = self.tf_manager.get_current_pose_with_yaw()

        # Save the pose with the given label
        self.pose_manager.save_pose(label, current_pose)

        self.logger.info(f"Pose saved for location '{label}': {current_pose}")

