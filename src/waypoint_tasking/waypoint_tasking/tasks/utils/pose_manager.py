import yaml

class PoseManager:
    def __init__(self, db_path: str):
        self.db_path = db_path
        self.poses = self._read_poses()
    
    def save_pose(self, label: str, pose: dict):
        self.poses.append({'label': label, 'pose': pose})
        self._write_poses(self.poses)
    
    def get_pose(self, label: str) -> dict:
        for entry in self.poses:
            print('Looking for:', label, 'Current entry:', entry['label'])
            if entry['label'] == label:
                return entry
        raise ValueError(f"Pose with label '{label}' not found.")

    def _read_poses(self):
        try:
            with open(self.db_path, 'r') as file:
                return yaml.safe_load(file) or []
        except FileNotFoundError:
            return []
    
    def _write_poses(self, poses):
        with open(self.db_path, 'w') as file:
            yaml.safe_dump(poses, file) 