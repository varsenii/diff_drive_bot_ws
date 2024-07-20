from diff_drive_bot_interfaces.srv import FaceIdentification


class FaceRecognitionManager:

    def __init__(self, node):
        self.node = node
        self.face_recognition_client = self.node.create_client(FaceIdentification, 'identify_person_by_face')
        self.logger = self.node.get_logger()

    def identify_by_face(self):
        self.logger.info('Performing face recognition...')
        request = FaceIdentification.Request()
        
        future = self.face_recognition_client.call_async(request)
        future.add_done_callback(self.face_recognition_callback)

    def face_recognition_callback(self, future):
        try:
            result = future.result()
            self.logger.info(f'Face recognition result: {result}')
            
            # Notify the parent tasker to move to the next waypoint
            self.parent_tasker.on_face_recognition_complete()
        except Exception as e:
            self.logger.error(f'Exception occurred during face recognition: {e}')

    def set_parent_tasker(self, tasker):
        self.parent_tasker = tasker