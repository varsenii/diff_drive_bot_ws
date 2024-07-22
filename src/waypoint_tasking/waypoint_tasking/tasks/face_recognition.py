from diff_drive_bot_interfaces.srv import FaceIdentification


class FaceRecognitionManager:

    def __init__(self, node):
        self.node = node
        self.face_recognition_client = self.node.create_client(FaceIdentification, 'identify_person_by_face')
        self.logger = self.node.get_logger()

    def perform_task(self):
        request = FaceIdentification.Request()
        
        future = self.face_recognition_client.call_async(request)
        future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result()
            self.logger.info(f'Face recognition result: {result}')
        except Exception as e:
            self.logger.error(f'Exception occurred during face recognition: {e}')

        # Notify the parent tasker to move to the next waypoint
        self.parent_tasker.on_task_complete(task='face_recognition', result=result)

    def set_parent_tasker(self, tasker):
        self.parent_tasker = tasker