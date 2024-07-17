import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from diff_drive_bot_interfaces.srv import FaceIdentification, RememberFace

import pickle
import os
from deepface import DeepFace


class FaceIdentificator(Node):

    def __init__(self):
        super().__init__('face_recognition')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.srv = self.create_service(FaceIdentification, 'identify_person_by_face', self.identify_person_by_face_callback)
        self.remember_srv = self.create_service(RememberFace, 'remember_face', self.remember_face_callback)
        
        self.cv_bridge = CvBridge()
        self.last_image = None
        self.embedding_file = os.path.join('.', 'src', 'face_recognition', 'data', 'face_embeddings.pkl')
        self.known_faces = {}

        self.model_name = 'Facenet512'
        self.detector_backend = 'retinaface'
        self.distance_metric = 'euclidean_l2'
        self.enforce_detection = False
        self.threshold = None
        self.anti_spoofing = False

        if os.path.exists(self.embedding_file):
            self.known_faces = self.load_embeddings()
        else:
            self.save_embeddings(self.known_faces)

    
    def identify_person_by_face_callback(self, request, response):
        person_name = self.get_match_person_name()

        if person_name is None:
            response.success = False
        else:
            response.success = True
            response.name = person_name
        return response
    
    def remember_face_callback(self, request, response):
        if self.last_image is None:
            self.get_logger().error('No image to process for face embedding')
            response.success = False
            return response
        
        try:
            embedding = DeepFace.represent(
                img_path=self.last_image, model_name=self.model_name, detector_backend=self.detector_backend,
                enforce_detection=self.enforce_detection)[0]['embedding']
            self.get_logger().info(f'Computed the embedding for {request.name}')
            
            if os.path.exists(self.embedding_file):
                self.known_faces = self.load_embeddings()
            else:
                self.known_faces = {}
            
            self.known_faces[request.name] = embedding
            self.save_embeddings(self.known_faces)
            
            response.success = True
        except Exception as e:
            self.get_logger().error(f'Failed to compute and save embedding: {e}')
            response.success = False
        
        return response

    def image_callback(self, msg):        
        self.last_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def get_match_person_name(self):
        if self.last_image is None:
            self.get_logger().error('No image to proceed with face recognition')
            return None

        for person_name, embedding in self.known_faces.items():
            verification = DeepFace.verify(
                img1_path = self.last_image, img2_path = embedding, model_name=self.model_name, 
                detector_backend=self.detector_backend, distance_metric=self.distance_metric,
                enforce_detection=self.enforce_detection,
                threshold=self.threshold, anti_spoofing=self.anti_spoofing)
            self.get_logger().info(f'Verififying if it is {person_name}')
            self.get_logger().info(f'Verification outcome: {verification}')

            if verification['verified'] is True:
                return person_name
        
        return None

    def save_embeddings(self, embeddings):
        with open(self.embedding_file, 'wb') as f:
            pickle.dump(embeddings, f)

    def load_embeddings(self):
        with open(self.embedding_file, 'rb') as f:
            embeddings = pickle.load(f)
        return embeddings
    

def main(args=None):
    rclpy.init(args=args)

    face_recognition = FaceIdentificator()
    rclpy.spin(face_recognition)
    
    face_recognition.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()