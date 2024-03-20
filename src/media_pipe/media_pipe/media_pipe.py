import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Float64
from .mediapipe_controller import MP_Controller
from sensor_msgs.msg import Image, CompressedImage

class media_pipe_node(Node):
    def __init__(self):
        super().__init__('media_pipe_node')
        self.bridge = CvBridge()
        self.head_x = -1
        self.head_y = -1
        self.GAME_MODE = 2  # to initialize face recognition
        self.frame_sub = self.create_subscription(CompressedImage, 'camera/image/compressed', self.image_callback, 10)
        self.pub_player_head_pos = self.create_publisher(Float64, 'y_position', 10)
        self.test_landmarker = MP_Controller(self.GAME_MODE)
        self.get_logger().info('MediaPipe Node has been started')

    def image_callback(self, msg):
        try:
            self.get_logger().info('Starting to Process the image frames')
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: {}'.format(e))
            return
        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape
        self.get_logger().info(f'Frame info: Height: {height} width: {width}')
        self.test_landmarker.detect_async(frame, self.GAME_MODE)

        try:
            self.get_logger().info('Getting Face Coordinates')
            self.head_x, self.head_y = self.test_landmarker.get_face_coordinates()
        except Exception as e:
            self.get_logger().error('Failed to get face coordinates: {}'.format(e))
            self.head_x, self.head_y = -1, -1

        if self.head_x is None or self.head_y is None:
           raise ValueError("Head Coordinates are None")
        
        if self.head_x != -1 and self.head_y != -1:
            y_pos = Float64()
            y_pos.data = self.head_y
            self.pub_player_head_pos.publish(y_pos)
            self.get_logger().info(f'Head Position: x: {self.head_x} y: {self.head_y}')
        else:
            self.get_logger().info('No face detected')

def main(args=None):
    rclpy.init(args=args)
    media_pipe = media_pipe_node()
    rclpy.spin(media_pipe)
    media_pipe.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
