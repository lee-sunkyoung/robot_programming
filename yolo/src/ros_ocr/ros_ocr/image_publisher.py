import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.br = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # 웹캠에서 이미지를 캡처
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()

        if ret:
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
