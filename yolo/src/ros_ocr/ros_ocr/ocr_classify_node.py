import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import easyocr
from cv_bridge import CvBridge

class OCRClassificationNode(Node):
    def __init__(self):
        super().__init__('ocr_classification_node')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'classification_result', 10)
        self.br = CvBridge()
        self.reader = easyocr.Reader(['en'])  # 언어 설정

        # Define the labels
        self.labels = ['paper', 'can', 'plastic', 'vinyl', 'other']

    def listener_callback(self, msg):
        self.get_logger().info('Received image')
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(msg)

        # Perform OCR
        results = self.reader.readtext(cv_image)

        # Initialize probabilities for each category
        probabilities = {label: 0 for label in self.labels}

        # Analyze OCR results and compute probabilities
        for res in results:
            text = res[1].lower()
            for label in self.labels:
                if label in text:
                    probabilities[label] += 1

        # Normalize probabilities
        total = sum(probabilities.values())
        if total > 0:
            for label in self.labels:
                probabilities[label] /= total

        # Get the top prediction
        top_label = max(probabilities, key=probabilities.get)
        top_prob = probabilities[top_label]

        # Publish the result
        result_str = f'{top_label}: {top_prob:.4f}'
        self.get_logger().info(f'Classification Result: {result_str}')
        text_msg = String()
        text_msg.data = result_str
        self.publisher_.publish(text_msg)

def main(args=None):
    rclpy.init(args=args)
    ocr_classification_node = OCRClassificationNode()
    rclpy.spin(ocr_classification_node)
    ocr_classification_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
