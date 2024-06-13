import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
import json

class YoloV5Node(Node):
    def __init__(self):
        super().__init__('yolov5_node')
        self.publisher_ = self.create_publisher(String, 'yolov5/labels', 10)
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.eval()
        self.cap = cv2.VideoCapture(0)  # 첫 번째 웹캠 열기

        # 1초마다 이미지 캡처 타이머 설정
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('웹캠에서 이미지를 캡처하는 데 실패했습니다.')
            return

        results = self.model(frame)
        detections = results.xyxy[0]  # tensor with (x1, y1, x2, y2, confidence, class)
        labels = []
        for detection in detections:
            x1, y1, x2, y2, confidence, class_idx = detection.tolist()
            label = self.model.names[int(class_idx)]
            labels.append({
                'label': label,
                'confidence': confidence,
                'bbox': [x1, y1, x2, y2]
            })

        label_str = json.dumps(labels)
        self.publisher_.publish(String(data=label_str))
        self.get_logger().info('발행 중: "%s"' % label_str)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloV5Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
