import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class YoloV5Subscriber(Node):
    def __init__(self):
        super().__init__('yolov5_subscriber')
        self.subscription = self.create_subscription(
            String,
            'yolov5/labels',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        labels = json.loads(msg.data)
        for label in labels:
            self.get_logger().info(f'Received: Label={label["label"]}, Confidence={label["confidence"]}, BBox={label["bbox"]}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloV5Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

