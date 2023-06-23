import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node

from sensor_msgs.msg import Image

class EdgeDetector(Node):

    def __init__(self):
        super().__init__('edge_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription

    def image_callback(self, msg):
        
        self.get_logger().info(f"Received height: {msg.height}, width: {msg.width}")

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")    

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray_image, 100, 200)  

        cv2.imshow("Edges", edges)

        cv2.waitKey(1)

def main(args=None):
    print("Staring to process")
    rclpy.init(args=args)
    image_subscriber = EdgeDetector()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()