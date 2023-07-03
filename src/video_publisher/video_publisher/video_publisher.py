import rclpy
from rclpy.node import Node
import os
import pkg_resources
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64MultiArray

class VideoPublisher(Node):
    def __init__(self, input_source):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frame', 10)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'object_detect',
            self.listener_callback,
            10
        )

        self.subscription
        
        self.bridge_ = CvBridge()

        if input_source == '0':
            input_source = 0

        self.video_capture = cv2.VideoCapture(input_source)

        if self.video_capture.isOpened():
            self.timer_ = self.create_timer(0.1, self.publish_frame)

    def listener_callback(self, msg):
        # Process received message
        data = msg.data
        timestamp_sec = data[-1]
        data = data[:-1]
        # Process the remaining data (bounding boxes)
        bounding_boxes = [list(data[i:i+4]) for i in range(0, len(data), 4)]
    
        # Do something with the bounding boxes and timestamp
        # For example, print them
        self.get_logger().info(f'Received bounding boxes: {bounding_boxes}')
        self.get_logger().info(f'Received timestamp: {timestamp_sec}')


    def publish_frame(self):

        ret, frame = self.video_capture.read()
        if ret:
            cv2.imshow('Input from the camera', frame)
            cv2.waitKey(1)
            msg = self.bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
            # curr_time = time.time() - init_time
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)

def choose_input_source():
    valid_choices = ['0', '1']

    while True:
        choice = input("Choose an input source: \n" +
        "0. Camera\n" + 
        "1. Video\n")

        if choice in valid_choices:
            return choice.lower()
        else:
            print("Invalid choice. Please try again. (0/1)")

def main(args=None):
    
    input_source = choose_input_source()

    # global init_time
    # init_time = time.time()

    if input_source == '1':
        input_source = input("Enter complete path of the video file:")

    rclpy.init(args=args)
    video_publisher = VideoPublisher(input_source)
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
