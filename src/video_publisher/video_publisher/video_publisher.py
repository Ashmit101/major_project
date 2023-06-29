import rclpy
from rclpy.node import Node
import os
import pkg_resources
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self, input_source):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frame', 10)
        
        self.bridge_ = CvBridge()

        if input_source == '0':
            input_source = 0

        self.video_capture = cv2.VideoCapture(input_source)

        if self.video_capture.isOpened():
            self.timer_ = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):

        ret, frame = self.video_capture.read()
        if ret:
            cv2.imshow('Input from the camera', frame)
            cv2.waitKey(1)
            msg = self.bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
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

    if input_source == '1':
        input_source = input("Enter complete path of the video file:")

    rclpy.init(args=args)
    video_publisher = VideoPublisher(input_source)
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
