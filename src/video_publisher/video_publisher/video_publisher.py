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

        self.frame_list = [] # To store frames for later processing
        
        self.bridge_ = CvBridge()

        if input_source == '0':
            input_source = 0

        self.video_capture = cv2.VideoCapture(input_source)

        if self.video_capture.isOpened():
            self.timer_ = self.create_timer(0.1, self.publish_frame)

    def listener_callback(self, msg):
        
        data = msg.data
        timestamp_sec = data[-1]
        data = data[:-1]
        # Process the remaining data (bounding boxes)
        bounding_boxes = [list(data[i:i+4]) for i in range(0, len(data), 4)]
        self.display_frame_with_bbox(bounding_boxes, timestamp_sec)


    def publish_frame(self):

        ret, frame = self.video_capture.read()
        if ret:
            cv2.imshow('Input from the camera', frame)
            cv2.waitKey(1)
            msg = self.bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
            # curr_time = time.time() - init_time
            msg.header.stamp = self.get_clock().now().to_msg()
            self.frame_list.append(msg) # Store the frame to the queue
            self.publisher_.publish(msg)

    def display_frame_with_bbox(self, coordinates, timestamp):

        i = 0
        for frame in self.frame_list:
            frame_timestamp = frame.header.stamp
            frame_timestamp = frame_timestamp.sec + frame_timestamp.nanosec * 1e-9

            if timestamp == frame_timestamp:
                frame_cv = self.bridge_.imgmsg_to_cv2(frame, 'bgr8')

                # Draw the bounding box on the frame
                for box in coordinates:
                    x1, y1, x2, y2 = box
                    cv2.rectangle(frame_cv, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                # Display the frame with the bounding box
                cv2.imshow('Frame with Bounding Box', frame_cv)
                cv2.waitKey(1)
                self.frame_list = self.frame_list[i+1:]
                break
            i = i + 1

        

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
