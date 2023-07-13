import rclpy
from rclpy.node import Node
import os
import pkg_resources
import cv2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64MultiArray
import base64
import json
import math

class VideoPublisher(Node):



    def __init__(self, input_source):
        super().__init__('video_publisher')
        self.using_rgbd = False # Not using RGB-D by default
        self.printedDepth = False
        self.printedPoint = False
        self.printedImage = False

        self.publisher_ = self.create_publisher(Image, 'video_frame', 10)

        self.bbox_subscription = self.create_subscription(
            Float64MultiArray,
            'object_detect',
            self.listener_callback,
            10
        )
        self.bbox_subscription

        self.rgbd_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgbd_callback,
            10
        )
        self.rgbd_subscription

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )


        self.frame_list = [] # To store frames for later processing
        
        self.bridge_ = CvBridge()

        if input_source == '0':
            input_source = 0
        elif input_source == '2':
            self.using_rgbd = True

        if not self.using_rgbd:
            self.video_capture = cv2.VideoCapture(input_source)
            if self.video_capture.isOpened():
                self.timer_ = self.create_timer(0.1, self.publish_frame)

    def pointcloud_callback(self, msg):
        if not self.printedPoint:
            print(f"Received point cloud dimension: {msg.height} x {msg.width}")
            self.printedPoint = True

    def depth_callback(self, msg):
        if not self.printedDepth:
            print(f"Received depth message with dimension: {msg.height} x {msg.width}")
            self.printedDepth = True

    def listener_callback(self, msg):
        
        data = msg.data
        timestamp_sec = data[-1]
        data = data[:-1]
        # Process the remaining data (bounding boxes)
        bounding_boxes = [list(data[i:i+4]) for i in range(0, len(data), 4)]
        self.display_frame_with_bbox(bounding_boxes, timestamp_sec)

    def rgbd_callback(self, image):
        if not self.printedImage:
            print(f"Received image with dimension: {image.height} x {image.width}")
            self.printedImage = True
        self.publish_frame(image) # Just publisht the frame received from RGB-D camera

    def publish_frame(self, msg=None):
        if msg is None:
            ret, frame = self.video_capture.read()
            if ret:
                cv2.imshow('Input from the camera', frame)
                cv2.waitKey(1)
                msg = self.bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
                # curr_time = time.time() - init_time
                msg.header.stamp = self.get_clock().now().to_msg()
                self.frame_list.append(msg) # Store the frame to the queue
                self.publisher_.publish(msg)
        else:
            frame = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Input from the RGB-D', frame)
            cv2.waitKey(1)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.frame_list.append(msg) # Store the frame to the queue
            self.publisher_.publish(msg)
        if self.frame_list.__sizeof__() > 10240:
            self.frame_list = self.frame_list[math.floor(len(self.frame_list)/2):]
            
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

    def convert_image_to_json(self, image, bboxes):
        # Convert image to Base64
        _, img_encoded = cv2.imencode('.jpg', image)
        img_base64 = base64.b64encode(img_encoded).decode('utf-8')

        # Create JSON payload
        payload = {
            'image': img_base64,
            'bounding_box': bboxes
        }

        # Convert payload to JSON string
        json_payload = json.dumps(payload)

        return json_payload        

def choose_input_source():
    valid_choices = ['0', '1', '2']

    while True:
        choice = input("Choose an input source: \n" +
        "0. Normal Camera\n" + 
        "1. Video\n" +
        "2. RGB-D Camera")

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
