import cv2
import os
import pkg_resources
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'person_detector'

class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'video_frame',
            self.image_callback,
            10
        )

        self.subscription

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'object_detect', 
            10)
        
        
        self.bridge= CvBridge()

        self.yolocfg, self.yoloweights, self.yolotxt = self.access_yolov3_files()


    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        Width = image.shape[1]
        Height = image.shape[0]
        scale = 0.00392
        print(msg.header.stamp)
        timestamp = msg.header.stamp

        # read class names, here only person is stored
        classes = None
        with open(self.yolotxt, 'r') as f:
             classes = [line.strip() for line in f.readlines()]


        # read pre-trained model and config file
        net = cv2.dnn.readNet(self.yoloweights, self.yolocfg)

        # create input blob 
        blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)
        net.setInput(blob)


        # to get the output layer names 
        def get_output_layers(net):
            layer_names = net.getLayerNames()
            output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
            return output_layers

        
        #  for determining the coordinates of the box
        bounding_boxes = []


        #  for drawing bounding box on the detected object with class name
        def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
            if class_id < len(classes) and classes[class_id] == "person":
                label = str(classes[class_id])
                color = (0,255,0)
                cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
                cv2.putText(img, label, (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                bounding_boxes.append((x, y, x + w, y + h))
            
            else:
                return

        
        # for output layers, used for bluebox detection and better prediction
        outs = net.forward(get_output_layers(net))


        class_ids = []
        confidences = []
        boxes = []
        conf_threshold = 0.5
        nms_threshold = 0.4

        # for each detection from output layers, we dont take if confidence is less than 0.5
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * Width)
                    center_y = int(detection[1] * Height)
                    w = int(detection[2] * Width)
                    h = int(detection[3] * Height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])

        
        # apply non-max suppression to filter out overlapping bounding boxes based on their confidence scores
        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)


        for i in indices:
            box = boxes[i]
            x = box[0]
            y = box[1]
            w = box[2]
            h = box[3]
    
        draw_bounding_box(image, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))

        # determines the coordinates of the bbox
        
        if bounding_boxes:
            coordinates=Float64MultiArray()
            for box in bounding_boxes:
                coordinates.data.extend(box)
            timestamp_sec = timestamp.sec + timestamp.nanosec * 1e-9
            print(timestamp)
            coordinates.data.append(timestamp_sec)
            self.publisher_.publish(coordinates)

       
        # msg=String()
        # msg.data=box
        # msg1=timestamp()
        # sec=msg1.sec
        # nanosec=msg1.nanosec
        # self.publisher.publish(msg, sec, nanosec)

    def access_yolov3_files(self):
        # Get the path to the yolov3 directory using pkg_resources
        package_share_directory = get_package_share_directory('person_detector')
        yolov3_dir = os.path.join(package_share_directory, 'yolov3')
        # Access the yolov3 files using their relative paths
        cfg_file = os.path.join(yolov3_dir, 'yolov3.cfg')
        weights_file = os.path.join(yolov3_dir, 'yolov3.weights')
        names_file = os.path.join(yolov3_dir, 'yolov3.txt')

        return cfg_file, weights_file, names_file


def main(args=None):
    rclpy.init(args=args)
    object_detection_node= ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()