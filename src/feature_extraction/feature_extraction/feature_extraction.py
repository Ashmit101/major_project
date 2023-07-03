import rclpy
from rclpy.node import Node
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

class FeatureExtractor(Node):
    def __init__(self):
        super().__init__("feature_extractor")

        self.img_subscriber = self.create_subscription(
            Image,
            "video_frame",
            self.image_callback,
            10
        )
        self.img_subscriber

        self.obj_subscriber = self.create_subscription(
            Float64MultiArray,
            "object_detect",
            self.obj_callback,
            10
        )
        self.obj_subscriber

        self.feature_publisher = None

        # self.feature_publisher = self.create_publisher(
        #     Float64MultiArray,
        #     'person_feature',
        #     10
        # )

        self.bridge = CvBridge()
        self.bboxedges = None
        self.cropped = None
        self.feature_array = None
    
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        # cv2.imshow("image", image)


        if self.bboxedges != None:
            self.cropped = []
            for bbox in self.bboxedges:
                x = bbox[0]
                y = bbox[1]
                w = bbox[2]
                h = bbox[3]
                self.cropped.append(image[y:y+h, x:x+w]) 

        if self.cropped != None:
            self.feature_array = []
            for cropped_img in self.cropped:
                feature = self.detect_features(cropped_img)
                self.feature_array.append(feature)
            self.publish_features()
                # self.feature_array.data.append(feature)
            # self.feature_publisher.publish(self.feature_array)

        

    def obj_callback(self, msg):
        self.bboxedges = []
        for i in range(int((len(msg.data)-1)/4)):
            self.bboxedges.append([int(msg.data[0+i]), int(msg.data[1+i]), int(msg.data[2+i]), int(msg.data[3+i])])

    def detect_features(self, img):
        # Initiate ORB detector
        orb = cv2.ORB_create()
        # find the keypoints with ORB
        kp = orb.detect(img,None)
        # compute the descriptors with ORB
        kp, des = orb.compute(img, kp)
        return des

    def publish_features(self):
        for i in range(len(self.feature_array)):
            feature = Float64MultiArray()
            self.feature_publisher = self.create_publisher(
                    Float64MultiArray,
                    f"people_feature_{i}",
                    10
                )
                    
            # Convert 2D array to 1D array and then publish it with width and height as feature
            for dat in self.feature_array[i]:
                for x in dat:
                    feature.data.append(x)
            
            feature.data.append(len(self.feature_array[i]))
            feature.data.append(len(self.feature_array[i][0]))
            
            self.feature_publisher.publish(feature)
            

def main(args=None):
    rclpy.init(args=args)
    feature_extractor = FeatureExtractor()
    rclpy.spin(feature_extractor)
    feature_extractor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()