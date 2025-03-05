#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import apriltag
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from burger_imu.msg import TagIdsWithDistances  # Import the custom message

class AprilTagDetector:
    def __init__(self):
        rospy.init_node('apriltag_detector', anonymous=True)
        self.bridge = CvBridge()
        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)
        self.tag_pub = rospy.Publisher('/apriltag_ids_sorted_by_distance', TagIdsWithDistances, queue_size=10)

        # Subscribers for synchronized color and depth images
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.depth_image = []

    def depth_callback(self, msg):
        self.depth_image = msg


    def image_callback(self, msg):
        try:
            # Convert the ROS image messages to OpenCV images
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr("Failed to convert images: %s", e)
            return

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        
        print(results)
        
        tags_with_distances = []

        for result in results:
            c = result.center
            x, y = int(c[0]), int(c[1])
            distance = depth_image[y, x]

            # Only consider valid distances
            if distance > 0:
                tags_with_distances.append((result, distance))

        tags_with_distances.sort(key=lambda x: x[1])

        tag_ids_msg = TagIdsWithDistances()
        tag_ids_msg.tag_ids = [result[0].tag_id for result in tags_with_distances]

        self.tag_pub.publish(tag_ids_msg)

if __name__ == '__main__':
    try:
        detector = AprilTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
