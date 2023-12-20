#!/usr/bin/env python3

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters

class image_converter:

    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)  # Initialize ROS node here

        self.image_pub = rospy.Publisher("image_topic_2", Image)

        self.bridge = CvBridge()
        self.color_image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_image_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        ts = message_filters.TimeSynchronizer([self.color_image_sub, self.depth_image_sub], 10)
        ts.registerCallback(self.img_cb)

    def img_cb(self, color_img_data, depth_img_data):
        try:
            color_cv_image = self.bridge.imgmsg_to_cv2(color_img_data, "bgr8")
            depth_cv_image = self.bridge.imgmsg_to_cv2(depth_img_data, "passthrough")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Color Image window", color_cv_image)
        cv2.imshow("Depth Image window", depth_cv_image)
        cv2.waitKey(3)

def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
