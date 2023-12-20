#!/usr/bin/env python3

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np

fx, fy, cx, cy = 610, 610, 320, 240
image_width = 640
image_height = 480

# Drone Dimensions (in meters)
drone_width = 0.3
drone_height = 0.15
drone_dim_top_view = 0.3  # drone's third dimension

# These are the dimensions of the ROI (rectangle) where the image processing will be done.
# Limits are: min - drone's dimensions in image frame; max - image dimensions
roi_im_b = image_width - 100  # the breadth of the ROI
roi_im_h = image_height - 100  # the height of the ROI

# Drone's dimensions in image frame assuming that it is present 2m in front of the camera
drone_im_b = 92  # breadth
drone_im_h = 46  # height

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

        # Define near and far thresholds (adjust as needed)
        far_threshold = 2000 + 100
        near_threshold = 2000 - 100

        # Coordinates of the ROI rectangle box in the image frame
        roi_x1, roi_y1 = int(cx - roi_im_b / 2), int(cy - roi_im_h / 2)
        roi_x2, roi_y2 = int(cx + roi_im_b / 2), int(cy + roi_im_h / 2)

        # Out of the color image, select only the ROI pixels setting the intensity of the rest to 0
        roi_image = np.zeros_like(color_cv_image)
        roi_image[roi_y1:roi_y2, roi_x1:roi_x2] = color_cv_image[roi_y1:roi_y2, roi_x1:roi_x2]

        mask_bg = np.where((depth_cv_image > far_threshold), 255, 0).astype(np.uint8)
        mask_fg = np.where((depth_cv_image < near_threshold), 255, 0).astype(np.uint8)
        mask_uncertain = np.where((depth_cv_image > near_threshold) & (depth_cv_image < far_threshold), 255, 0).astype(
            np.uint8)

        # Apply the mask to the grayscale image using mask_bg since we are interested to pass through the gap
        bg_info_image = cv2.bitwise_and(roi_image, roi_image, mask=mask_bg)

        bg_info_gray = cv2.cvtColor(bg_info_image, cv2.COLOR_BGR2GRAY)

        contours, _ = cv2.findContours(bg_info_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through contours and find the contour with maximum area
        max_contour = None
        max_area = 0

        for contour in contours:
            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Update max_area and max_contour if the current contour has a larger area
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            # Get the minimum bounding rectangle of the max_area contour
            rect = cv2.minAreaRect(max_contour)

            # Extract the rectangle parameters
            (cx_gap_box, cy_gap_box), (w, h), angle = rect

            tol_to_align_centers = 50  # This tolerance is used to check if the center of the max_area rectangle is aligning with that of ROI
            tol = 50  # This tolerance is used to check if there is any obstacle in front of the drone within 2 m. If that space is clear, the drone can pass through without checking for the remaining conditions for the current frame

            if abs(w - roi_im_b) < tol and abs(h - roi_im_h) < tol:  # Condition to check if dimensions of max_area rectangle are close to ROI in the current img frame
                print("It is a free space ahead! The drone can pass through freely")

            elif abs(cx_gap_box - cx) < tol_to_align_centers and abs(cy_gap_box - cy) < tol_to_align_centers:  # centers are aligning
                if w > drone_im_b and h > drone_im_h:
                    print("The drone can pass safely without morphing")  # If gap dimensions are more than that of the drone, move forward
                else:
                    print("The drone has to morph")  # If gap dimensions are less than that of the drone, morph

            elif abs(cx_gap_box - cx) > tol_to_align_centers and abs(cy_gap_box - cy) < tol_to_align_centers:  # x-coordinate of the centers are not aligning
                if (cx_gap_box - cx) < 0:
                    print("Try turning the drone towards left to check for a better window to pass through")

                elif (cx_gap_box - cx) > 0:
                    print("Try turning the drone towards right to check for a better window to pass through")

            elif abs(cx_gap_box - cx) < tol_to_align_centers and abs(cy_gap_box - cy) > tol_to_align_centers:  # y-coordinate of the centers are not aligning
                if (cy_gap_box - cy) < 0:
                    print("Try turning the drone upwards to check for a better window to pass through")

                elif (cy_gap_box - cy) > 0:
                    print("Try turning the drone downwards to check for a better window to pass through")

            # Draw the minimum bounding rectangle on the image
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(bg_info_image, [box], 0, (0, 255, 0), 2)

        cv2.rectangle(bg_info_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 255), 2)
        cv2.imshow('Original Color', color_cv_image)
        cv2.imshow('Image of interest', bg_info_image)

        # cv2.imshow("Color Image window", color_cv_image)
        # cv2.imshow("Depth Image window", depth_cv_image)
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
