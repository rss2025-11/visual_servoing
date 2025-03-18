#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from vs_msgs.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("cone_detector")
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Cone Detector Initialized")

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        input_image = image
        offset_mult = 1 if self.LineFollower else 0
        lower_orange =  np.array([0, 0, 0])
        upper_orange = np.array([255,255,255])

        if self.LineFollower:
            cropped_image = image[152:258, :]
            input_image = cropped_image
            lower_orange =  np.array([10, 120, 115])
            upper_orange = np.array([15,255,255])
        else:
            #The 8, x, 80, allows for darker oranges to be picked up but not some reds
            # the x,200,x prevents lighter browns from being registered
            #28 allows for bright oranges to be picked up but not yellows
            lower_orange =  np.array([5, 200, 160])
            upper_orange = np.array([15,255,255])


        # if self.LineFollower:
        #     lower_orange =  np.array([5, 200, 160])
        #     upper_orange = np.array([15,255,255])

        bounding_box = cd_color_segmentation(input_image, (lower_orange, upper_orange))

        lower_right_corner = bounding_box[1]

        lower_left_corner = (bounding_box[0][0], bounding_box[1][1])

        lower_middle_point =( (lower_left_corner[0] + lower_right_corner[0])/2, lower_right_corner[1] )

        cone_location_px_msg = ConeLocationPixel()
        cone_location_px_msg.u = float(lower_middle_point[0]) 
        cone_location_px_msg.v = float(lower_middle_point[1] + offset_mult*130)
        
        self.cone_pub.publish(cone_location_px_msg)

        cv2.rectangle(input_image, bounding_box[0], bounding_box[1], (0,0,255), 3)
        
        debug_msg = self.bridge.cv2_to_imgmsg(input_image, "bgr8")
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    rclpy.spin(cone_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
