#! /usr/bin/env python3

import rospy
import cv2 as cv
import matplotlib.pyplot as plt
import sys
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Finds the middle index of the road (used in circle)
    def find_center(strip):
        road = []
        for i in range(len(strip)):
            if(strip[i] == 255):
                road.append(i)
        if len(road) == 0:
            return None
        else:
            road = np.array(road)
            return int(np.mean(road))
    
    # Places a circle in the middle of the road on a specific frame
    def center(frame):
        grayScale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        radius = 20

        #Threshold to mask
        ret, mask = cv2.threshold(grayScale,70,255,cv2.THRESH_BINARY_INV)
        strip = mask[-radius,:,]
        center_x = find_center(strip)
        h, w, c = frame.shape
        center_y = h - radius
        image = np.copy(frame)


    # Line detection on every frame
    ret, frame = cv_image.read()

    while ret == True:
        center(frame)
        ret, frame = cv_image.read()

    # cv.imshow("Image window", cv_image)
    # cv.waitKey(3)
    move_pub.publish(move)
    return None


bridge = CvBridge()
image_sub = rospy.Subscriber("/robot/camera1/image_raw", Image, callback)
move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

rospy.init_node('image_converter', anonymous=True)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting Down")
cv.destroyAllWindows()