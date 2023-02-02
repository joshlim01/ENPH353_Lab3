#! /usr/bin/env python3

import rospy
import cv2
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

THRESHOLD = 100 # robot correction value to center (80-120 is acceptable)
PREVIOUS_CENTER = 0
RADIUS = 20
ANGULAR_SPEED = 5

def callback(data):
    global PREVIOUS_CENTER

    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    rate = rospy.Rate(2)
    move = Twist()
    move.linear.x = 0.5
    #move.angular.z = 0.5

    grayScale = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    ret, mask = cv2.threshold(grayScale,70,255,cv2.THRESH_BINARY_INV)
    ret, mask = cv2.threshold(grayScale,70,255,cv2.THRESH_BINARY_INV)
    strip = mask[-RADIUS,:,]

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
    
    center_x = find_center(strip)
    h, w, c = cv_image.shape
    center_y = h - RADIUS

    circle = cv2.circle(cv_image, center=(center_x,center_y), radius=RADIUS, color=(0,0,255), thickness=-1)
    cv2.imshow("camera view", circle)
    cv2.waitKey(3)

    if center_x == None:
        path_center = PREVIOUS_CENTER
    else:
        path_center = center_x

    PREVIOUS_CENTER = path_center
    path_diff = np.abs(path_center - (w / 2))

    if path_diff > THRESHOLD:
        if path_center < w / 2:
            move.angular.z = ANGULAR_SPEED
        if path_center > w / 2:
            move.angular.z = -ANGULAR_SPEED

    cv.imshow("Image window", cv_image)
    cv.waitKey(3)

    #try: 
        #image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
        #print(e)

    move_pub.publish(move)
    return None

if __name__ == '__main__':
    #image_pub = rospy.Publisher("image_topic_2",Image)
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image, callback)
    move_pub = rospy.Publisher('/cmd_vel', Twist, 
    queue_size=1)

    rospy.init_node('image_converter', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()