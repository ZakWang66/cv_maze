#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
from cv_maze.msg import LineData
from sensor_msgs.msg import Image

line_info = None


def image_callback(msg):
    global line_info
    # get image from camera
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg)
    image = image[len(image) * 3 / 10:]
    h, w, d = image.shape

    if line_info is not None:

        leftLine = line_info.leftLine
        rightLine = line_info.rightLine
        frontLine = line_info.frontLine
        leftIntersect = line_info.leftIntersect
        rightIntersect = line_info.rightIntersect

        if leftLine is not None and len(leftLine) != 0:
            if line_info.wallLeft:
                cv2.line(image, (leftLine[0], leftLine[1]), (leftLine[2], leftLine[3]), (0, 0, 255), 2)
            else:
                cv2.line(image, (leftLine[0], leftLine[1]), (leftLine[2], leftLine[3]), (255, 0, 255), 2)
        if rightLine is not None and len(rightLine) != 0:
            cv2.line(image, (rightLine[0], rightLine[1]), (rightLine[2], rightLine[3]), (0, 255, 255), 2)
        if frontLine is not None and len(frontLine) != 0:
            if line_info.wallFront:
                cv2.line(image, (frontLine[0], frontLine[1]), (frontLine[2], frontLine[3]), (0, 255, 0), 2)
            else:
                cv2.line(image, (frontLine[0], frontLine[1]), (frontLine[2], frontLine[3]), (255, 0, 255), 2)
        if leftIntersect != -1:
            cv2.circle(image, (0, leftIntersect), 5, (0, 0, 0), -1)
        if rightIntersect != -1:
            cv2.circle(image, (w, rightIntersect), 5, (0, 0, 0), -1)
        line_info = None
    cv2.imshow("image", image)
    cv2.waitKey(3)


def line_callback(msg):
    global line_info
    line_info = msg


rospy.init_node('visualization')
image_sub = rospy.Subscriber('camera/image', Image, image_callback)
line_sub = rospy.Subscriber('/line_detection', LineData, line_callback)
rospy.spin()
