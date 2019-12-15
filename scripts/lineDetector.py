#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
import numpy
from cv_maze.msg import LineData
from sensor_msgs.msg import CompressedImage


def image_callback(msg):

    def calcY(line, x):
        return int((-line[4]) * (x - line[0]) + line[1])

    def calcX(line, y):
        return int((y - line[1]) / (-line[4]) + line[0])

    # get image from camera
    bridge = cv_bridge.CvBridge()
    image = bridge.compressed_imgmsg_to_cv2(msg)

    # rotate 180
    h, w = image.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, 180, 1)
    image = cv2.warpAffine(image, M, (w, h))

    image = image[len(image) * 3 / 10:]
    h, w, d = image.shape
    # image = cv2.Gau\ssianBlur(image,(3,3),0)

    # process image to get lines
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    grayImage = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)

    edges = cv2.Canny(mask, 150, 200)
    lines = cv2.HoughLines(edges, 1, numpy.pi / 180, 80)

    # get the three critical lines
    leftLine = None
    rightLine = None
    frontLine = None
    minK = 0
    maxK = 0
    horizentalK = 0.05
    # print(len(lines))
    if lines is not None:
        for line in lines:
            line = line[0]
            rho = line[0]
            theta = line[1]
            a = numpy.cos(theta)
            b = numpy.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            if b == 0:
                k = float("inf")
            else:
                k = a / b

            if (abs(k) < 3 and abs(k) > 0.3):
                if k > 0 and k > maxK:
                    maxK = k
                    leftLine = x1, y1, x2, y2, k
                elif k < 0 and k < minK:
                    minK = k
                    rightLine = x1, y1, x2, y2, k

            if abs(k) < horizentalK:
                horizentalK = abs(k)
                frontLine = x1, y1, x2, y2, k

    # determin whether there are walls at left, right and front
    # Get 9*9 average color to recognize whether it is a cross road or dead end

    # img = numpy.copy(image)
    wallFront = False
    wallLeft = False

    leftIntersect = -1
    rightIntersect = -1
    frontMidY = -1

    count = 0
    if frontLine is not None and len(frontLine) != 0:
        middle = w / 2
        y = calcY(frontLine, middle)
        frontMidY = y
        averageFront = 0
        for i in range(y - 11, y - 2):
            for j in range(middle - 4, middle + 5):
                if i < 0 or i >= h:
                    continue
                if j < 0 or j >= w:
                    continue
                averageFront += mask[i][j]
                count += 1
                # img[i, j, :] = mask[i, j]
        if count != 0:
            averageFront /= count
        if averageFront > 127:
            wallFront = True

    count = 0
    if leftLine is not None and len(leftLine) != 0:
        leftIntersect = calcY(leftLine, 0)
        middle = 2 * h / 3
        x = calcX(leftLine, middle)
        averageLeft = 0
        for i in range(middle - 4, middle + 5):
            for j in range(x - 11, x - 2):
                if i < 0 or i >= h:
                    continue
                if j < 0 or j >= w:
                    continue
                averageLeft += mask[i][j]
                count += 1
                # img[i, j, :] = mask[i, j]
        if count != 0:
            averageLeft /= count
        if averageLeft > 127:
            wallLeft = True

    if rightLine is not None and len(rightLine) != 0:
        rightIntersect = calcY(rightLine, w)

    # if leftLine is not None and len(leftLine) != 0:
    #     cv2.line(img, (leftLine[0], leftLine[1]), (leftLine[2], leftLine[3]), (0, 0, 255), 2)
    # if rightLine is not None and len(rightLine) != 0:
    #     cv2.line(img, (rightLine[0], rightLine[1]), (rightLine[2], rightLine[3]), (0, 255, 255), 2)
    # if frontLine is not None and len(frontLine) != 0:
    #     cv2.line(img, (frontLine[0], frontLine[1]), (frontLine[2], frontLine[3]), (0, 255, 0), 2)
    # cv2.imshow("image", img)
    # cv2.waitKey(3)

    pubLine.publish(leftLine, rightLine, frontLine, wallLeft, False, wallFront, leftIntersect, rightIntersect, frontMidY, h, w)


rospy.init_node('line_detector')
image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, image_callback)
pubLine = rospy.Publisher('/line_detection', LineData, queue_size=10)
rospy.spin()
