#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

DISTANCE_TO_WALL = 50
HISTORY_SIZE = 10

cx = 0
cy = 0
err_history = []

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

    def image_callback(self, msg):
        global cx
        global cy
        global err_history

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = numpy.array([ 95, 0, 0])
        upper = numpy.array([ 115, 255, 255])
        mask = cv2.inRange(hsv,  lower, upper)
        masked = cv2.bitwise_and(image, image, mask=mask)

        
        
        # clear all but a 20 pixel band near the top of the image
        h, w, d = image.shape
        
        search_top = 3 * h /4
        search_bot = search_top + 30
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        

            # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        self.logcount += 1

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        else:
            cx = 0

        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

        # Move at 0.2 M/sec
        # add a turn if the centroid is not in the center
        # Hope for the best. Lots of failure modes.
        err = cx - DISTANCE_TO_WALL
        cv2.circle(image, (err + w/2, cy), 20, (0,255,0), -1)
        
        # Calc I, D
        err_history.append(err)
        if (len(err_history) > HISTORY_SIZE):
            err_history.pop(0)
        iTerm = sum(err_history) / len(err_history)
        if (len(err_history)%2 == 0):
            dTerm = sum(err_history[len(err_history)/2:]) - sum(err_history[:len(err_history)/2])
        else:
            dTerm = sum(err_history[(len(err_history) - 1)/2:-1]) - sum(err_history[:(len(err_history) - 1)/2])

        
        self.twist.linear.x = 0.20
        self.twist.angular.z = -float(0.4 * err + 0.1 * iTerm + 0.0 * dTerm) / 100
        self.cmd_vel_pub.publish(self.twist)
        
        
        # print("M00 %d %d" % (M['m00'], self.logcount))
        print("%d err: %d; iTerm: %d; dTerm: %d; z: %f" % (self.logcount, err, iTerm, dTerm, self.twist.angular.z))


        cv2.imshow("mask", mask)
        cv2.imshow("image", image)
        cv2.waitKey(3)
        
rospy.init_node('follower')
follower = Follower()
rospy.spin()