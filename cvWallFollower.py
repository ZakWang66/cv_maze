#!/usr/bin/env python
import math, time
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#DRAW
DOT_R = 10

DISTANCE_TO_WALL = 100
HISTORY_SIZE = 10

STATIC_TURN_SPD = 0.58
ARC_FORWARDING_SPD = 0.19

cx = 0
cy = 0
err_history = []

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

        self.degree = 0
        self.lastDegree = 0
        self.hasTurned = 0

        self.turning = 0
        self.start = 0
        self.end = 0
        self.degreeToTurn = 0

    def odom_callback(self, msg):

        degree1 = math.asin(msg.pose.pose.orientation.z) / math.pi * 360
        degree2 = math.acos(msg.pose.pose.orientation.w) / math.pi * 360

        if degree1 >= 0:
            self.degree = degree2
        else:
            self.degree = 360-degree2

    def image_callback(self, msg):

        def calcY(line, x):
            return int((-line[4])*(x - line[0]) + line[1])

        def turnArc(clockwise, forward_speed, degree):
            self.start = self.degree
            if not clockwise:
                self.end = self.start + degree
                if self.start >= 360 - degree:
                    self.end -= 360
                print 'Turn Left', self.start, self.end
                self.turning = 1
                self.twist.angular.z = STATIC_TURN_SPD
            else:
                self.end = self.start - degree
                if self.start <= degree:
                    self.end += 360
                print 'Turn Right', self.start, self.end
                self.turning = -1
                self.twist.angular.z = -STATIC_TURN_SPD

            self.hasTurned = 0
            self.degreeToTurn = degree
            self.twist.linear.x = forward_speed
            self.cmd_vel_pub.publish(self.twist)
            

        global cx
        global cy
        global err_history

        if self.turning != 0:

            accumulate = abs(self.degree - self.lastDegree)

            if self.lastDegree > 350 and self.degree < 10:
                accumulate = 360 - self.lastDegree + self.degree
            
            if self.degree > 350 and self.lastDegree < 10:
                accumulate = 360 - self.degree + self.lastDegree

            self.hasTurned += accumulate
            self.lastDegree = self.degree

            if self.hasTurned > self.degreeToTurn:
                print self.hasTurned, 'finish'
                self.turning = 0

            return

        self.lastDegree = self.degree

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        image = image[len(image) * 3/ 10:]
        h, w, d = image.shape
        #image = cv2.GaussianBlur(image,(3,3),0)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = numpy.array([ 95, 0, 0])
        upper = numpy.array([ 115, 255, 255])
        mask = cv2.inRange(hsv,  lower, upper)
        masked = cv2.bitwise_and(image, image, mask=mask)
        edges = cv2.Canny(masked, 150, 200)
        lines = cv2.HoughLines(edges,1,numpy.pi/180,100)

        

        leftLine = None
        rightLine = None
        frontLine = None
        
        if not lines is None:
            for line in lines:
                line = line[0]
                rho = line[0]
                theta = line[1]
                a = numpy.cos(theta)
                b = numpy.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                k = a/b

                # print k
                if (abs(k) < 3 and abs(k) > 0.5):
                    if k > 0:
                        leftLine = x1, y1, x2, y2, k
                    else:
                        rightLine = x1, y1, x2, y2, k
                        
                if abs(k) < 0.15:
                        frontLine = x1, y1, x2, y2, k

        special = False
        err = 0

        if frontLine is None or max(calcY(frontLine, 0), calcY(frontLine, w)) < h/4:

            # Normal case

            if leftLine is not None and rightLine is not None:

                left_yIntersect = calcY(leftLine, 0)
                right_yIntersect = calcY(rightLine, w)

                cv2.line(image,(leftLine[0], leftLine[1]),(leftLine[2], leftLine[3]),(0,0,255),2)
                cv2.line(image,(rightLine[0], rightLine[1]),(rightLine[2], rightLine[3]),(0,255,255),2)
                
                cv2.circle(image, (0, left_yIntersect), DOT_R, (0,0,0), -1)
                cv2.circle(image, (w, right_yIntersect), DOT_R, (0,0,0), -1)
                err = left_yIntersect - right_yIntersect # x - 320
            else:
                special = True
                self.twist.linear.x = 0

                # No clue, spinning

                if leftLine is None and rightLine is None:
                    self.twist.angular.z = STATIC_TURN_SPD
                else:

                    # Turn sharply to the direcion that the line cannot be seen

                    if leftLine is not None:
                        self.twist.angular.z = -STATIC_TURN_SPD
                        cv2.line(image,(leftLine[0], leftLine[1]),(leftLine[2], leftLine[3]),(0,0,255),2)
                    if rightLine is not None:
                        self.twist.angular.z = STATIC_TURN_SPD
                        cv2.line(image,(rightLine[0], rightLine[1]),(rightLine[2], rightLine[3]),(0,255,255),2)

        else:
            cv2.line(image,(frontLine[0], frontLine[1]),(frontLine[2], frontLine[3]),(0,255,0),2)

            # When see the wall ahead, use the line of wall 's k to balance the err value until the wall is too close

            if max(calcY(frontLine, 0), calcY(frontLine, w)) < h/2:
                err = -frontLine[4] * 10000
            else:
                # Wall is close, need to turn 90 degree conner (maybe cross way) or turn 180 degree back

                special = True
                self.twist.linear.x = 0#0.1
                self.twist.angular.z = 0#STATIC_TURN_SPD
                self.cmd_vel_pub.publish(self.twist)

                if leftLine is not None and rightLine is not None:
                    
                    highest = min(calcY(frontLine, 0), calcY(frontLine, w))

                    # Get 9*9 average color to recognize whether it is a cross road or dead end
                    # print highest, mask[highest][w/2]
                    average = 0
                    for i in range(highest-9, highest):
                        for j in range(w/2-4, w/2+5):
                            average += mask[i][j]
                    average /= 81
                    

                    if average < 128:
                        turnArc(False, ARC_FORWARDING_SPD, 90)
                    else:
                        turnArc(False, 0, 180)
                    cv2.line(image,(leftLine[0], leftLine[1]),(leftLine[2], leftLine[3]),(0,0,255),2)
                    cv2.line(image,(rightLine[0], rightLine[1]),(rightLine[2], rightLine[3]),(0,255,255),2)
                else:
                    if leftLine is None and rightLine is None:
                        #turn90Arc(False)
                        pass
                    else:
                        if leftLine is not None:
                            turnArc(True, ARC_FORWARDING_SPD, 90)
                            cv2.line(image,(leftLine[0], leftLine[1]),(leftLine[2], leftLine[3]),(0,0,255),2)
                        if rightLine is not None:
                            turnArc(False, ARC_FORWARDING_SPD, 90)
                            cv2.line(image,(rightLine[0], rightLine[1]),(rightLine[2], rightLine[3]),(0,255,255),2)

        self.logcount += 1

        cv2.circle(image, (0, h), 20, (0,0,255), -1)
        cv2.circle(image, (w, h), 20, (0,255,255), -1)
        
        # Calc I, D
        err_history.append(err)
        if (len(err_history) > HISTORY_SIZE):
            err_history.pop(0)
        iTerm = sum(err_history) / len(err_history)
        if (len(err_history)%2 == 0):
            dTerm = sum(err_history[len(err_history)/2:]) - sum(err_history[:len(err_history)/2])
        else:
            dTerm = sum(err_history[(len(err_history) - 1)/2:-1]) - sum(err_history[:(len(err_history) - 1)/2])


        if not special:
            self.twist.linear.x = 0.30
            self.twist.angular.z = -float(1 * err + 0.0 * iTerm + 0.0 * dTerm) / 500

        self.cmd_vel_pub.publish(self.twist)
        
        
        # print("M00 %d %d" % (M['m00'], self.logcount))
        print self.degree
        print("%d err: %f; iTerm: %f; dTerm: %f; z: %f" % (self.logcount, err, iTerm, dTerm, self.twist.angular.z))


        cv2.imshow("mask", mask)
        cv2.imshow("image", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()


        # def intersection(line1, line2):
        #     # x=(k1*x0-k2*x2+y2-y0)/(k1-k2)
        #     # y=y0+(x-x0)*k1
        #     x = int((line1[4] * line1[0] - line2[4] * line2[0] + line1[1] - line2[1]) / (line1[4] - line2[4]))
        #     y = int(line1[1] + (x - line1[0]) * -line1[4])
        #     return x, y


        # def bisector(line1, line2, intersection):
        #     def getDfarPointOnLine(line, distance, point, atLeft):
        #         k = line[4]
        #         x0 = point[0]
        #         y0 = point[1]
        #         x = distance/math.sqrt(k*k+1)
        #         if atLeft:
        #             x = -x
        #         x += x0
        #         y = -k*(x-x0) + y0
        #         return x, y
        #     x1, y1 = getDfarPointOnLine(line1, 100, intersection, True)
        #     x2, y2 = getDfarPointOnLine(line2, 100, intersection, False)
        #     k1 = -1/line1[4]
        #     k2 = -1/line2[4]
        #     x_critical = (x2 - k1/k2 * x1) / (1 - k1/k2)
        #     y_critical = -k1 * (x_critical - x1) + y1
        #     return x, y, int(x_critical), int(y_critical), (y - y_critical)/(x - x_critical)


        # clear all but a 20 pixel band near the top of the image
        
        # search_top = 2 * h /5
        # search_bot = h
        # mask[0:search_top, 0:w] = 0
        # mask[search_bot:h, 0:w] = 0
        

        # Compute the "centroid" and display a red circle to denote it
        # M = cv2.moments(mask)

        # if M['m00'] > 0:
        #     cx = int(M['m10']/M['m00'])
        #     cy = int(M['m01']/M['m00'])
        # else:
        #     cx = 0


                # lines = cv2.HoughLinesP(edges,1,numpy.pi/180,10,1000, 1)
        # if not lines is None:
        #     for line in lines:
        #         x1,y1,x2,y2 = line[0]
        #         if x1 == x2:
        #             k = float('Inf')
        #         else:
        #             k = -float(y2-y1)/float(x2-x1)
                
        #         if (len(image) - max(y1, y2) < 20) or x1 < 10 or len(image[0]) - x2 < 10:




            # if self.turning > 0:
            #     if (self.degree > self.end):
            #         print self.start, self.degree
            #         if (self.start >= 360 - self.degreeToTurn and self.degree >= self.start):
            #             pass
            #         else:
            #             print 'finish'
            #             self.turning = 0
            # else:
            #     if (self.degree < self.end):
            #         print self.start, self.degree
            #         if (self.start <= self.degreeToTurn and self.degree <= self.start):
            #             pass
            #         else:
            #             print 'finish'
            #             self.turning = 0