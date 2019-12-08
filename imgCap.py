import math, time
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

numpy.set_printoptions(threshold=numpy.inf)

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
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback)

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

        def calcX(line, y):
            return int((y - line[1]) / (-line[4]) + line[0])

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

            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            

        global cx
        global cy
        global err_history


        # get image from camera
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        # rotate 180
        h, w = image.shape[:2]
        center = (w // 2, h // 2)
        M = cv2.getRotationMatrix2D(center, 180, 1)
        image = cv2.warpAffine(image, M, (w, h))

        # cut image
        image = image[len(image) * 3/ 10:]
        h, w, d = image.shape
        
        # get lines from the image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        grayImage = cv2.cvtColor(hsv,cv2.COLOR_BGR2GRAY)
        ret,mask = cv2.threshold(grayImage,127,255,cv2.THRESH_BINARY)
        # lower = numpy.array([ 50, 0, 0])
        # upper = numpy.array([ 255, 255, 255])
        # mask = cv2.inRange(hsv,  lower, upper)
        # masked = cv2.bitwise_and(image, image, mask=mask)
        edges = cv2.Canny(mask, 150, 200)
        lines = cv2.HoughLines(edges,1,numpy.pi/180,50)

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

                # cv2.line(image,(x1, y1),(x2, y2),(0,0,255),2)

                # print k
                if (abs(k) < 3 and abs(k) > 0.3):
                    if k > 0:
                        leftLine = x1, y1, x2, y2, k
                    else:
                        rightLine = x1, y1, x2, y2, k
                        
                if abs(k) < 0.05:
                        frontLine = x1, y1, x2, y2, k

        if leftLine is not None:
            cv2.line(image,(leftLine[0], leftLine[1]),(leftLine[2], leftLine[3]),(0,0,255),2)
        if rightLine is not None:
            cv2.line(image,(rightLine[0], rightLine[1]),(rightLine[2], rightLine[3]),(0,255,255),2)
        if frontLine is not None:
            cv2.line(image,(frontLine[0], frontLine[1]),(frontLine[2], frontLine[3]),(0,255,0),2)



        special = False
        err = 0

        if frontLine is None or max(calcY(frontLine, 0), calcY(frontLine, w)) < h/3:

            # Normal case

            if leftLine is not None and rightLine is not None:

                left_yIntersect = calcY(leftLine, 0)
                right_yIntersect = calcY(rightLine, w)

                cv2.line(image,(leftLine[0], leftLine[1]),(leftLine[2], leftLine[3]),(0,0,255),2)
                cv2.line(image,(rightLine[0], rightLine[1]),(rightLine[2], rightLine[3]),(0,255,255),2)
                
                cv2.circle(image, (0, left_yIntersect), DOT_R, (0,0,0), -1)
                cv2.circle(image, (w, right_yIntersect), DOT_R, (0,0,0), -1)
                err = left_yIntersect - right_yIntersect # x - 320
                print("by left and right")
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
                err = -frontLine[4] * 1000
                print("by front")
            else:
                # Wall is close, need to turn 90 degree conner (maybe cross way) or turn 180 degree back

                special = True
                self.twist.linear.x = 0#0.1
                self.twist.angular.z = 0#STATIC_TURN_SPD
                self.cmd_vel_pub.publish(self.twist)

                if leftLine is not None and rightLine is not None:
                    
                    highest = min(calcY(frontLine, 0), calcY(frontLine, w))

                    lowest = max(calcY(frontLine, 0), calcY(frontLine, w))

                    leftCrossPoint = calcX(leftLine, lowest+9)


                    # Get 9*9 average color to recognize whether it is a cross road or dead end
                    # print highest, mask[highest][w/2]
                    average = 0
                    for i in range(highest-9, highest):
                        for j in range(w/2-4, w/2+5):
                            average += mask[i][j]
                    average /= 81

                    averageLeft = 0
                    for i in range(lowest, lowest + 9):
                        for j in range(leftCrossPoint-9, leftCrossPoint):
                            averageLeft += mask[i][j]
                    averageLeft /= 81

                    if averageLeft < 128:
                        turnArc(False, ARC_FORWARDING_SPD, 90)
                    elif average >= 128:
                        turnArc(False, 0, 180)
                    cv2.line(image,(leftLine[0], leftLine[1]),(leftLine[2], leftLine[3]),(0,0,255),2)
                    cv2.line(image,(rightLine[0], rightLine[1]),(rightLine[2], rightLine[3]),(0,255,255),2)
                else:
                    if leftLine is None and rightLine is None:
                        turnArc(False, ARC_FORWARDING_SPD, 90)
                    else:
                        if leftLine is not None: #right turn
                            turnArc(True, ARC_FORWARDING_SPD, 90)
                            cv2.line(image,(leftLine[0], leftLine[1]),(leftLine[2], leftLine[3]),(0,0,255),2)
                        if rightLine is not None:#left turn
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

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        
        
        # print("M00 %d %d" % (M['m00'], self.logcount))
        print self.degree
        print("%d err: %f; iTerm: %f; dTerm: %f; z: %f" % (self.logcount, err, iTerm, dTerm, self.twist.angular.z))
        





        # a = numpy.asarray(hsv[:,:,0], dtype=numpy.uint8)
        # print(a.shape)
        # print("\n")
        # numpy.savetxt("foo.csv", a, delimiter=",")

        cv2.imshow("image", image)
        # cv2.imshow("hsv", hsv)
        cv2.imshow("mask", mask)
        cv2.waitKey(3)

rospy.init_node('follower2')
follower = Follower()
rospy.spin()

