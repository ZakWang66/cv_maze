import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
import time

numpy.set_printoptions(threshold=numpy.inf)

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback)

    def image_callback(self, msg):

        def calcY(line, x):
            return int((-line[4])*(x - line[0]) + line[1])
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
        lower = numpy.array([ 0, 0, 0])
        upper = numpy.array([ 255, 255, 255])
        mask = cv2.inRange(hsv,  lower, upper)
        masked = cv2.bitwise_and(image, image, mask=mask)
        edges = cv2.Canny(masked, 70, 100)
        lines = cv2.HoughLines(edges,1,numpy.pi/180,50)

        leftLine = None
        rightLine = None
        frontLine = None
        minK = 0
        maxK = 0
        horizentalK = 0.05
        print(len(lines))
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
                    if k > 0 and k > maxK:
                        maxK = k
                        leftLine = x1, y1, x2, y2, k
                    elif k < 0 and k < minK:
                        minK = k
                        rightLine = x1, y1, x2, y2, k
                        
                if abs(k) < horizentalK:
                    horizentalK = abs(k)
                    frontLine = x1, y1, x2, y2, k

        if leftLine is not None:
            cv2.line(image,(leftLine[0], leftLine[1]),(leftLine[2], leftLine[3]),(0,0,255),2)
            
        if rightLine is not None:
            cv2.line(image,(rightLine[0], rightLine[1]),(rightLine[2], rightLine[3]),(0,255,255),2)
        if frontLine is not None:
            cv2.line(image,(frontLine[0], frontLine[1]),(frontLine[2], frontLine[3]),(0,255,0),2)
        left_yIntersect = calcY(leftLine, 0)
        right_yIntersect = calcY(rightLine, w)
        err = left_yIntersect - right_yIntersect
        print("front line is :" + str(max(calcY(frontLine, 0), calcY(frontLine, w))))
        print("err is :" + str(err))

        # a = numpy.asarray(hsv[:,:,0], dtype=numpy.uint8)
        # print(a.shape)
        # print("\n")
        # numpy.savetxt("foo.csv", a, delimiter=",")

        cv2.imshow("image", image)
        cv2.imshow("hsv", hsv)
        cv2.imshow("masked", masked)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()

