#!/usr/bin/env python
import math, time
import rospy, numpy
from geometry_msgs.msg import Twist
from cv_maze.srv import Pid
import actionlib
from cv_maze.msg import TurnAction, TurnGoal, LineData

#DRAW
DOT_R = 10

DISTANCE_TO_WALL = 100
HISTORY_SIZE = 10

STATIC_TURN_SPD = 0.58
FORWARDING_SPD = 0.19

cx = 0
cy = 0
err_history = []

class Follower:

        if frontLine is None or max(calcY(frontLine, 0), calcY(frontLine, w)) < h/4:

            # Normal case

            if leftLine is not None and rightLine is not None:
                err = float(right_yIntersect - left_yIntersect) # x - 320
            else:
                self.twist.linear.x = 0

                # No clue, spinning
                if leftLine is None and rightLine is None:
                    self.twist.angular.z = STATIC_TURN_SPD
                else:

                    # Turn sharply to the direcion that the line cannot be seen
                    if leftLine is not None:
                        self.twist.angular.z = -STATIC_TURN_SPD
                    if rightLine is not None:
                        self.twist.angular.z = STATIC_TURN_SPD

        else:

            # When see the wall ahead, use the line of wall 's k to balance the err value until the wall is too close
            if max(calcY(frontLine, 0), calcY(frontLine, w)) < h/2:
                err = frontLine[4] * 10000
            else:
                # Wall is close, need to turn 90 degree conner (maybe cross way) or turn 180 degree back
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
                else:
                    if leftLine is None and rightLine is None:
                        #turn90Arc(False)
                        pass
                    else:
                        if leftLine is not None:
                            turnArc(True, ARC_FORWARDING_SPD, 90)
                        if rightLine is not None:
                            turnArc(False, ARC_FORWARDING_SPD, 90)


line_info = None
logcount = 0

def line_callback(msg):
    global line_info
    line_info = msg

rospy.init_node('main')

line_sub = rospy.Subscriber('/line_detection', LineData, line_callback)

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()

action_client = actionlib.SimpleActionClient('turn', TurnAction)
action_client.wait_for_server()
turnGoal = TurnGoal()

rospy.wait_for_service('pid_server')
pid = rospy.ServiceProxy('pid_server', Pid)

print "maze solver started"

while True:

    leftLine = None
    rightLine = None
    frontLine = None
    leftIntersect = line_info.leftIntersect
    rightIntersect = line_info.rightIntersect
    h = line_info.imgH
    w = line_info.imgW

    if line_info.leftLine is not None and len(line_info.leftLine) != 0:
        leftLine = line_info.leftLine
    if line_info.rightLine is not None and len(line_info.rightLine) != 0:
        rightLine = line_info.rightLine
    if line_info.frontLine is not None and len(line_info.frontLine) != 0:
        frontLine = line_info.frontLine

    if frontLine is None or line_info.frontMidY < h/4:
        if leftLine is not None and rightLine is not None:
            twist.linear.x = FORWARDING_SPD
            err = float(rightIntersect - leftIntersect)
            twist.angular.z = pid(time.time(), err) / 500
        else:
            twist.linear.x = 0
            # No clue, turn left by default
            if leftLine is None and rightLine is None:
                twist.angular.z = STATIC_TURN_SPD
            else:
                # Turn sharply to the direcion that the line cannot be seen
                if leftLine is not None:
                    twist.angular.z = -STATIC_TURN_SPD
                if rightLine is not None:
                    twist.angular.z = STATIC_TURN_SPD
    else:
        # When see the wall ahead, use the line of wall 's k to keep go straight until the wall is too close
        if line_info.frontMidY < h/2:
            err = frontLine[4] * 10000
        else:
            # Wall is close, need to turn 90 degree conner (maybe cross way) or turn 180 degree back
            # TODO
            pass
    logcount += 1
    cmd_vel_pub.publish(twist)