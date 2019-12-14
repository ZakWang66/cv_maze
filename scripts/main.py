#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Twist
from cv_maze.srv import Pid
import actionlib
from cv_maze.msg import ConnerAction, ConnerGoal, LineData

# DRAW
DOT_R = 10

DISTANCE_TO_WALL = 100
HISTORY_SIZE = 10

STATIC_TURN_SPD = 0.58
FORWARDING_SPD = 0.19

line_info = None
logcount = 0
inConner = False


def line_callback(msg):
    global line_info
    line_info = msg


def action_done_cb(status, result):
    global inConner
    inConner = False


rospy.init_node('main')

line_sub = rospy.Subscriber('/line_detection', LineData, line_callback)

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()

rospy.wait_for_service('pid_server')
pid = rospy.ServiceProxy('pid_server', Pid)

action_client = actionlib.SimpleActionClient('conner', ConnerAction)
action_client.wait_for_server()

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
            twist.angular.z = pid(time.time(), err, False) / 500
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
            action_client.send_goal(ConnerGoal(), done_cb=action_done_cb)
            inConner = True
            while inConner:
                pass

    logcount += 1
    cmd_vel_pub.publish(twist)
