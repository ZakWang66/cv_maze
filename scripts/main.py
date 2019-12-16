#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Twist
from cv_maze.srv import Pid
import actionlib
from cv_maze.msg import CornersAction, CornersGoal, LineData

FORWARDING_SPD = 0.06

line_info = None
inCorner = False
line_updated = False


def line_callback(msg):
    global line_info
    global line_updated
    line_info = msg
    line_updated = True


def action_done_cb(status, result):
    global inCorner
    inCorner = False
    print "[main] corner completed, type code: ", result.situationType


rospy.init_node('main')

line_sub = rospy.Subscriber('/line_detection', LineData, line_callback)

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()

rospy.wait_for_service('pid_server')
pid = rospy.ServiceProxy('pid_server', Pid)

action_client = actionlib.SimpleActionClient('corners', CornersAction)
action_client.wait_for_server()

print "[main] maze solver started"

rate = rospy.Rate(1000)

# To determin the situation of whether there is a front wall
hasFrontWallCount = 0
imgCount = 0

while not rospy.is_shutdown():

    leftLine = None
    rightLine = None
    frontLine = None

    if line_info is None:
        continue

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

    if leftLine is not None and rightLine is not None:
        err = float(rightIntersect - leftIntersect)
    else:
        if leftLine is None and rightLine is None:
            if frontLine is not None:
                err = frontLine[4] * 10
        else:
            dummyLineIntersect = h - 20
            if leftLine is not None:
                err = float(dummyLineIntersect - leftIntersect)
            if rightLine is not None:
                err = float(rightIntersect - dummyLineIntersect)

    twist.linear.x = FORWARDING_SPD
    if frontLine is not None and line_info.frontMidY > 2 * h / 5:
        twist.linear.x = FORWARDING_SPD * (float(h / 2 - line_info.frontMidY) / float(h / 2) + 0.2)
        if line_updated:
            line_updated = False
            imgCount += 1
            if line_info.wallFront:
                hasFrontWallCount += 1

    twist.angular.z = pid(time.time(), err, False).err_pid
    cmd_vel_pub.publish(twist)

    if frontLine is not None and line_info.frontMidY > h / 2:
        # Wall is close, call action to solve a corner case
        cGoal = CornersGoal()
        twist.linear.x = 0
        twist.angular.z = 0
        cmd_vel_pub.publish(twist)

        if imgCount != 0:
            chance = float(hasFrontWallCount) / float(imgCount)
            cGoal.hasFrontWall = (chance > 0.5)
            hasFrontWallCount = 0
            imgCount = 0
        else:
            cGoal.hasFrontWall = line_info.wallFront

        action_client.send_goal(cGoal, done_cb=action_done_cb)
        inCorner = True
        print "[main] start a corner case"
        while inCorner:
            rate.sleep()

    rate.sleep()
