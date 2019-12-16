#!/usr/bin/env python
import rospy
import time
import actionlib
from cv_maze.msg import CornersAction, CornersResult, TurnAction, TurnGoal, ForwardAction, ForwardGoal, LineData

CHANCE_LINE_THRESHOLD = 0.3
SCAN_RANGE = 20

line_info = None
Moving = False
line_updated = False


def action_done_cb(status, result):
    global Moving
    Moving = False


def line_callback(msg):
    global line_info
    global line_updated
    line_info = msg
    line_updated = True


def action_callback(goal):

    start_time = time.time()
    rate = rospy.Rate(1000)

    def getLineStatisticsRes(scan_range, side):
        global Moving
        global line_info
        global line_updated

        turn_goal.degree_to_turn = scan_range
        turn_action_client.send_goal(turn_goal, done_cb=action_done_cb)

        Moving = True
        hasLineCount = 0
        hasWallCount = 0
        imgCount = 0
        while Moving:
            if line_updated:
                line_updated = False
                imgCount += 1
                line = line_info.leftLine
                if side:
                    line = line_info.rightLine
                if line is not None and len(line) != 0:
                    hasLineCount += 1
                    if not side:
                        if line_info.wallLeft:
                            hasWallCount += 1

            rate.sleep()
        return hasWallCount, hasLineCount, imgCount

    turn_goal = TurnGoal()
    forward_goal = ForwardGoal()

    hasLeftLine = False
    hasLeftWall = False
    hasRightLine = False
    hasFrontWall = goal.hasFrontWall

    # Left
    hasWallCount1, hasLineCount1, imgCount1 = getLineStatisticsRes(SCAN_RANGE, False)
    hasWallCount2, hasLineCount2, imgCount2 = getLineStatisticsRes(-SCAN_RANGE, False)

    chanceLine = float(hasLineCount1 + hasLineCount2) / float(imgCount1 + imgCount2)
    if chanceLine > CHANCE_LINE_THRESHOLD:
        hasLeftLine = True
    chanceWall = float(hasWallCount1 + hasWallCount2) / float(imgCount1 + imgCount2)
    if chanceWall > 0.5:
        hasLeftWall = True

    def forward(distance):
        global Moving
        forward_goal.goalIncre = distance
        forward_action_client.send_goal(forward_goal, done_cb=action_done_cb)
        Moving = True
        while Moving:
            rate.sleep()

    def turn(degree):
        global Moving
        turn_goal.degree_to_turn = degree
        turn_action_client.send_goal(turn_goal, done_cb=action_done_cb)
        Moving = True
        while Moving:
            rate.sleep()

    result = CornersResult()

    if not hasLeftLine or not hasLeftWall:
        print "[corner_handler] left turn case or left T shape road case"
        forward(0.41)
        turn(90)
        result.situationType = 0
    else:
        if not hasFrontWall:
            if hasLeftWall:
                print "[corner_handler] go ahead T shape road case"
                forward(0.55)
                result.situationType = 1
        else:
            _, hasLineCount1, imgCount1 = getLineStatisticsRes(-SCAN_RANGE, True)
            _, hasLineCount2, imgCount2 = getLineStatisticsRes(SCAN_RANGE, True)

            chanceLine = float(hasLineCount1 + hasLineCount2) / float(imgCount1 + imgCount2)
            if chanceLine > CHANCE_LINE_THRESHOLD:
                hasRightLine = True
            if hasRightLine:
                print "[corner_handler] dead end case"
                forward(0.35)
                turn(180)
                result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
                result.situationType = 2
                server.set_succeeded(result)
                return
            else:
                print "[corner_handler] right turn case"
                forward(0.41)
                turn(-90)
                result.situationType = 3

    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    server.set_succeeded(result)


rospy.init_node('corner_handle_action_server')

line_sub = rospy.Subscriber('/line_detection', LineData, line_callback)

turn_action_client = actionlib.SimpleActionClient('turn', TurnAction)
turn_action_client.wait_for_server()

forward_action_client = actionlib.SimpleActionClient('forward', ForwardAction)
forward_action_client.wait_for_server()

server = actionlib.SimpleActionServer('corners', CornersAction, action_callback, False)
server.start()
print '[corner_handler] Listening...'

rospy.spin()
