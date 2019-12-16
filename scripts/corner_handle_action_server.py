#!/usr/bin/env python
import rospy
import time
import actionlib
from cv_maze.msg import CornersAction, CornersResult, TurnAction, TurnGoal, ForwardAction, ForwardGoal, LineData

CHANCE_THRESHOLD = 0.3
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

'''

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
    if chanceLine > CHANCE_THRESHOLD:
        hasLeftLine = True
    chanceWall = float(hasWallCount1 + hasWallCount2) / float(imgCount1 + imgCount2)
    if chanceWall > 0.5:
        hasLeftWall = True

    # print chanceLine, chanceWall, hasLeftLine, hasLeftWall, hasFrontWall

    # Right
    _, hasLineCount1, imgCount1 = getLineStatisticsRes(-SCAN_RANGE, True)
    _, hasLineCount2, imgCount2 = getLineStatisticsRes(SCAN_RANGE, True)

    chanceLine = float(hasLineCount1 + hasLineCount2) / float(imgCount1 + imgCount2)
    if chanceLine > CHANCE_THRESHOLD:
        hasRightLine = True

    # print chanceLine, hasRightLine
    # print '\n'

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

    if hasFrontWall:
        if hasLeftLine and hasRightLine:
            turn(180)
            result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
            result.situationType = 0
            server.set_succeeded(result)
            return
        else:
            forward(0.35)
            if not hasLeftLine and not hasRightLine:
                turn(90)
                result.situationType = 1
            else:
                if not hasLeftLine:
                    turn(90)
                    result.situationType = 2
                if not hasRightLine:
                    turn(-90)
                    result.situationType = 3
    else:
        if hasLeftWall:
            forward(0.6)
            result.situationType = 4
        else:
            forward(0.35)
            turn(90)
            result.situationType = 5

    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    server.set_succeeded(result)
'''
    # # Wall is close, need to turn 90 degree corner (maybe cross way) or turn 180 degree back

    # if hasLeftLine and hasRightLine is not None:

    #     highest = min(calcY(frontLine, 0), calcY(frontLine, w))

    #     # Get 9*9 average color to recognize whether it is a cross road or dead end
    #     # print highest, mask[highest][w/2]
    #     average = 0
    #     for i in range(highest-9, highest):
    #         for j in range(w/2-4, w/2+5):
    #             average += mask[i][j]
    #     average /= 81

    #     if average < 128:
    #         turnArc(False, ARC_FORWARDING_SPD, 90)
    #     else:
    #         turnArc(False, 0, 180)
    # else:
    #     if hasLeftLine and hasRightLine:
    #         #turn90Arc(False)
    #         pass
    #     else:
    #         if hasLeftLine:
    #             turnArc(True, ARC_FORWARDING_SPD, 90)
    #         if hasRightLine:
    #             turnArc(False, ARC_FORWARDING_SPD, 90)

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
    if chanceLine > CHANCE_THRESHOLD:
        hasLeftLine = True
    chanceWall = float(hasWallCount1 + hasWallCount2) / float(imgCount1 + imgCount2)
    if chanceWall > 0.5:
        hasLeftWall = True

    # print chanceLine, chanceWall, hasLeftLine, hasLeftWall, hasFrontWall

    # Right
    

    # print chanceLine, hasRightLine
    # print '\n'

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
        print "[node: corner_handel_action_server] left turn case or left T shape road case"
        forward(0.41)
        turn(90)
        result.situationType = 2  # include case 1 and 2
    else:
        if not hasFrontWall:
            if hasLeftWall:
                print "[node: corner_handel_action_server] ahead T shape road case"
                forward(0.55)
                result.situationType = 4
            # else:
            #     print "[node: corner_handel_action_server] left T shape road case"
            #     forward(0.45)
            #     turn(90)
            #     result.situationType = 5
        else:
            _, hasLineCount1, imgCount1 = getLineStatisticsRes(-SCAN_RANGE, True)
            _, hasLineCount2, imgCount2 = getLineStatisticsRes(SCAN_RANGE, True)

            chanceLine = float(hasLineCount1 + hasLineCount2) / float(imgCount1 + imgCount2)
            if chanceLine > CHANCE_THRESHOLD:
                hasRightLine = True
            if hasRightLine:
                print "[node: corner_handel_action_server] dead end case"
                forward(0.35)
                turn(180)
                result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
                result.situationType = 0
                server.set_succeeded(result)
                return
            else:
                print "[node: corner_handel_action_server] right turn case"
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
