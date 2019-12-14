import rospy
import actionlib
from cv_maze.msg import ConnerAction, TurnAction, TurnGoal, ForwardAction, ForwardGoal, LineData

line_info = None


def line_callback(msg):
    global line_info
    line_info = msg


def action_callback(goal):
    

    # Wall is close, need to turn 90 degree conner (maybe cross way) or turn 180 degree back

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


rospy.init_node('conner_handle_action_server')

line_sub = rospy.Subscriber('/line_detection', LineData, line_callback)

action_client = actionlib.SimpleActionClient('turn', TurnAction)
action_client.wait_for_server()
goal = TurnGoal()

action_client = actionlib.SimpleActionClient('forward', ForwardAction)
action_client.wait_for_server()
goal = ForwardGoal()

server = actionlib.SimpleActionServer('conner', ConnerAction, action_callback, False)
server.start()
print 'Listening...'

rospy.spin()
