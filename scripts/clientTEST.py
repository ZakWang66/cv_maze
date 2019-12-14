#!/usr/bin/env python
import time
import rospy, cv2, cv_bridge, numpy
from cv_maze.srv import FindLines, FindLinesResponse
from sensor_msgs.msg import Image
import actionlib
from ros_summary_project.msg import TurnAction, TurnGoal, TurnResult, TurnFeedback
from cv_maze.srv import Pid

Turning = False

def action_done_cb(status, result):
    global Turning
    global startTime
    print "Time Ellapse1: ", result.time_elapsed.to_sec()
    print "Time Ellapse2: ", time.time() - startTime
    print status
    Turning = False

rospy.init_node('client')
rospy.wait_for_service('pid_server')
pid = rospy.ServiceProxy('pid_server', Pid)

for i in range(20):
    res = pid(time.time(), i)
    print res


# action_client = actionlib.SimpleActionClient('turn', TurnAction)
# action_client.wait_for_server()
# goal = TurnGoal()


# startTime = time.time()
# goal.degree_to_turn = 20
# Turning = True
# action_client.send_goal(goal, done_cb = action_done_cb)
# while Turning:
#     pass

# startTime = time.time()
# goal.degree_to_turn = -40
# Turning = True
# action_client.send_goal(goal, done_cb = action_done_cb)
# while Turning:
#     pass

# startTime = time.time()
# goal.degree_to_turn = 20
# Turning = True
# action_client.send_goal(goal, done_cb = action_done_cb)
# while Turning:
#     pass

# print "done"
