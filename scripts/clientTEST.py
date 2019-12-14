#!/usr/bin/env python
import time
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
import actionlib
from cv_maze.msg import TurnAction, TurnGoal, TurnResult, TurnFeedback, ForwardAction, ForwardGoal, LineData
from cv_maze.srv import Pid

Turning = False


def action_done_cb(status, result):
    global Turning
    global startTime
    print "Time Ellapse1: ", result.time_elapsed.to_sec()
    # print "Time Ellapse2: ", time.time() - startTime
    print "current pos: ", result.currentPose
    print status
    Turning = False


rospy.init_node('client')
# rospy.wait_for_service('pid_server')
# pid = rospy.ServiceProxy('pid_server', Pid)

# for i in range(20):
#     res = pid(time.time(), i, False)
#     print res


forward_action_client = actionlib.SimpleActionClient('forward', ForwardAction)
forward_action_client.wait_for_server()
forward_goal = ForwardGoal()
forward_goal.goalIncre = input("go how much :")

forward_action_client.send_goal(forward_goal, done_cb=action_done_cb)
rospy.spin()

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
