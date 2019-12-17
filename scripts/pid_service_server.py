#!/usr/bin/env python
import rospy
from collections import deque
from cv_maze.srv import Pid, PidResponse

PP = 0.0005
II = 0.00001
DD = 0.000001
HISTORY_SIZE = 10

timeStamp_history = deque(maxlen=HISTORY_SIZE)
err_history = deque(maxlen=HISTORY_SIZE)


def callback(request):
    global timeStamp_history
    global err_history

    if request.restart:
        timeStamp_history = deque(maxlen=HISTORY_SIZE)
        err_history = deque(maxlen=HISTORY_SIZE)
        return PidResponse(0)

    timeStamp = request.timeStamp
    err = request.err
    timeStamp_history.appendleft(timeStamp)
    err_history.appendleft(err)

    iTerm = 0
    dTerm = 0
    if len(err_history) >= 2:
        dTerm = (err_history[0] - err_history[1]) * (timeStamp_history[0] - timeStamp_history[1])
        for i in range(len(err_history) - 1):
            iTerm += (err_history[i] + err_history[i + 1]) * (timeStamp_history[i] - timeStamp_history[i + 1]) / 2
        iTerm /= len(err_history) - 1
    pid = PP * err + II * iTerm + DD * dTerm
    # print '[pid_server]', err, iTerm, dTerm
    return PidResponse(pid)


rospy.init_node('pid_server')
service = rospy.Service('pid_server', Pid, callback)
print '[pid_server] started...'
rospy.spin()
