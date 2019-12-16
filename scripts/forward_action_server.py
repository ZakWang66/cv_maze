#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
import actionlib
from cv_maze.msg import ForwardAction, ForwardResult

SPEED = 0.04


def action_callback(goal):

    print '[forward_action_server] start to go forward', goal.goalIncres
    start_time = time.time()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    distance = goal.goalIncre

    twist = Twist()
    twist.angular.z = 0
    twist.linear.x = SPEED
    if distance < 0:
        twist.linear.x = -twist.linear.x

    pub.publish(twist)

    rate = rospy.Rate(SPEED / abs(distance) * 100)
    i = 0
    while i < 100:
        pub.publish(twist)
        rate.sleep()
        i += 1

    twist.linear.x = 0
    pub.publish(twist)
    print '[forward_action_server] done'
    result = ForwardResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    server.set_succeeded(result, "move forward successfully")


rospy.init_node("forward_action_server")
server = actionlib.SimpleActionServer('forward', ForwardAction, action_callback, False)
server.start()
print '[forward_action_server] Listening...'
rospy.spin()
