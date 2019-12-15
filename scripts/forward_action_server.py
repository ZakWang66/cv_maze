#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
import actionlib
from cv_maze.msg import ForwardAction, ForwardResult

SPEED = 0.06


def action_callback(goal):

    start_time = time.time()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

    distance = goal.goalIncre

    speed = Twist()
    speed.angular.z = 0
    speed.linear.x = SPEED
    if distance < 0:
        speed.linear.x = -speed.linear.x

    pub.publish(speed)

    rate = rospy.Rate(SPEED / abs(distance) * 100)

    i = 0
    while i < 100:
        pub.publish(speed)
        rate.sleep()
        i += 1

    speed.linear.x = 0
    pub.publish(speed)
    result = ForwardResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    server.set_succeeded(result, "move forward successfully")


rospy.init_node("forward_action_server")
server = actionlib.SimpleActionServer('forward', ForwardAction, action_callback, False)
server.start()
print '[forward_action_server] Listening...'
rospy.spin()
