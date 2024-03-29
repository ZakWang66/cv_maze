#!/usr/bin/env python
import math
import time
import rospy
import actionlib
from cv_maze.msg import TurnAction, TurnResult, TurnFeedback
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SPEED = 0.20

degree = 0
lastDegree = 0
rounds = 0


def odom_callback(msg):
    global degree
    degree1 = math.asin(msg.pose.pose.orientation.z) / math.pi * 360
    degree2 = math.acos(msg.pose.pose.orientation.w) / math.pi * 360
    if degree1 >= 0:
        degree = degree2
    else:
        degree = 360 - degree2


def action_callback(goal):
    global degree
    global lastDegree
    global hasTurned

    print "[turn_action_server] star to turn", goal.degree_to_turn

    start_time = time.time()
    lastDegree = degree

    hasTurned = 0
    degreeToTurn = abs(goal.degree_to_turn)

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = 0
    speed = SPEED
    if goal.degree_to_turn < 0:
        speed = -SPEED
    twist.angular.z = speed
    feedback = TurnFeedback()

    rate = rospy.Rate(1000)
    while hasTurned < degreeToTurn:

        accumulate = abs(degree - lastDegree)

        if lastDegree > 350 and degree < 10:
            accumulate = 360 - lastDegree + degree

        if degree > 350 and lastDegree < 10:
            accumulate = 360 - degree + lastDegree

        hasTurned += accumulate
        lastDegree = degree

        twist.angular.z = speed * ((degreeToTurn - hasTurned) / degreeToTurn + 0.2)
        twist.linear.x = 0
        cmd_vel_pub.publish(twist)
        feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        server.publish_feedback(feedback)
        rate.sleep()

    twist.angular.z = 0
    cmd_vel_pub.publish(twist)
    print "[turn_action_server] done"
    result = TurnResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    server.set_succeeded(result, "Turning completed successfully")


rospy.init_node('turn_action_server')
sub = rospy.Subscriber('/odom', Odometry, odom_callback)
server = actionlib.SimpleActionServer('turn', TurnAction, action_callback, False)
server.start()
print '[turn_action_server] Listening...'
rospy.spin()
