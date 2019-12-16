#! /usr/bin/env python
import rospy
import time
import actionlib
from geometry_msgs.msg import Twist, Pose
from ros_summary_project.msg import ActionServerAction, ActionServerGoal, ActionServerResult, ActionServerFeedback
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

current_pose = Pose()
theta = 0.0 
def callback(msg): 
    global current_pose
    current_pose = msg.pose.pose
    global theta
    theta = get_radian(current_pose)

def get_radian(current_pose):
    (roll, pitch, theta_radian) = euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
    return theta_radian


#in this method we will use euler_from_quaternion function to get the radiance
def get_target_radiance(a,b,c):
    #get the target radian
    target_angle_radian = (c.angular.z + get_radian(b)) % (2 * math.pi)
    if(target_angle_radian > math.pi):
        target_angle_radian = target_angle_radian - 2 *math.pi 
    return target_angle_radian

    
    current_radian = get_radian(a)

    if(abs(current_radian - target_angle_radian) > 0.1):
        return False
    else:
        return True

def get_twist(current_pose, start_pose):
    c = Twist()
    c.angular.z = get_radian(current_pose) - get_radian(start_pose)
    return c

def do_action(goal):
    global current_pose

    feedback = ActionServerFeedback()
    feedback.angle_changed = Twist()
    radiance_to_change = goal.degree_to_turn / 180 * math.pi

    speed = Twist()
    if radiance_to_change > 0:
        speed.angular.z = 0.1
    else:
        speed.angular.z = -0.1
    time_duration = radiance_to_change / speed.angular.z

    start_time = time.time()
    start_pose = current_pose
    update_count = 0
    
    pub.publish(speed)

    target_angle_radian = get_target_radiance(current_pose, start_pose, goal.angle_to_change)
    
    while (time.time() - start_time) < time_duration:#not is_get_goal(current_pose, start_pose, goal.angle_to_change):
        if server.is_preempt_requested():
            result = ActionServerResult()
            result.angle_changed = get_twist(current_pose, start_pose)
            result.updates_sent = update_count
            server.set_preempted(result, "ActionServer preempted")

        
        feedback.angle_changed.angular.z = speed.angular.z * (time.time() - start_time)#get_twist(current_pose, start_pose)

        server.publish_feedback(feedback)
        update_count += 1
        print "current radian is: " + str(get_radian(current_pose))
        print "start radian is: " + str(get_radian(start_pose))
        print "goal_radiance: " + str(goal.angle_to_change.angular.z)
        print "target radian is" + str(target_angle_radian)
        time.sleep(1.0)
    
    while abs(get_radian(current_pose) - target_angle_radian) > 0.1:
        if server.is_preempt_requested():
            result = ActionServerResult()
            result.angle_changed = get_twist(current_pose, start_pose)
            result.updates_sent = update_count
            server.set_preempted(result, "ActionServer preempted")

        
        feedback.angle_changed.angular.z = speed.angular.z * (time.time() - start_time)#get_twist(current_pose, start_pose)

        server.publish_feedback(feedback)
        update_count += 1
        print "current radian is: " + str(get_radian(current_pose))
        print "start radian is: " + str(get_radian(start_pose))
        print "goal_radiance: " + str(goal.angle_to_change.angular.z)
        print "target radian is" + str(target_angle_radian)
        time.sleep(1.0)
    
    speed.angular.z = 0.0
    print 'stop!!!!' + str(speed.angular.z)
    pub.publish(speed)

    result = result = ActionServerResult()
    result.angle_changed.angular.z = 0.1 * (time.time() - start_time)
    result.updates_sent = update_count
    server.set_succeeded(result, "ActionServer completed successfully")


rospy.init_node('ActionServer')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
server = actionlib.SimpleActionServer('actionServer', ActionServerAction, do_action, False)
odom_sub = rospy.Subscriber('odom', Odometry, callback)
server.start()
rospy.spin()