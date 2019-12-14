import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import actionlib
from cv_maze.msg import ForwardAction, ForwardGoal, ForwardResult, ForwardFeedback
import time

x = 0.0

inc_x = 0.0

def newOdom(msg):
    global x

    x = msg.pose.pose.position.x

def action_callback(goal):
    global inc_x
    global x

    start_time = time.time()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    inc_x = goal.goalIncre
    x_goal = x + inc_x

    speed = Twist()
    speed.linear.x = 0.1 * (inc_x / abs(inc))

    feedback = ForwardFeedback()

    while abs(x_goal - x) > 0.05:
        pub.publish(speed)
        feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        feedback.currentPose = x
        server.publish_feedback(feedback)
    
    speed.linear.x = 0;
    pub.publish(speed)
    result = ForwardResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.currentPose = x
    server.set_succeeded(result, "move forward successfully")

rospy.init_node("forward_action")
sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
server = actionlib.SimpleActionServer('forward', ForwardAction, action_callback, false)
server.start()
rospy.spin()





    