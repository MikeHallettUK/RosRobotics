# to communicate thru ActionLib with MoveBase

import rospy
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from nav_msgs.msg import Odometry                                 # nav_msgs/Odometry.msg
from geometry_msgs.msg import PoseStamped
from math import radians, pi, sqrt, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler

routes = [[0.5,0.0,pi/2.0],[0.5,0.5,pi],[0.0,0.5,3.0*pi/2.0],[0.0,0.0,0.0]]  # go around in a square... anti-cl
rospy.init_node('movebase_client')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
Feedback = MoveBaseFeedback()   # ... = Feedback.base_position. header, pose. position, orientation

def QtoYaw(orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    if yaw < 0:
        yaw = 2 * pi + yaw  # goes from 0 to 2*pi, anti-clockwise.  jumps at 0:2pi
    return yaw

def get_odom(msg):                               # CallBack: at ~ 30 Hz nav.message ...
    global odomodom
    yaw = QtoYaw(msg.pose.pose.orientation)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    odomodom = np.copy((x, y, yaw))         # un-transformed odometry pose

def active_cb():
    print("Goal pose is now being processed by the MoveBase Action Server...")

def feedback_cb(feedback):
    global Feedback         # = geometry_msgs/PoseStamped = base_position. header, pose. position, orientation
    Feedback = feedback
    #print("Feedback for goal: " + str(feedback))

def done_cb(status, result):        # result = null ""
    #print("DONE with status: " + str(status) + "  " + str(result))
    if status == 3:
        print("Goal pose reached: SUCCESS")
    if status == 4:
        print("Goal was aborted by the Action Server.")
    if status == 5:
        print("Goal pose has been rejected by the Action Server.")
    if status == 8:
        print("Goal successfully cancelled.")

subodom = rospy.Subscriber('/odom', Odometry, get_odom, queue_size=1)    # nav_msgs/Odometry
server_up = client.wait_for_server(rospy.Duration(5.0))
goal = MoveBaseGoal()                # header; actionlib_msgs/GoalID goal_id. stamp, id; MoveBaseGoal goal.target_pose
goal.target_pose.header.frame_id = "map"
startbase = np.copy(odomodom)         # un-transformed odometry pose: x,y, yaw
print("Got initial pose in odom frame: ", startbase)
if not server_up:
    print("Timed out waiting for MoveBase server_up.")
else:
    for route in routes:
        print(" >>>>>>>  Going to : ", route[0], route[1], route[2])
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = route[0]
        goal.target_pose.pose.position.y = route[1]
        q = quaternion_from_euler(0, 0, route[2])      # yaw at destination = route[2]
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        client.send_goal(goal, done_cb, active_cb, feedback_cb)     # the callbacks are not mandated: but can be useful
        done = client.wait_for_result(rospy.Duration.from_sec(20.0))
        # returns True when done: Blocks until done: or times out
        if done:
            #print(client.get_state())  # States: 0 to 9 status: same as def done_cb(status, result):
            x = Feedback.base_position.pose.position.x
            y = Feedback.base_position.pose.position.y
            yaw = QtoYaw(Feedback.base_position.pose.orientation)
            print(">>> In map frame: Got to x %.3f , y %.3f , yaw %.3f" % (x, y, yaw))
        else:
            print("Failed to reach goal in time...")

odombase = np.copy(odomodom)         # un-transformed odometry pose: x,y, yaw
print("Got final pose in odom frame: ", odombase)
Difyaw = 180.0*(odombase[2] - startbase[2])/pi  - 360.0
Difdist = sqrt((odombase[0] - startbase[0])**2 + (odombase[1] - startbase[1])**2)
print("Change in odom odom Yaw = %.1f degrees, and in Dist = %.3f metres " % (Difyaw, Difdist))
print("TurtleBot stopped.")
# end of program ...