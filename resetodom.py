# read Odometry and reset to zero

import rospy
from nav_msgs.msg import Odometry  # nav_msgs/Odometry.msg
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
from math import radians, cos, tan, sin, sqrt, atan, pi, asin, exp
import numpy as np

rospy.init_node('resetter')
count = 0
start = rospy.get_rostime()
data = np.zeros(7)

def QtoYaw(orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    if yaw<0:
        yaw = 2*pi + yaw    # goes from 0 to 2*pi, anti-clockwise.  jumps at 0:2pi
    return yaw

def get_odom(msg):                  # at ~ 30 Hz
    global start, data
    yaw = QtoYaw(msg.pose.pose.orientation)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    move = msg.twist.twist.linear.x
    drift = msg.twist.twist.linear.y
    turn = msg.twist.twist.angular.z
    now = msg.header.stamp - start
    seconds = now.to_sec()          # time in floating point
    data = np.copy((move, turn, drift, yaw, x, y, seconds))

subO = rospy.Subscriber('/odom', Odometry, get_odom)    # nav_msgs/Odometry
reset_odom = rospy.Publisher('/reset', Empty, queue_size=1)  # rostopic pub /reset std_msgs/Empty "{}"

print("Waiting for odom call back to start...")
while data[6] == 0:         # wait for odom call back to start...
    rospy.sleep(0.1)

while count < 5:
    datacopy = np.copy(data)
    yaw = datacopy[3]
    if yaw > pi:
        yaw = yaw - 2*pi
    print("At %.3f secs, x %.4f , y %.4f : yaw %.4f, drift %.6f" %
          (datacopy[6], datacopy[4], datacopy[5], yaw, datacopy[2]))  # (move, turn, drift, yaw, x, y, seconds)
    if abs(yaw) < 0.005:            # eg. yaw is zeroed to better than +/- 0.3 degrees
        rospy.loginfo("Zero")
        break
    if count < 4:
        reset_odom.publish(Empty())
        print("reset")
        rospy.sleep(4.0)            # it takes ~ 3 seconds to reset ....
    count = count + 1

# end of program ...