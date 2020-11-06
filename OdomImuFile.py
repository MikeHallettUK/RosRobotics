# read Odometry, cmd_vel and IMU: and write to file while /cmd_vel says to move

import rospy
from nav_msgs.msg import Odometry  # nav_msgs/Odometry.msg
from geometry_msgs.msg import Twist   # geometry_msgs/Twist.msg
from sensor_msgs.msg import Imu # sensor_msgs/Imu
from tf.transformations import euler_from_quaternion
import numpy as np

Lastseq = Oseq = 0
count = 0
Cmove = Cturn = 0.0
data = np.zeros(8)
dataarray = np.zeros(22)
imu = Imu()

def get_odom(msg):                  # at ~ 30 Hz
    global Oseq, count, data
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    move = msg.twist.twist.linear.x
    turn = msg.twist.twist.angular.z
    count = count +1
    data = np.copy((count, move, turn, yaw, x, y, msg.header.stamp.secs, msg.header.stamp.nsecs))
    Oseq = msg.header.seq  # make this last variable to update because main prog tests this to see if updated yet ....
    #print(data)

def get_vel(msg):                  # at ~ 5 Hz
    global Cmove, Cturn
    Cmove = msg.linear.x
    Cturn = msg.angular.z
    #print(Cmove, Cturn)

def imu_cb(msg):  # at ~ 190 Hz
    global imu
    imu = msg
    # print(imu)

def collect():
    global Cmove, Cturn, imu, data, dataarray, Oseq
    imudata = np.array([1.0] * 10, np.float32)
    Lastseq = int(Oseq)
    now = rospy.get_rostime()
    imudata[0] = imu.orientation.x
    imudata[1] = imu.orientation.y
    imudata[2] = imu.orientation.z
    imudata[3] = imu.orientation.w
    imudata[4] = imu.linear_acceleration.x
    imudata[5] = imu.linear_acceleration.y
    imudata[6] = imu.linear_acceleration.z
    imudata[7] = imu.angular_velocity.x
    imudata[8] = imu.angular_velocity.y
    imudata[9] = imu.angular_velocity.z
    dataslice = np.append(data, np.copy((Cmove, Cturn)))
    dataslice = np.append(dataslice, imudata)
    dataslice = np.append(dataslice, np.copy((now.secs, now.nsecs)))
    dataarray = np.vstack([dataarray, dataslice])
    return Lastseq

rospy.init_node('my_odom')
subO = rospy.Subscriber('/odom', Odometry, get_odom)    # nav_msgs/Odometry
cmd_vel = rospy.Subscriber('/cmd_vel', Twist, get_vel)  # geometry_msgs/Twist
sub_imu = rospy.Subscriber('/imu', Imu, imu_cb)         # sensor_msgs/Imu
r = rospy.Rate(200)

outfile = open("OdomData.csv", "w")  # write out data for analysis to file.
outfile.write("Count,move,turn,yaw,x,y,OdomSec,OdomNsec,Cmove,Cturn,Qx,Qy,Qz,Qw,AccX,AccY,AccZ,AngX,AngY,AngZ,NowSec,NowNsec \n")
print("Opened file...  starting")

while count < 11 and not rospy.is_shutdown():
    if int(Oseq) > Lastseq:   # wait until Odom callback triggers... collect data. even if stationary
        Lastseq = collect()
print("Got first ten %d " % (count))
print("Waiting to move ...")

moving = False
moved = rospy.get_rostime()
while not moving and not rospy.is_shutdown():
    if abs(Cmove) < 0.001 and abs(Cturn) < 0.001:
        count = 0    # skip until moving...
        r.sleep()
    else:
        moving = True
        moved = rospy.get_rostime()
while moving and not rospy.is_shutdown():
        if int(Oseq) > Lastseq:   # wait until callback triggers... collect data.
            Lastseq = collect()
            if abs(Cmove) < 0.001 and abs(Cturn) < 0.001:
                duration = rospy.get_rostime() - moved          # stopped... so see for how long...
                if duration > rospy.Duration.from_sec(2.0):      # stopped for more than 2 second ...
                    moving = False
            else:
                moved = rospy.get_rostime()                 # still moving at this time...
print("Got moving batch %d " % (count))

count = 0
while count < 11 and not rospy.is_shutdown():
    if int(Oseq) > Lastseq:         # wait until callback triggers... collect data.
        Lastseq = collect()
print("Got last ten %d " % (count))

for i in range(len(dataarray)):
    slice = dataarray[i,:]
    slice.tofile(outfile, sep=',')
    outfile.write('\n')
outfile.close()
print("Closed file...  Finished")
# end of program ...
