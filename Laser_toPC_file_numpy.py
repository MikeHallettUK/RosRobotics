# Simple projection: does not fix scan-lag effect for moving robot
# The method projectLaser() projects a single laser scan from a linear array into a sensor_msgs/PointCloud2.
# The generated cloud will be in the same frame as the original laser scan.

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_geometry.laser_geometry import LaserProjection
import numpy as np
import ros_numpy
import time

count = 0
Lastseq = seq = 0
pointarray = np.zeros((1, 3))

rospy.init_node("laserscan_to_pointcloud")
pointcloud = LaserProjection()
timer = time.time()

def Printime(text):
    global timer
    duration = time.time() - timer
    print(text + " took %.8f seconds" % (duration))
    timer = time.time()

def scan_cb(msg):
    global count, pointarray, seq
    # convert the message of type LaserScan to a PointCloud2 into Numpy format
    pc2_msg = pointcloud.projectLaser(msg)    #  = x, y, z, "intensity",  "index"
    pc = ros_numpy.numpify(pc2_msg)
    pointarray = np.zeros((pc.shape[0], 3))
    pointarray[:, 0] = pc['index']
    pointarray[:, 1] = pc['x']
    pointarray[:, 2] = pc['y']
    count = count + 1
    seq = msg.header.seq

rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
outfile = open("PointData.csv", "w")  # write out data for analysis to file.
outfile.write("Angle,x,y \n")
print("Starting")
while count < 2:
    if int(seq) > Lastseq:   # wait until callback triggers... collect data
        pointout = np.copy(pointarray)        # take copy of volatile data
        Lastseq = seq
        for i in range(len(pointout)):
            slice = pointout[i, :]
            slice.tofile(outfile, sep=',')
            outfile.write('\n')

outfile.close()
Printime("done")
print("Closed file...  Finished " + str(count))


