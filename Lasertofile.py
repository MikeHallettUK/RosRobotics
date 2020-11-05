# read LDS01 laser scan data = 360 @ 5Hz - write sample to file with basic statistics

import rospy
from sensor_msgs.msg import LaserScan
import time
import numpy as np

timer = time.time()
count = 0
lastcount = 0
laser = np.array([], np.float32)
intensity = np.array([], np.float32)
Angle = np.arange(360)

def callback(msg):
    global timer, count, laser, intensity
    duration = time.time() - timer
    timer = time.time()
    count = count +1
    print("%d) Time %.3f secs, len = %d" % (count, duration, len(msg.ranges)))
    #print(msg.header.seq, msg.header.stamp, rospy.Time.now())
    laser = np.copy(msg.ranges)
    intensity = np.copy(msg.intensities)

def getbest(slice):                     # calcs stats from samples
    good = np.array([], np.float32)
    slice = np.delete(slice, np.where(slice == 0)[0])  # remove nul values
    num = len(slice)
    avg = 99.0
    sd = 99.0
    if num > 3:      # only use if +4 datums
        avg = np.average(slice)
        sd = np.std(slice)
        upp = avg + 3.5 * sd
        low = avg - 3.5 * sd
        for datum in slice:
            if datum > low and datum < upp:    # then good reading: else is outlier to ignore
                good = np.append(good, datum)
        num = len(good)
        if num > 3:  # only use if +4 datums
            avg = np.average(good)
            sd = np.std(good)
    return num, avg, sd

rospy.init_node('my_scan')
sub = rospy.Subscriber('/scan', LaserScan, callback)
r = rospy.Rate(20)

print("Starting")
while count < 11:
    if count > lastcount:   # wait until callback triggers... collect scan data.
        lastcount = count
        if count == 1:
            laserarray = laser
            intensityarray = intensity
        else:
            laserarray = np.vstack([laserarray, laser]) # stack 10 readings of 360 scans [360,10]
            intensityarray = np.vstack([intensityarray, intensity])
    else:
        r.sleep()

outfile = open("LaserData.csv", "w")  # write out a data sample for analysis to file.
outfile.write("Angle, Range, Intensity >>>  \n")
Angle.tofile(outfile, sep=',')
outfile.write('\n')
laserarray[0, :].tofile(outfile, sep=',')
outfile.write('\n')
intensityarray[0, :].tofile(outfile, sep=',')
outfile.write('\n')
outfile.close()

statfile = open("LaserStats.csv", "w")  # write out a data stats for analysis to file.
statfile.write("Angle, RangeN, RangeAv, RangeSD, IntN, IntAv, IntSD \n")
# Now calc best-avg for each set of 10, and write out one sample
for i in range(360):
    statfile.write(str(Angle[i]) + ',')
    num, avg, sd = getbest(laserarray[:, i])  # 360 sets of 3 stats: num, avg, sd
    statfile.write(str(num) + ',' + str(avg) + ',' + str(sd) + ',')
    num, avg, sd = getbest(intensityarray[:, i])
    statfile.write(str(num) +','+ str(avg) +','+ str(sd) + '\n')
statfile.close()
print("Closed files...  Finished")
# end of program ...