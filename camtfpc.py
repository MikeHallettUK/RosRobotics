#!/usr/bin/env python3
# tf_pc_cam subscribes to PC2 from RealSense camera = and transforms it to frame odom /points2

import rospy
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import PointCloud2, do_transform_cloud      # to support PointCloud2

rospy.init_node("tf_pc_cam")
tf_pub = rospy.Publisher("points2", PointCloud2, queue_size=10)
tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
tf_listener = tf2_ros.TransformListener(tf_buffer)
rospy.sleep(0.2)    # let tf_buffer fill up a bit ...

def pc_cb(msg):
    cantran = tf_buffer.can_transform("odom", msg.header.frame_id, 
                                      msg.header.stamp,
                                      rospy.Duration(0.1))
    if cantran:
        trans = tf_buffer.lookup_transform("odom", msg.header.frame_id,
                                           msg.header.stamp,
                                           rospy.Duration(0.1))
        cloud_out = do_transform_cloud(msg, trans)
        tf_pub.publish(cloud_out)

print("Starting do_transform_cloud from /camera/depth/color/points v1")
rospy.Subscriber("/camera/depth/color/points", PointCloud2, pc_cb, queue_size=1, buff_size=2**24)

rospy.spin()
