#!/usr/bin/env python3

# RosViewer.py = node that listens to a ROS image message topic,
#   and displays the image using OpenCV.

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_viewer:               #   "/camera/color/image_raw"  or   "/camera/color/video"
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.ros_cb, queue_size=1, buff_size=2 ** 24)

  def ros_cb(self,msg):
    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")   # in msg.data as "rgb8" but is "bgr8" from RS camera ??
    cv2.imshow("Ros video", cv_image)
    key = cv2.waitKey(10)                # in milliseconds
    if key == 113:                        # 113 is the letter 'q'
      cv2.destroyAllWindows()
      rospy.signal_shutdown("Quitting")

print("Starting Ros video image_viewer v1.2 ; press q to quit in video-window.")
rospy.init_node('image_viewer', anonymous=True)
iv = image_viewer()
rospy.spin()
print("Finished")
