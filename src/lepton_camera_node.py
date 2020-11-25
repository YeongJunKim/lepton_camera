#! /usr/bin/env python2

import cv2
import numpy as np
import rospy
import sys
import std_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs
from cv_bridge.boost.cv_bridge_boost import getCvType



# class OpenCvCapture(object):
#     """
#     Encapsulate state for capture from Pure Thermal 1 with OpenCV
#     """

#     def __init__(self):
#         # capture from the LAST camera in the system
#         # presumably, if the system has a built-in webcam it will be the first
#         for i in reversed(range(10)):
#             print "Testing for presense of camera #{0}...".format(i)
#             cv2_cap = cv2.VideoCapture(i)
#             if cv2_cap.isOpened():
#                 break

#         if not cv2_cap.isOpened():
#             print "Camera not found!"
#             exit(1)

#         self.cv2_cap = cv2_cap

#     def show_video(self):
#         """
#         Run loop for cv2 capture from lepton
#         """

#         cv2.namedWindow("lepton", cv2.WINDOW_NORMAL)
#         print "Running, ESC or Ctrl-c to exit..."
#         while True:
#             ret, img = self.cv2_cap.read()

#             if ret == False:
#                 print "Error reading image"
#                 break

#             cv2.imshow("lepton", cv2.resize(img, (640, 480)))
#             if cv2.waitKey(5) == 27:
#                 break

#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     OpenCvCapture().show_video()


class LeptonCamera:
    def __init__(self):
        self.rgb_topic = "/usb_cam/image_raw"
        self.ir_topic = "/ir/usb_cam/image_raw"
        self.depth_topic = "/camera/depth/image_rect_raw"
        self.bridge = CvBridge
        self.sub_rgb = rospy.Subscriber(self.rgb_topic, Image, self.rgb_callbcak, queue_size=1)
        self.sub_ir = rospy.Subscriber(self.ir_topic, Image, self.ir_callback, queue_size=1)
        self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
        
        
        self.count = 0
        
        self.output_pub_ir = rospy.Publisher("/lepton/out", Image, queue_size=1)
        
        self.output_pub_depth = rospy.Publisher("/depth/out", Image, queue_size=1)
        
        self.nowTick = rospy.get_time()
        self.pastTick = self.nowTick
        
        
    def ir_callback(self, msg):
        # print("Received an image!")
        bridge = CvBridge()
        self.image_ir = bridge.imgmsg_to_cv2(msg, "mono8")
        self.main(1)
    def rgb_callbcak(self, msg):
        # print("Received an rgb image!")
        bridge = CvBridge()
        self.image_rgb = bridge.imgmsg_to_cv2(msg, "bgr8")
        self.main(2)
    
    def depth_callback(self, msg):
        # print("Received an depth image!")
        bridge = CvBridge()
        self.image_depth = bridge.imgmsg_to_cv2(msg)
        self.main(3)
    
        
    def main(self, type):
        self.nowTick = rospy.get_time()
        # if self.nowTick - self.pastTick > 0.05:
        print("run")
        if type == 1:
            msg = CvBridge().cv2_to_imgmsg(self.image_ir)
            self.output_pub_ir.publish(msg)
        
        if type == 3:
            msg = CvBridge().cv2_to_imgmsg(self.image_depth)
            self.output_pub_depth.publish(msg)    
            
        self.nowTick = rospy.get_time()            
        self.pastTick = self.nowTick

def main(args):
    rospy.init_node('lepton')
    node = LeptonCamera()
    try:
        rospy.spin()
        # node.main()
    except KeyoardInterrupt:
        # except:
        print("Shutting down")
            
if __name__ == '__main__':
    main(sys.argv)
        
    

# cv2.namedWindow("preview")
# cameraID = 0
# vc = cv2.VideoCapture(cameraID)
# if vc.isOpened(): # try to get the first frame
#     rval, frame = vc.read()
#     print(rval)
#     print(frame)
# else:
#     print('no rval')
#     rval = False
# while rval:
#     cv2.imshow("preview", frame)
#     rval, frame = vc.read()
#     frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
#     print('hello')
#     key = cv2.waitKey(20)
#     if key == 27: # exit on ESC
#         break









# if __name__ == '__main__':
#     rospy.init_node('turtlebot3_python_node')

#     print('init complete')
#     rospy.sleep(1)
#     while not rospy.is_shutdown():

#         rospy.sleep(0.1)