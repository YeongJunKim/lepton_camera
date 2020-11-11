#! /usr/bin/env python2

import cv2
import numpy as np
import rospy
import std_msgs
import sensor_msgs
import geometry_msgs



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






cv2.namedWindow("preview")
cameraID = 0
vc = cv2.VideoCapture(cameraID)
if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
    print(rval)
    print(frame)
else:
    print('no rval')
    rval = False
while rval:
    cv2.imshow("preview", frame)
    rval, frame = vc.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
    print('hello')
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break









# if __name__ == '__main__':
#     rospy.init_node('turtlebot3_python_node')

#     print('init complete')
#     rospy.sleep(1)
#     while not rospy.is_shutdown():

#         rospy.sleep(0.1)