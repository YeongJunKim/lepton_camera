#!/usr/bin/env python
# -*- coding: utf-8 -*-

from uvctypes import *
import time
import cv2
import numpy as np
try:
  from queue import Queue
except ImportError:
  from Queue import Queue
import platform

import rospy
import sys
import std_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs
from cv_bridge.boost.cv_bridge_boost import getCvType



class LeptonCamera:
  def __init__(self):
    print("initialize");
    self.PTR_PY_FRAME_CALLBACK = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(self.py_frame_callback)
    self.ctx = POINTER(uvc_context)()
    self.dev = POINTER(uvc_device)()
    self.devh = POINTER(uvc_device_handle)()
    self.ctrl = uvc_stream_ctrl()
    self.BUF_SIZE= 2
    self.q = Queue(self.BUF_SIZE)

    self.output_pub = rospy.Publisher("/lepton/out", Image, queue_size = 1)
    self.output_minmax_pub = rospy.Publisher("/lepton/rgb/out", Image, queue_size = 1)
    

    self.res = libuvc.uvc_init(byref(self.ctx), 0)
    if self.res < 0:
      print("uvc_init error")
      exit(1)

    try:
      self.res = libuvc.uvc_find_device(self.ctx, byref(self.dev), PT_USB_VID, PT_USB_PID, 0)
      if self.res < 0:
        print("uvc_find_device error")
        exit(1)

      try:
        self.res = libuvc.uvc_open(self.dev, byref(self.devh))
        if self.res < 0:
          print("uvc_open error")
          exit(1)

        print("device opened!")

        print_device_info(self.devh)
        print_device_formats(self.devh)

        self.frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_Y16)
        if len(self.frame_formats) == 0:
          print("device does not support Y16")
          exit(1)

        libuvc.uvc_get_stream_ctrl_format_size(self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_Y16,
          self.frame_formats[0].wWidth, self.frame_formats[0].wHeight, int(1e7 / self.frame_formats[0].dwDefaultFrameInterval)
        )

        self.res = libuvc.uvc_start_streaming(self.devh, byref(self.ctrl), self.PTR_PY_FRAME_CALLBACK, None, 0)
        if self.res < 0:
          print("uvc_start_streaming failed: {0}".format(self.res))
          exit(1)

        # try:
        #   while True:
        #     data = q.get(True, 500)
        #     if data is None:
        #       break
        #     data = cv2.resize(data[:,:], (640, 480))
        #     minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)
        #     img = raw_to_8bit(data)
        #     display_temperature(img, minVal, minLoc, (255, 0, 0))
        #     display_temperature(img, maxVal, maxLoc, (0, 0, 255))
        #     cv2.imshow('Lepton Radiometry', img)
        #     cv2.waitKey(1)

        #   cv2.destroyAllWindows()
        # finally:
        #   libuvc.uvc_stop_streaming(devh)
        rospy.spin()
        print("done")
      finally:
        libuvc.uvc_unref_device(self.dev)
    finally:
      libuvc.uvc_exit(self.ctx)


  def py_frame_callback(self, frame, userptr):
    self.array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
    self.data = np.frombuffer(
      self.array_pointer.contents, dtype=np.dtype(np.uint16)
    ).reshape(
      frame.contents.height, frame.contents.width
    ) # no copy

    # data = np.fromiter(
    #   frame.contents.data, dtype=np.dtype(np.uint8), count=frame.contents.data_bytes
    # ).reshape(
    #   frame.contents.height, frame.contents.width, 2
    # ) # copy
    if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
      return
    self.q.put(self.data)
    self.data = self.q.get(True, 500)
    self.data = cv2.resize(self.data[:,:], (640, 480))

    
    msg = CvBridge().cv2_to_imgmsg(self.data)
    self.output_pub.publish(msg)

    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(self.data)
    self.img = self.raw_to_8bit(self.data)
    self.display_temperature(self.img, minVal, minLoc, (255, 0, 0))
    self.display_temperature(self.img, maxVal, maxLoc, (0, 0, 255))

    msg = CvBridge().cv2_to_imgmsg(self.img, 'bgr8')
    self.output_minmax_pub.publish(msg)

    cv2.imshow('Lepton Radiometry', self.img)
    cv2.waitKey(1)

  def ktof(self, val):
    return (1.8 * self.ktoc(val) + 32.0)

  def ktoc(self, val):
    return (val - 27315) / 100.0

  def raw_to_8bit(self, data):
    cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
    np.right_shift(data, 8, data)
    return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)

  def display_temperature(self, img, val_k, loc, color):
    val = self.ktof(val_k)
    cv2.putText(img,"{0:.1f} degF".format(val), loc, cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
    x, y = loc
    cv2.line(img, (x - 2, y), (x + 2, y), color, 1)
    cv2.line(img, (x, y - 2), (x, y + 2), color, 1)

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
        