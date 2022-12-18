#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu
import numpy as np

imu_msg = Imu()
calib_msg = Int16MultiArray()

def imuCb(msg):
  imu_msg = msg
  print("--- imu ---")
  print(str(imu_msg.orientation))
  print("\n")

def calibCb(msg):
  calib_msg = msg
  print("-- calib --")
  print(calib_msg.data)
  print("\n")

rospy.init_node('view', anonymous=True)
rospy.Subscriber("sensor/imu", Imu, imuCb)
rospy.Subscriber("sensor/calib", Int16MultiArray, calibCb)
rospy.spin()
