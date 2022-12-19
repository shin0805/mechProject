#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np

from config import *

rospy.init_node('control', anonymous=True)

command = Int16MultiArray()

pub = rospy.Publisher('servo/command', Int16MultiArray, queue_size=1)

def safeClip(command):
  command.data = np.clip(np.array(command.data), ANGLE_MIN, ANGLE_MAX).tolist()
  return command

step = 0
st = time.time()
rate = rospy.Rate(5)
while not rospy.is_shutdown():
  if step % 4 == 0:
    command.data = [0, 0, 0, 0, 0, 0]
  elif step % 4 == 1:
    command.data = [90, 90, 90, 90, 90, 90]
  elif step % 4 == 2:
    command.data = [180, 180, 180, 180, 180, 180]
  elif step % 4 == 3:
    command.data = [90, 90, 90, 90, 90, 90]

  pub.publish(safeClip(command))

  rate.sleep()

  if (time.time() - st) >= 1:
    step += 1
    st = time.time()
