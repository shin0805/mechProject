#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16MultiArray
import time

rospy.init_node('control', anonymous=True)

data = Int16MultiArray()

pub = rospy.Publisher('servo/command', Int16MultiArray, queue_size=1)

step = 0
st = time.time()
rate = rospy.Rate(5)
while not rospy.is_shutdown():
  if step % 4 == 0:
    data.data = [0, 0, 0, 0, 0, 0]
  elif step % 4 == 1:
    data.data = [90, 90, 90, 90, 90, 90]
  elif step % 4 == 2:
    data.data = [180, 180, 180, 180, 180, 180]
  elif step % 4 == 3:
    data.data = [90, 90, 90, 90, 90, 90]

  pub.publish(data)

  rate.sleep()

  if (time.time() - st) >= 1:
    step += 1
    st = time.time()
