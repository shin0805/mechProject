#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import std_msgs.msg
import time

rospy.init_node('control', anonymous=True)

data = std_msgs.msg.Int16MultiArray()

pub = rospy.Publisher('servo/command', std_msgs.msg.Int16MultiArray, queue_size=1)

step = 0
st = time.time()
rate = rospy.Rate(5)
while not rospy.is_shutdown():
  if step % 3 == 0:
    data.data = [0, 0, 0, 0, 0, 0]
  elif step % 3 == 1:
    data.data = [90, 90, 90, 90, 90, 90]
  else:
    data.data = [180, 180, 180, 180, 180, 180]

  pub.publish(data)

  rate.sleep()

  if (time.time() - st) >= 1:
    step += 1
    st = time.time()
