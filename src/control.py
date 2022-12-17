#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import std_msgs.msg
import time
import sys

# ノードの初期化
rospy.init_node('control', anonymous=True)

# Float32MultiArray型のメッセージを定義
data = std_msgs.msg.Float32MultiArray()
# see https://answers.ros.org/question/380890/unable-to-publish-float64multiarray-in-python/
data.layout.dim = [std_msgs.msg.MultiArrayDimension(label='dim0', size=6, stride=6*4)]
# data.layout.append(std_msgs.msg.MultiArrayLayout())
# data.layout.dim[0].size = 6

# パブリッシュするトピックを定義
pub = rospy.Publisher('servo/command', std_msgs.msg.Float32MultiArray, queue_size=1)

step = 0
st = time.time()
# トピックにメッセージをパブリッシュ
rate = rospy.Rate(1)
while not rospy.is_shutdown():
  if step % 3 == 0:
    data.data = [0, 0, 0, 0, 0, 0]
    # data.data = [60, 60, 60, 60, 60, 60]
  elif step % 3 == 1:
    data.data = [90, 90, 90, 90, 90, 90]
  else:
    data.data = [180, 180, 180, 180, 180, 180]

  pub.publish(data)

  # print(sys.getsizeof(data)) 72
  rate.sleep()

  if (time.time() - st) >= 2:
    step += 1
    st = time.time()

