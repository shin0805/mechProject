#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import tf

imu_msg = Imu()
euler_msg = Vector3()
calib_msg = Int16MultiArray()

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
          euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def imuCb(msg):
  imu_msg = msg
  print("--- imu ---")
  print(str(imu_msg.orientation))
  # print("\n")
  # print(str(quaternion_to_euler(imu_msg.orientation)))
  print("\n")

def eulerCb(msg):
  euler_msg = msg
  print("--- euler ---")
  print(str(euler_msg.x))
  print(str(euler_msg.y))
  print(str(euler_msg.z))


def calibCb(msg):
  calib_msg = msg
  print("-- calib --")
  print(calib_msg.data)
  print("\n")

rospy.init_node('view', anonymous=True)
rospy.Subscriber("sensor/imu", Imu, imuCb)
rospy.Subscriber("sensor/euler", Vector3, eulerCb)
rospy.Subscriber("sensor/calib", Int16MultiArray, calibCb)
rospy.spin()
