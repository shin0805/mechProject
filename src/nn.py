#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time

import torch
import torch.nn as nn
from rl_games.algos_torch.models import ModelA2CContinuousLogStd
import onnxruntime
import numpy as np

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from config import *


command = Int16MultiArray()
commands = SLEEPING_POS # SKIP
commands = np.concatenate([commands, STANDING_POS], 0) # SKIP
commands = np.concatenate([commands, STANDING_POS], 0) # SKIP

euler_msg = Vector3()
imu_msg = Imu()

def safeClip(command):
  tmp = np.clip(np.array(command.data), ANGLE_MIN, ANGLE_MAX).tolist()
  command.data = list(map(int, tmp))
  return command

def linspace(start, end, step):
  L = np.concatenate([start.T, end.T], 1)
  coef = np.linspace(1, 0, step).reshape(1, step)
  R =  np.concatenate([coef, coef[:, ::-1]], 0)
  return np.dot(L, R).T

def imuCb(msg):
  global imu_msg
  imu_msg = msg

def eulerCb(msg):
  global euler_msg
  euler_msg = msg

def run_onnx_model(model_path, input_data):
  session = onnxruntime.InferenceSession(model_path)

  input_name = session.get_inputs()[0].name
  input_shape = session.get_inputs()[0].shape
  input_data = np.array(input_data, dtype=np.float32)
  input_data = input_data.reshape(input_shape)

  outputs = session.run(["mu"], {input_name: input_data})

  return outputs[0]

def simRad2realDeg(rad):
    # TODO
    pass


if __name__=="__main__":
  rospy.init_node('rl_control', anonymous=True)
  rospy.Subscriber("sensor/euler", Vector3, eulerCb)
  rospy.Subscriber("sensor/imu", Imu, imuCb)
  pub = rospy.Publisher('servo/command', Int16MultiArray, queue_size=1)
  rate = rospy.Rate(20)
  model_path=(os.environ['HOME'] + "/IsaacGymEnvs/isaacgymenvs/runs/Chair_sample/nn/Chair.onnx")
  numRotationHis = 20
  numActionHis = 20
  rotation_history = np.zeros([numRotationHis, 4])
  rotation_history[:,  3] = 1.0
  action_history = np.ones([numActionHis, 6])
  
  while not rospy.is_shutdown():
    # action
    action = run_onnx_model(model_path, np.concatenate([rotation_history.flatten(), action_history.flatten()], 0))
    print(action)
  
    commands = np.concatenate([commands, linspace(ROLLED_POS, STANDING_POS, 2)], 0)
    # commands = np.concatenate([commands, simRad2realDeg(action)], 0)
    command.data = commands[0, :].tolist()
    if (commands.shape[0] - 1):
      commands = np.delete(commands, 0, 0)
    # print(command.data)
    pub.publish(safeClip(command))

    # obs
    rotation = np.array([[imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]])
    rotation_history = np.concatenate([rotation, rotation_history], 0)[:-1, :]
    action_history = np.concatenate([action, action_history], 0)[:-1, :]

    rate.sleep()
