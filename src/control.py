#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np

from config import *

rospy.init_node('control', anonymous=True)

command = Int16MultiArray()

commands = SLEEPING_POS # SKIP
commands = np.concatenate([commands, SLEEPING_POS], 0) # SKIP
commands = np.concatenate([commands, STANDING_POS], 0) # SKIP

pub = rospy.Publisher('servo/command', Int16MultiArray, queue_size=1)

def safeClip(command):
  command.data = np.clip(np.array(command.data), ANGLE_MIN, ANGLE_MAX).tolist()
  return command

def linspace(start, end, step):
  L = np.concatenate([start.T, end.T], 1)
  coef = np.linspace(1, 0, step).reshape(1, step)
  R =  np.concatenate([coef, coef[:, ::-1]], 0)
  return np.dot(L, R).T

commands = np.concatenate([commands, linspace(SLEEPING_POS, STANDING_POS, 30)], 0)
commands = np.concatenate([commands, linspace(STANDING_POS, SLEEPING_POS, 30)], 0) 
commands = np.concatenate([commands, linspace(SLEEPING_POS, STANDING_POS, 30)], 0)

rate = rospy.Rate(20)
start_time = time.time()
while not rospy.is_shutdown():
  foot = STANDING_POS.copy()
  foot[0, 0] -= 15 * np.cos(4 * (time.time() - start_time))
  foot[0, 1] -= 15 * np.sin(4 * (time.time() - start_time))
  foot[0, 2] -= 15 * np.cos(4 * (time.time() - start_time))
  foot[0, 3] += 15 * np.sin(4 * (time.time() - start_time))
  foot[0, 4] += 15 * np.sin(4 * (time.time() - start_time))
  foot[0, 5] += 15 * np.cos(4 * (time.time() - start_time))
  commands = np.concatenate([commands, foot], 0)
  command.data = commands[0, :].tolist()
  if (commands.shape[0] - 1):
    commands = np.delete(commands, 0, 0)
  print(command.data)
  pub.publish(safeClip(command))
  rate.sleep()
