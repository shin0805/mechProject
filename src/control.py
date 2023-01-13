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
commands = np.concatenate([commands, STANDING_POS], 0) # SKIP
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

def addRise():
  global commands
  commands = np.concatenate([commands, linspace(STANDING_POS, EXTENTION_POS, 10)], 0) 
  commands = np.concatenate([commands, linspace(EXTENTION_POS, ROLLED_POS, 1)], 0) 
  commands = np.concatenate([commands, linspace(ROLLED_POS, SLEEPING_POS, 40)], 0) 
  commands = np.concatenate([commands, linspace(SLEEPING_POS, STANDING_POS, 30)], 0)

def addTurn(is_left):
  global commands
  if is_left:
    commands = np.concatenate([commands, linspace(STANDING_POS, FORLEFT_POS1, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORLEFT_POS1, FORLEFT_POS2, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORLEFT_POS2, FORLEFT_POS3, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORLEFT_POS3, STANDING_POS, 15)], 0) 
  else:
    commands = np.concatenate([commands, linspace(STANDING_POS, FORRIGHT_POS1, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORRIGHT_POS1, FORRIGHT_POS2, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORRIGHT_POS2, FORRIGHT_POS3, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORRIGHT_POS3, STANDING_POS, 15)], 0) 

def addStep():
  global commands
  commands = np.concatenate([commands, linspace(STANDING_POS, WALKING_POS1, 2)], 0) 
  commands = np.concatenate([commands, linspace(WALKING_POS1, WALKING_POS1, 4)], 0) 
  commands = np.concatenate([commands, linspace(WALKING_POS1, WALKING_POS2, 2)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS2, WALKING_POS2, 4)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS2, WALKING_POS3, 2)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS3, WALKING_POS3, 4)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS3, WALKING_POS4, 2)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS4, WALKING_POS4, 4)], 0)
  # commands = np.concatenate([commands, linspace(WALKING_POS4, STANDING_POS, 8)], 0)

  commands = np.concatenate([commands, linspace(WALKING_POS4, STANDING_POS, 8)], 0)
  commands = np.concatenate([commands, linspace(STANDING_POS, STANDING_POS, 4)], 0)

  # commands = np.concatenate([commands, linspace(WALKING_POS3, WALKING_POS4, 2)], 0)
  # commands = np.concatenate([commands, linspace(WALKING_POS4, WALKING_POS4, 20)], 0)
  # commands = np.concatenate([commands, linspace(WALKING_POS4, WALKING_POS5, 2)], 0)
  # commands = np.concatenate([commands, linspace(WALKING_POS5, WALKING_POS5, 20)], 0)
  # commands = np.concatenate([commands, linspace(WALKING_POS5, WALKING_POS6, 2)], 0)


# addRise()
# for i in range(5):
#   addTurn(False)
# 
# for i in range(5):
#   addTurn(True)

for i in  range(5):
  addStep()

# commands = np.concatenate([commands, linspace(STANDING_POS, SLEEPING_POS, 30)], 0) 
# commands = np.concatenate([commands, linspace(SLEEPING_POS, STANDING_POS, 30)], 0)






rate = rospy.Rate(20)
start_time = time.time()
while not rospy.is_shutdown():
  # foot = STANDING_POS.copy()
  # foot[0, 0] += 10 * np.sin(4 * (time.time() - start_time))
  # foot[0, 1] += 10 * np.cos(4 * (time.time() - start_time))
  # foot[0, 2] += 10 * np.cos(4 * (time.time() - start_time))
  # foot[0, 3] += 10 * np.sin(4 * (time.time() - start_time))
  # foot[0, 4] += 10 * np.sin(4 * (time.time() - start_time))
  # foot[0, 5] += 10 * np.cos(4 * (time.time() - start_time))
  # commands = np.concatenate([commands, foot], 0)

  command.data = commands[0, :].tolist()
  if (commands.shape[0] - 1):
    commands = np.delete(commands, 0, 0)
  print(command.data)
  pub.publish(safeClip(command))
  rate.sleep()
