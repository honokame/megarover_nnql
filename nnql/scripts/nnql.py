#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
import tf
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState

pos_start = [[2.2,2],[2.2,1],[2.2,3],[5.5,1],[4.5,7.5]]
pos_goal = [[2,5],[2,7],[2,1],[5.2,3],[5.2,4],[6,8.5]]

def set_model(name, x, y, z, yaw): #指定位置にセット
  #rospy.init_node('nnql') #ノードの初期化
  
  model_state = ModelState()
  model_state.model_name = name
  model_state.pose.position.x = x
  model_state.pose.position.y = y
  model_state.pose.position.z = z
  model_state.pose.orientation.x = 0
  model_state.pose.orientation.y = 0
  model_state.pose.orientation.z = np.sin((yaw*np.pi/180) / 2)
  model_state.pose.orientation.w = np.cos((yaw*np.pi/180) / 2)
  model_state.twist.linear.x = 0
  model_state.twist.linear.y = 0
  model_state.twist.linear.z = 0
  model_state.twist.angular.x = 0
  model_state.twist.angular.y = 0
  model_state.twist.angular.z = 0
  
  rospy.wait_for_service('/gazebo/set_model_state')
  try:
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    res = set_state(model_state)
  except rospy.ServiceException, e:
    print("Service call failed: %s" %e)

def get_model(name):
  rospy.wait_for_service('/gazebo/get_model_state')
  try:
    serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    get_state = serviceResponse(model_name=name)
  except rospy.ServiceException, e:
    print("Service call failed: %s" %e)
  return get_state

def get_status(name):
  model_state = get_model(name)
  x = round(model_state.pose.position.x) #roundにするかは要検討
  y = round(model_state.pose.position.y)
  q1 = model_state.pose.orientation.x
  q2 = model_state.pose.orientation.y 
  q3 = model_state.pose.orientation.z
  q4 = model_state.pose.orientation.w
  eular = tf.transformations.euler_from_quaternion((q1, q2, q3, q4))
  yaw = round(eular[2]*180/np.pi)
  print(str(name),x, y, yaw)

if __name__ == '__main__':
   rospy.init_node('nnql') #ノードの初期化
   #np.random.shuffle(pos)
   #x, y = pos[0] 
   start_y, start_x = pos_start[np.random.randint(0, len(pos_start))]
   goal_y, goal_x = pos_goal[np.random.randint(0, len(pos_goal))]
   set_model('vmegarover', start_x, start_y, 0, np.random.randint(0,361))
   set_model('target', goal_x, goal_y, 0.01, 0)
   #set_model('vmegarover', 0, 0, -90)
   get_status('vmegarover')
   get_status('target')
