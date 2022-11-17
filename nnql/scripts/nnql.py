#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def set_model(name, x, y, yaw): #指定位置にセット
  rospy.init_node('nnql') #ノードの初期化
  
  model_state = ModelState()
  model_state.model_name = name
  model_state.pose.position.x = x
  model_state.pose.position.y = y
  model_state.pose.position.z = 0
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
  print(yaw)
  rospy.wait_for_service('/gazebo/set_model_state')
  try:
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(model_state)
  except rospy.ServiceException as e:
    rospy.loginfo("Service did not process request: "+str(e))

if __name__ == '__main__':
   set_model('vmegarover', 0, 0, np.random.randint(0,361))

