#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def set_model():
  rospy.init_node('spawn') #ノードの初期化
  #rospy.wait_for_service('/gazebo/set_model_state')
  
  model_state = ModelState()
  model_state.model_name = 'vmegarover'
  model_state.pose.position.x = 1
  model_state.pose.position.y = 0
  model_state.pose.position.z = 0
  model_state.pose.orientation.x = 0
  model_state.pose.orientation.y = 0
  model_state.pose.orientation.z = 0
  model_state.pose.orientation.w = 1
  model_state.twist.linear.x = 0
  model_state.twist.linear.y = 0
  model_state.twist.linear.z = 0
  model_state.twist.angular.x = 0
  model_state.twist.angular.y = 0
  model_state.twist.angular.z = 0

  rospy.wait_for_service('/gazebo/set_model_state')
  try:
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(model_state)
  except rospy.ServiceException as e:
    rospy.loginfo("Service did not process request: "+str(e))

if __name__ == '__main__':
#  try
   set_model()
#  expect rospy.ROSInterruptException:
#    pass
