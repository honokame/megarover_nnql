#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
import tf
import sys #引数
import os #ファイル読み書き
import csv #リストをcsvに書き込み
import feature_r9t12 as feature #特徴量計算
from subprocess import *
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

pos_start = [[0.5,2.3],[1.5,2.5],[4.2,2.5],[5.5,2.3],[6.5,2.2],[7.5,4.5],[8.5,5.7],[4.3,5.3]]

def set_model(name, x, y, z, yaw): #指定位置にセット
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
  x = model_state.pose.position.x
  y = model_state.pose.position.y
  q1 = model_state.pose.orientation.x
  q2 = model_state.pose.orientation.y 
  q3 = model_state.pose.orientation.z
  q4 = model_state.pose.orientation.w
  eular = tf.transformations.euler_from_quaternion((q1, q2, q3, q4))
  yaw = round(eular[2]*180/np.pi)
  #print(str(name),x, y, yaw)
  #print(str(name),int(x),int(y),round(yaw))
  rospy.loginfo('x:%d, y:%d, yaw:%d',int(x),int(y),int(yaw))
  status = str(int(x)) + ',' + str(int(y)) + ',' + str(int(yaw)) + '\n'
  f_status.write(status)

def get_scan():
  msg = rospy.wait_for_message('/scan', LaserScan)
  scan = msg.ranges
  #writer = csv.writer(f_scan)
  #writer.writerow(scan)
  return scan

def wall():
  p = call(['rosrun','wall_follower','follow_wall.py'])
  rospy.sleep(1)

def frontier():
  p = call(['rosrun','explore_lite','explore'])
  rospy.sleep(1)

def random():
  ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  while not ac.wait_for_server(rospy.Duration(5)):
    rospy.loginfo('wait server')
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = 'map'
  goal.target_pose.header.stamp = rospy.Time.now()
  goal.target_pose.pose.position.x = np.random.randint(0,8)*np.random.rand()
  goal.target_pose.pose.position.y = np.random.randint(0,5)*np.random.rand()
  yaw = np.random.randint(0,361)
  goal.target_pose.pose.orientation.x = 0
  goal.target_pose.pose.orientation.y = 0
  goal.target_pose.pose.orientation.z = np.sin((yaw*np.pi/180) / 2)
  goal.target_pose.pose.orientation.w = np.cos((yaw*np.pi/180) / 2)
  ac.send_goal(goal) 
  rospy.sleep(10)
  p = call(['rosnode','kill','move_base'])

if __name__ == '__main__':
  rospy.init_node('nnql') #ノードの初期化

  args = sys.argv #引数
  status_csv = 'status' + args[1] + '.csv'
  scan_csv = 'scan' + args[1] + '.csv'
  f_status = open(status_csv,'w')
  f_scan = open(scan_csv,'w')
  action_list = ["frontier", "wall", "random"]
  for j in range(10):
    p = call(['rosnode','kill','slam_gmapping'])
    start_x, start_y = pos_start[np.random.randint(0, len(pos_start))]
    start_yaw = np.random.randint(0, 361)
    set_model('vmegarover', start_x, start_y, 0, start_yaw)
    rospy.loginfo('ep:%d, x:%d, y:%d, yaw:%d',j,start_x,start_y,start_yaw)
    get_status('vmegarover')
    scan = get_scan()
    feature.shapecontext(scan)
    os.system('python3 gcntest.py')
    state = np.loadtxt(fname="temp_rrf.csv", dtype="float", delimiter=",")
    now_state = state.tolist()
    print(now_state)
    for i in range(4):
      rospy.loginfo('%dstep',i)
      action_index = np.random.randint(0, 3)
      action_index = 0
      action = action_list[action_index]
      if action == "frontier":
        frontier()
      elif action == "wall":
        wall()
      elif action == "random":
        random()
      
      get_status('vmegarover')
      scan = get_scan()
      feature.shapecontext(scan)
      os.system('python3 gcntest.py')
