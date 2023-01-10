#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
import tf
import sys #引数
import os #ファイル読み書き
import csv #リストをcsvに書き込み
from subprocess import *
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_srvs.srv import Empty


pos_start = [[0.5,2.3],[1.5,2.5],[4.2,2.5],[5.5,2.3],[6.5,2.2],[7.5,4.5],[8.5,5.7],[4.3,5.3]]
odom_x , odom_y, odom_theta = 0.5, 2.3, 0.0

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
  rospy.loginfo('x:%d, y:%d, yaw:%d',int(x),int(y),int(yaw))
  #status = str(int(x)) + ',' + str(int(y)) + ',' + str(int(yaw)) + '\n'
  #f_status.write(status)

def get_scan():
  msg = rospy.wait_for_message('/scan', LaserScan)
  scan = msg.ranges
  writer = csv.writer(f_scan)
  writer.writerow(scan)

def callback_odom(msg):
  # global odom_x, odom_y, odom_theta
  odom_x = msg.pose.pose.position.x + 0.5
  odom_y = msg.pose.pose.position.y + 2.3
  qx = msg.pose.pose.orientation.x
  qy = msg.pose.pose.orientation.y 
  qz = msg.pose.pose.orientation.z
  qw = msg.pose.pose.orientation.w
  eular = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
  odom_theta = round(eular[2]*180/np.pi)
  rospy.loginfo('odom_x:%d, odom_y:%d, odom_theta:%d',int(odom_x),int(odom_y),int(odom_theta))
  #status = str(int(x)) + ',' + str(int(y)) + ',' + str(int(yaw)) + '\n'
  #f_status.write(status)

def odometry():
  # rospy.init_node('collect')
  odom_subscriber = rospy.Subscriber('/odom', Odometry, callback_odom)
  get_status('vmegarover')
  rospy.spin()

if __name__ == '__main__':
   rospy.init_node('collect') #ノードの初期化
   args = sys.argv #引数
   status_csv = 'status' + args[1] + '.csv'
   scan_csv = 'scan' + args[1] + '.csv'
   f_status = open(status_csv,'w')
   rospy.loginfo('create %s',status_csv)
   f_scan = open(scan_csv,'w')
   rospy.loginfo('create %s',scan_csv)
   p = call(['rosnode','kill','slam_gmapping'])
   set_model('vmegarover', 0.5, 2.3, 0, 0)
   get_status('vmegarover')
   odometry()

   f_status.write('\n')
   f_scan.write('\n')
