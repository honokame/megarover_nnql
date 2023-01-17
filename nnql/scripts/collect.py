#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
import tf
import sys #引数
import os #ファイル読み書き
import csv #リストをcsvに書き込み
import message_filters
from subprocess import *
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid


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

def callback_scan(msg):
  time = msg.header.stamp
  scan = msg.ranges
  f_scan.write(str(time)) 
  writer = csv.writer(f_scan)
  writer.writerow(scan)

def callback_odom(msg):
  # global odom_x, odom_y, odom_theta
  time = msg.header.stamp
  odom_x = msg.pose.pose.position.x + 0.5
  odom_y = msg.pose.pose.position.y + 2.3 #2.3
  qx = msg.pose.pose.orientation.x
  qy = msg.pose.pose.orientation.y 
  qz = msg.pose.pose.orientation.z
  qw = msg.pose.pose.orientation.w
  eular = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
  odom_theta = round(eular[2]*180/np.pi)
  rospy.loginfo('odom_x:%d, odom_y:%d, odom_theta:%d',int(odom_x),int(odom_y),int(odom_theta))
  status = str(time) + ',' + str(int(odom_x)) + ',' + str(int(odom_y)) + ',' + str(int(odom_theta)) + '\n'
  f_status.write(status)

def callback_map(msg):
  mapdata = msg.data
  time = msg.header.stamp
  rospy.loginfo('size:%d, unknow:%d, know:%d',len(mapdata),mapdata.count(-1),mapdata.count(100))
  map_status = str(time) + ',' + str(mapdata.count(-1)) + ',' + str(mapdata.count(100)) + '\n'
  f_map.write(map_status)
  

def callback(msg1, msg2):
  time1 = msg1.header.stamp
  scan = msg1.ranges
  f_scan.write(str(time1)) 
  writer = csv.writer(f_scan)
  writer.writerow(scan)
  #f_scan.write('\n') 
  time2 = msg2.header.stamp
  odom_x = msg2.pose.pose.position.x + 2.59
  odom_y = msg2.pose.pose.position.y + 5.23 #2.3
  qx = msg2.pose.pose.orientation.x
  qy = msg2.pose.pose.orientation.y 
  qz = msg2.pose.pose.orientation.z
  qw = msg2.pose.pose.orientation.w
  eular = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
  odom_theta = round(eular[2]*180/np.pi)  
  if(odom_theta < 0):
    odom_theta = odom_theta + 360
 # odom_theta = odom_theta + 90
 
  #rospy.loginfo(str(time1))
  #rospy.loginfo(str(time2))
  #rospy.loginfo('odom_x:%d, odom_y:%d, odom_theta:%d',int(odom_x),int(odom_y),int(odom_theta))
  status = str(time2) + ',' + str(int(odom_x)) + ',' + str(int(odom_y)) + ',' + str(int(odom_theta)) + '\n'
  f_status.write(status)
  #f_status.write('\n')

def odometry():
  # rospy.init_node('collect')
  #scan_subscriber = rospy.Subscriber('/scan', LaserScan, callback_scan)
  #odom_subscriber = rospy.Subscriber('/odom', Odometry, callback_odom)
  map_subscriber = rospy.Subscriber('/map', OccupancyGrid, callback_map)
  scan_subscriber = message_filters.Subscriber('/scan', LaserScan)
  odom_subscriber = message_filters.Subscriber('/odom', Odometry)
  mf = message_filters.ApproximateTimeSynchronizer([scan_subscriber, odom_subscriber], 10, 0.01)
  mf.registerCallback(callback)
  for i in range(10000):
    get_status('vmegarover')
  rospy.spin()

if __name__ == '__main__':
   rospy.init_node('collect') #ノードの初期化
   args = sys.argv #引数
   status_csv = 'status' + args[1] + '.csv'
   scan_csv = 'scan' + args[1] + '.csv'
   map_csv = 'map' + args[1] + '.csv'
   f_status = open(status_csv,'w')
   rospy.loginfo('create %s',status_csv)
   f_scan = open(scan_csv,'w')
   rospy.loginfo('create %s',scan_csv)
   f_map = open(map_csv,'w')
   p = call(['rosnode','kill','slam_gmapping'])
   set_model('vmegarover', 6.6, 2.1, 0, 180)
   odometry()
  # f_status.write('\n')
  # f_scan.write('\n')
