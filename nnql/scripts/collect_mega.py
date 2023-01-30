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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
import commands

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
  time2 = msg2.header.stamp
  odom_x = msg2.pose.pose.position.x + 1.5
  odom_y = msg2.pose.pose.position.y + 5.2 #2.3
  qx = msg2.pose.pose.orientation.x
  qy = msg2.pose.pose.orientation.y 
  qz = msg2.pose.pose.orientation.z
  qw = msg2.pose.pose.orientation.w
  eular = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
  odom_theta = round(eular[2]*180/np.pi)  
  if(odom_theta < 0):
    odom_theta = odom_theta + 360
 
  rospy.loginfo('odom_x:%d, odom_y:%d, odom_theta:%d',odom_x,odom_y,int(odom_theta))
  status = str(time2) + ',' + str(odom_x) + ',' + str(odom_y) + ',' + str(int(odom_theta)) + '\n'
  f_status.write(status)

def odometry():
  map_subscriber = rospy.Subscriber('/map', OccupancyGrid, callback_map)
  scan_subscriber = message_filters.Subscriber('/scan', LaserScan)
  odom_subscriber = message_filters.Subscriber('/odom', Odometry)
  mf = message_filters.ApproximateTimeSynchronizer([scan_subscriber, odom_subscriber], 10, 0.05)
  mf.registerCallback(callback)
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
   odometry()
