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
from std_srvs.srv import Empty
from learn import NNQL_class

pos_start = [[2,2.2],[1,2.2],[3,2.2],[1,5.5],[7.5,4.5]]
pos_goal = [[5,2],[7,2],[1,2],[3,5.2],[4,5.2],[8.5,5.5]]

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
  writer = csv.writer(f_scan)
  writer.writerow(scan)
  #print(scan[1])

def wall():
  p = call(['rosrun','wall_follower','wall_follower'])
  rospy.sleep(1)

def frontier():
  p = call(['rosrun','explore_lite','explore'])
  rospy.sleep(1)

if __name__ == '__main__':
   rospy.init_node('nnql') #ノードの初期化

   args = sys.argv #引数
   status_csv = 'status_' + args[1] + '.csv'
   scan_csv = 'scan_' + args[1] + '.csv'
   f_status = open(status_csv,'w')
   rospy.loginfo('create %s',status_csv)
   f_scan = open(scan_csv,'w')
   rospy.loginfo('create %s',scan_csv)
   
   NNQL = NNQL_class(VPR_path,robot_data,class_data)

   Qdatabase = NNQL.mk_Qdatabase(0)
   use_Qdatabase = Qdatabase

   action_list = ["wall","random","frontier"]

   #np.random.shuffle(pos)
   #x, y = pos[0] 
   #goal_x, goal_y = pos_goal[np.random.randint(0, len(pos_goal))]
   #set_model('target', goal_x, goal_y, 0.01, 0)
   #set_model('vmegarover', 0, 0, -90)
   #get_status('target')
   for j in range(10):
     p = call(['rosnode','kill','slam_gmapping'])
     start_x, start_y = pos_start[np.random.randint(0, len(pos_start))]
     start_yaw = np.random.randint(0,361)
     set_model('vmegarover', start_x, start_y, 0, start_yaw)
     rospy.loginfo('init x:%d, y:%d, yaw:%d',start_x,start_y,start_yaw)

     eps = 1/(0.1*(j)+1)
   
     for i in range(100):
       rospy.loginfo('%d',i)

       eps_random=random.random()
       if not (j < 25 or eps>eps_random) :
         q_average,knn_list = NNQL.knn(now_state,use_Qdatabase) #knn,now_state=クラス固有ベクトル
         action_maxIndex = [i for i, x in enumerate(q_average) if x == max(q_average)]
         action_index = random.choice(action_maxIndex) # action_index = max(q_avg)
         action = action_list[action_index]
       else:
        q_average = list([0]*3)
         action_index = random.randint(0, 2)
         knn_list = None
         action = action_list[action_index]


       Qdatabase = NNQL.Q_data_add(now_state,q_average,Qdatabase)  #qデータベース追加

       if action == "wall":
         wall()
       elif action == "random":
         random()
       elif action == "frontier":
         frontier()

       get_scan()
       get_status('vmegarover')



       next_state = #gcn

       env_list.append([now_state,next_state,action_index,knn_list,q_average]) #結果保存

       now_state = next_state


       if flag == 1:
         break

     reward = NNQL.make_reward(i,flag) #報酬
     for env in env_list:
        state = env[0]
        state_next = env[1]
        do_action = env[2]
        knn_list_all = env[4]
        q_list = env[5]
        Qdatabase = NNQL.q_learning(state,state_next,do_action,reward,knn_list_all,q_list,Qdatabase) #q値更新 
   

     if episode%1000 == 0: 
        NNQL.save_Qdatabase(Qdatabase,episode,train) #Qdatabase保存

     if episode%25 == 0:
        use_Qdatabase = Qdatabase

   NNQL.save_Qdatabase(Qdatabase,episode,train) #Qdatabase保存
