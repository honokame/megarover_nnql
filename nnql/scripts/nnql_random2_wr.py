#!/usr/bin/env python
# -*- coding: utf-8 -*-
#データセット3のパラメータ
import numpy as np

import rospy
import tf
import sys #引数
import os #ファイル読み書き
import csv #リストをcsvに書き込み
import feature_r9t12 as feature #特徴量計算
import random
from subprocess import *
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
import actionlib #random()
from actionlib_msgs.msg import * #random()
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #random()
from learn import NNQL_class
import rosparam
import rosnode

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
  if(yaw < 0):
    yaw = yaw + 360
  rospy.loginfo('x:%d, y:%d, yaw:%d',int(x),int(y),int(yaw))
  status = str(int(x)) + ',' + str(int(y)) + ',' + str(int(yaw)) + '\n'
  f_status.write(status)

def get_scan():
  msg = rospy.wait_for_message('/scan', LaserScan)
  scan = msg.ranges
  return scan

def wall():
  p = call(['rosrun','wall_follower','follow_wall.py'])
  rospy.sleep(1)

def frontier():
  rosparam.set_param("/move_base/global_costmap/inflation_layer/inflation_radius","0.35") 
  rosparam.set_param("/move_base/local_costmap/width","3")
  rosparam.set_param("/move_base/local_costmap/height","3")
  rosparam.set_param("/move_base/base_global_planner", "global_planner/GlobalPlanner")
  p = call(['rosnode','kill','move_base'])
  p = call(['rosrun','explore_lite','explore'])
  rospy.sleep(1)

def random(): 
  """
  rosparam.set_param("/move_base/global_costmap/inflation_layer/inflation_radius","0.25")
  rosparam.set_param("/move_base/local_costmap/width","2")
  rosparam.set_param("/move_base/local_costmap/height","2")
  rosparam.set_param("/move_base/base_global_planner", "rrtstar_planner/RRT")
  p = call(['rosnode','kill','move_base'])
  """
  ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  while not ac.wait_for_server(rospy.Duration(5)):
    rospy.loginfo('wait server')
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = 'map'
  goal.target_pose.header.stamp = rospy.Time.now()
  goal.target_pose.pose.position.x = 0
  goal.target_pose.pose.position.y = 0
  goal.target_pose.pose.orientation.x = 0
  goal.target_pose.pose.orientation.y = 0
  goal.target_pose.pose.orientation.z = 0
  goal.target_pose.pose.orientation.w = 1
  ac.send_goal(goal) 
  rospy.sleep(40)
  p = call(['rosnode','kill','move_base'])
  rospy.sleep(1)

def get_rrf(scan):
  feature.shapecontext(scan)
  os.system('python3 gcntest.py')
  state = np.loadtxt(fname="temp_rrf.csv", dtype="float", delimiter=",")
  rrf = state.tolist()
  return rrf

def get_occupancy(): #14000:100%,11200:80%
  msg = rospy.wait_for_message('map', OccupancyGrid)
  mapdata = msg.data
  occupancy = len(mapdata) - mapdata.count(-1)
  rospy.loginfo('occupancy cell:%d',occupancy)
  #rate = occupancy/float(14000)*100
  return occupancy

def get_distance():
  temp_dis = np.loadtxt(fname="temp_dis.csv", dtype="float", delimiter=",")
  rospy.loginfo('temp_dis:%f',temp_dis)
  return temp_dis

def get_reward(occupancy_init, occupancy, distance):
  #rate = occupancy/float(12520)*100 #既知領域÷距離
  rate = (occupancy-occupancy_init)/float(12520)*100 #増加面積÷移動距離
  reward = rate/float(distance) 
  rospy.loginfo('init:%d, occupancy:%d, distance:%f, reward:%f',occupancy_init,occupancy, distance, reward)
  return reward

def start():
  seed = np.random.randint(0,19) #19
  yaw = np.random.randint(0,361)
  if(seed < 8): #0-7
    x = np.random.uniform(0.5,7.0)
    if((x >= 1.7) & (x < 2.3)):
      y = 2.4
    else:
      y = np.random.uniform(2.2,2.4)
  elif(seed < 12): #8-11
    x = np.random.uniform(1.4,3.6)
    #if((x >= 1.8) & (x <= 2.9) | ((x >= 3.7) & (x <= 4.3))): 
    if((x >= 1.4) & (x <= 2.9)):
      y = 5.05 #5.3
      f = np.random.randint(0,2)
      yaw = 180*f
    else:
      y = np.random.uniform(5.2,5.3)
  elif(seed < 14): #12-13
    x = 0.95
    y = np.random.uniform(2.4,4.8)
    f = np.random.randint(1,3)
    yaw = 90*f
  elif(seed == 16): #14-15
    x = 4.1
    y = np.random.uniform(3.2,4.6) 
    f = np.random.randint(1,3)
    yaw = 90*f
  elif(seed < 19): #16-18
    x = 7.4
    y = np.random.uniform(3.4,4.7)
  elif(seed < 20): # 19   
    x = np.random.uniform(8.1,8.4)
    y = np.random.uniform(5.2,5.5)
  return x,y,yaw
  
if __name__ == '__main__':
  rospy.init_node('nnql') #ノードの初期化

  #args = sys.argv #引数
  #status_csv = 'status' + args[1] + '.csv'
  status_csv = 'NNQL/groundtruth.csv'
  #scan_csv = 'scan' + args[1] + '.csv'
  f_status = open(status_csv,'w')
  #f_scan = open(scan_csv,'w')
  result_csv = 'NNQL/result.csv'
  f_result = open(result_csv,'w')
  qdata_path = 'NNQL/Qdatabase'
  episode = 1 #901
  max_episode = 600
  step = 1
  NNQL = NNQL_class(qdata_path) #NNQL
  Qdatabase = NNQL.mk_Qdatabase(episode) #Qデータベース作成
  use_Qdatabase = Qdatabase #使用するデータベース
  action_list = ["frontier", "wall", "random"] #行動集合
  #for j in range(episode): #episode
  while(episode < max_episode+1):
    print("=======================================================================")
    p = call(['rosnode','kill','slam_gmapping'])
    start_x, start_y, start_yaw = start()
    set_model('vmegarover', start_x, start_y, 0, start_yaw)
    rospy.loginfo('%depisode',episode)
    distance = 0
    get_status('vmegarover')
    scan = get_scan()
    now_state = get_rrf(scan)
    occupancy_init =  get_occupancy()
    
    env_list = []
    #eps = 1/(0.1*(episode)+1) 
    
    #for i in range(4): #step
    while(step < 5):
      print("---------------------------------------------------------------------")
      rospy.loginfo('%dstep',step)
      #eps_random = np.random.rand()
      #if not (episode < 25 or eps>eps_random) :
      q_average,knn_list = NNQL.knn(now_state,use_Qdatabase) #knn,now_state=クラス固有ベクトル
      action_maxIndex = [i for i, x in enumerate(q_average) if x == max(q_average)]
      #  action_index = np.random.choice(action_maxIndex) # action_index = max(q_avg)
      #  action = action_list[action_index]
      #else:
      #  q_average = list([0]*3)
      action_index = np.random.randint(1, 3) #wall,random
      #  knn_list = None
      action = action_list[action_index]
      Qdatabase = NNQL.Q_data_add(now_state,q_average,Qdatabase)  #qデータベース追加
      
      rospy.loginfo('action:%s',action)

      if action == "frontier": #行動選択
        frontier()
      elif action == "wall":
        wall()
      elif action == "random":
        random()
      
      get_status('vmegarover')
      scan = get_scan()
      next_state = get_rrf(scan) #行動後の状態
      env_list.append([now_state,next_state,action_index,knn_list,q_average]) #結果保存
      now_state = next_state
      
      occupancy = get_occupancy()
      temp_dis = get_distance()
      distance = distance + temp_dis  
      step = step+1
      result_action = str(action_index)+','
      f_result.write(result_action)
    reward = get_reward(occupancy_init, occupancy, distance) #報酬
    result_episode = str(reward)+','+str(occupancy_init)+','+str(occupancy)+','+str(distance)+'\n'
    f_result.write(result_episode)
    for env in env_list:
      state = env[0]
      state_next = env[1]
      do_action = env[2]
      knn_list_all = env[3]
      q_list = env[4]
      Qdatabase = NNQL.q_learning(state,state_next,do_action,reward,knn_list_all,q_list,Qdatabase) #q値更新 
   
    if episode%25 == 0: #1000>50
      NNQL.save_Qdatabase(Qdatabase,episode,0) #Qdatabase保存

    if episode%25 == 0:
      use_Qdatabase = Qdatabase


    episode = episode+1
    step = 1
  NNQL.save_Qdatabase(Qdatabase,episode,0) #Qdatabase保存 
