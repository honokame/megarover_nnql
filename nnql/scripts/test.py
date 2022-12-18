#!/usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import os

import pandas as pd
from nnql import NNQL_class
import sys
import pickle
import random
import time
import csv
#引数

if len(sys.argv) <= 2:
  print("Error! : python3 main.py [save_name] [T]")
  sys.exit()

save_name = str(sys.argv[1])
T = str(sys.argv[2])

#data
robot_data = pd.read_csv('./data/data_0804.csv').values.tolist()

class_data = pd.read_csv('./data/class0804.csv',header=None).values.tolist()

gcn_state_path = './vgg16_0804/'


with open('save'+save_name+'/Qdatabase_T'+T+'.pickle', mode='rb') as f:
     Qdatabase = pickle.load(f)
state_data = [Qdatabase[i][0] for i in range(len(Qdatabase))]

action_list = [i for i in range(1,11)]

NNQL = NNQL_class(gcn_state_path,robot_data,Qdatabase,class_data)

N = 0
result = [0]*4
R = []
TIME=[]
while N < 1000:
   print(N)
   action_count = 0
   now_index = random.randint(0,len(robot_data)-2)
   now_obs = NNQL.get_state(now_index)
   now_bunpu,now_state  = NNQL.start_PD(now_obs,now_index)
   reward_1step = []
   while action_count < 4:
      q_max_index,q_average,_ = NNQL.knn(now_state,Qdatabase,state_data) #knn
      action = action_list[q_max_index]
      next_index = NNQL.next_position(action,now_index) #行動後の位置

      if next_index == None:break #終了判定
      next_state = NNQL.get_state(next_index) #行動後の状態

      next_pre_rank,bunpu_update = NNQL.bayes(now_bunpu,next_state,now_index,next_index)

      reward = NNQL.make_reward(next_index,next_pre_rank)

      if reward==1:
         reward_1step.append(reward)
      else:
         reward_1step.append(0)

      R.append([class_data[next_index],np.argmax(next_pre_rank)])
      now_state = next_pre_rank
      now_index = next_index
      now_bunpu = bunpu_update
      action_count += 1
      
   if action_count == 4:
     result= np.array(result) +  np.array(reward_1step)
     N += 1

result = list(result)
for j in range(len(result)):
   result[j] = result[j]/N*100

sum_re = sum(result)


os.makedirs('result',exist_ok=True)
with open('result/'+save_name+'_T'+T+'.txt','w') as f:
   print('エピソード数'+str(N),file=f)
   for k in range(len(result)):
      p = result[k]
      print('action'+str(k+1)+':'+str(p)+'%',file=f)
   print('avg.'+str(sum_re/4)+'%',file=f)


