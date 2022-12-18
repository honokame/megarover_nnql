#!/usr/bin/env python
# _*_ coding:utf_8 _*_

import numpy as np
import random
import os
import pandas as pd
import glob
from nnql import NNQL_class
import sys
import pickle
import matplotlib.pyplot as plt

#引数
if len(sys.argv) <= 3:
  print("Error! : python3 main.py [train_season] [start_episode] [MAX_epoch] [num]")
  sys.exit()

train = str(sys.argv[1])
start_episode = int(sys.argv[2])
max_episode = int(sys.argv[3])
num = str(sys.argv[4])

#data
robot_data = pd.read_csv('./data/data_'+ train + '.csv').values.tolist()
class_data = pd.read_csv('./data/class'+train+'.csv',header=None).values.tolist()
vgg_path = './data/vgg16_' + train + '/'

#データを保存
os.makedirs('save'+num,exist_ok=True)
def save(Qdatabase,T):
   with open('save'+num+'/Qdatabase_T'+str(T)+'.pickle', mode='wb') as f:
        pickle.dump(Qdatabase,f)

#データ読み込み
if start_episode != 1:
  with open('./save'+num+'/Qdatabase_T'+str(start_episode)+'.pickle', mode='rb') as f:
     use_Qdatabase = pickle.load(f)
  use_state_data = [use_Qdatabase[i][0] for i in range(len(use_Qdatabase))]
elif start_episode == 1:
  use_Qdatabase = []
  use_state_data = []

#main
NNQL = NNQL_class(vgg_path,robot_data,use_Qdatabase,class_data)
action_list = [i for i in range(1,11)]

episode = start_episode
R = 0
x_list = []
y_list = []
while episode < max_episode+1:
    print('episode =',episode,'/',max_episode)
    #初期値
    now_index = random.randint(0, len(robot_data)-2)
    now_obs = NNQL.get_state(now_index)
    now_bunpu,now_state  = NNQL.start_PD(now_obs,now_index)
    action_count = 0
    eps = 1/(0.1*(episode+1)+1)
    while action_count < 4: #1eps
      if episode > 25 :
         q_max_index,q_average,nn_idx_list = NNQL.knn(now_state,use_Qdatabase,use_state_data) #knn
         if eps>random.random():
             q_max_index = random.randint(0, 9)
      else:
         q_max_index = random.randint(0, 9)
         nn_idx_list = None
         q_average = list([0]*10)
         #q_average = [random.uniform(-0.1,0.1) for i in range(10)]

      NNQL.q_data_add(now_state,q_average) #qデータベース追加
      print(len(use_Qdatabase))
      print(len(use_state_data))
      act = action_list[q_max_index]
      noise = act*0.1
      action = act + random.uniform(-noise,noise)

      next_index = NNQL.next_position(action,now_index) #行動後の位置

      if next_index == None:break #終了判定

      next_obs = NNQL.get_state(next_index) #行動後の状態

      next_state,bunpu_update = NNQL.bayes(now_bunpu,next_obs,now_index,next_index) #bayes
      
      reward = NNQL.make_reward2(next_index,next_state) #報酬
      Qdatabase,state_data = NNQL.q_learning(now_state,next_state,q_max_index,reward, nn_idx_list) #q値更新

      now_bunpu = bunpu_update
      now_state = next_state
      now_index = next_index
      action_count += 1

    if episode < 25:
        q_average = list([0]*10)
        #q_average = [random.uniform(-0.1,0.1) for i in range(10)]
        NNQL.q_data_add(now_state,q_average)
    else:
        _,q_average,_ = NNQL.knn(now_state,use_Qdatabase,use_state_data) #knn
        NNQL.q_data_add(now_state,q_average)

    episode+=1

    if episode%5000 == 0: 
        save(Qdatabase,episode) #Qdatabase保存

    if episode%25 == 0:
        use_Qdatabase = Qdatabase
        use_state_data = state_data
    #plot
    if action_count == 4:
       r = NNQL.acc(now_index)
       if r == 1:
           R += 1
       else:
           pass
       x_list.append(R/(episode-start_episode))
       y_list.append(episode-start_episode)

    if episode%1000 == 0: 
        fig = plt.figure()
        plt.plot(y_list,x_list)
        plt.savefig("train"+num+".png")
        plt.close()



