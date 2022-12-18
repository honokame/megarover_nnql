#!/usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import random
import cv2
import pandas as pd
import glob
from scipy.stats import rankdata
import time

class NNQL_class:
   def __init__(self,state_path,robot_data,Qdatabase,class_data):
       self.state_path = state_path
       self.robot_data = robot_data
       self.class_data = class_data
       self.Class = [class_data[i][1] for i in range(len(class_data))] 
       #class_dataをvgg16の出力にする案はある
       self.class_data_bayes = [np.where(np.array(self.Class) == i)[0].tolist() for i in range(100)]
       
       self.Qdatabase = Qdatabase
       self.state_data = [Qdatabase[i][0] for i in range(len(self.Qdatabase))]

       files=glob.glob(state_path+'/*')
       files.sort()

       self.data_all = []
       self.state_all = []
       for i in range(len(files)):
          Pd = np.loadtxt(files[i],delimiter=',').tolist()
          rank = rankdata(-np.array(Pd))
          state = list(np.reciprocal(rank))
          self.state_all.append(state)
          max_idx = np.argmax(state)
          self.data_all.append(max_idx)


   #state
   def get_state(self,now_index):
       return self.state_all[now_index]


   #ロボットの位置
   def next_position(self,action, robot_index): 
     d_sum = 0
     next_index = None
     for i in self.robot_data[robot_index+1:]:
       d = float(i[3])
       d_sum += d
       if d_sum >= action:
         next_index = self.robot_data.index(i)
         break
       else:
         next_index = None
         pass
     return next_index #行動後のロボットのdataのidx


   #match
   def state_match(state,state_data_knn,Qdatabase_knn):
       if state in state_data_knn:
          match_index = state_data_knn.index(state)
          match_q = Qdatabase_knn[match_index][1]
          return match_q
       else:
          return None


   #k近傍探索
   def knn(self,state,Qdatabase_befor,state_data_befor):
    Qdatabase_knn = Qdatabase_befor
    state_data_knn = state_data_befor 
    match_q = NNQL_class.state_match(state,state_data_knn,Qdatabase_knn)
    if match_q == None: #マッチしなかった=KNN
      des2 = state_data_knn
      des1 = [state]
      des1 = np.array(des1).astype('uint8')
      des2 = np.array(des2).astype('uint8')
   
      bf = cv2.BFMatcher()
      matches = bf.knnMatch(des1,des2, k=4)
      matches = matches[0]
   
      sum_q = np.zeros(10)
      nn_index_list = []
      for i in matches:
        nn_index_list.append(i.trainIdx)
        sum_q += np.array(Qdatabase_knn[i.trainIdx][1])
      q_average = list((sum_q*(1/4)))
    else: #マッチ
      q_average = match_q
      nn_index_list = None

    q_max_index = q_average.index(max(q_average))
    return q_max_index,q_average,nn_index_list #[状態1のindex, 状態2のindex, 状態3のindex]


   # Qデータベース追加
   def q_data_add(self,state,q_average):
      if not state in list(self.state_data):
        self.Qdatabase.append([list(state),list(q_average)])
        self.state_data.append(list(state))
      else:pass 
      return


   #報酬関数
   def make_reward(self,next_index,next_state):
      class_label = self.Class[next_index]
      reward = next_state[class_label]
      return reward

   def make_reward2(self,next_index,next_state):
      class_label = self.Class[next_index]
      r_rank_val = next_state[class_label]
      if r_rank_val >= 0.1:
          reward = 1
      else:
          reward = -1
      return reward

   def make_reward3(self,next_index,next_state):
      class_label = self.Class[next_index]
      r_rank_val = next_state[class_label]
      if r_rank_val >= 0.1:
          reward = r_rank_val
      else:
          reward = -1
      return reward
 
   #評価
   def anr(self,next_index):
       class_label = self.Class[next_index]
       rank_val = self.next_rank[class_label]
       return rank_val
   
   def acc(self,next_index):
       class_label = self.Class[next_index]
       r_rank_val = self.acc_state[class_label]
       return r_rank_val

   #Q値を更新する関数
   def q_learning(self,now_state,next_state,action_index,reward,knn_index): 
      eta = 0.1  # 学習率
      #eta = 0.5  
      gamma = 0.9  # 時間割引率

      now_state_Qidx = self.state_data.index(now_state)
      
      _,next_q,_ = NNQL_class.knn(self,next_state,self.Qdatabase,self.state_data)

      now_q = self.Qdatabase[now_state_Qidx][1][action_index]

      self.Qdatabase[now_state_Qidx][1][action_index] = now_q + eta*(reward+gamma* np.nanmax(next_q) - now_q)

      if not knn_index == None: 
        for index in knn_index:
           now_q2 = self.Qdatabase[index][1][action_index]
           self.Qdatabase[index][1][action_index] = now_q2 + eta*(reward+gamma* np.nanmax(next_q)-now_q2)
      return list(self.Qdatabase),list(self.state_data)


   def start_PD(self,now_state,now_index):
        p=1/len(self.robot_data)
        P=[p for _ in range(len(self.robot_data))]
        start_state,start_bunpu = self.bayes(P,now_state,now_index,now_index)
        return start_bunpu,start_state

   def bayes(self,now_bunpu,next_obs,now_index,next_index): #pd=確率分布
        local=len(self.robot_data)
        bunpu_now = now_bunpu

        #bunpu_load
        bunpu_new = [next_obs[int(self.Class[i])] for i in range(local)]

        #bunpu_move
        bunpu_min=min(bunpu_now)
        move = next_index - now_index
        bunpu_move = [bunpu_min/1000 if i < move else bunpu_now[i-move] for i in range(local)]

        #bunpu_update
        s = np.array(bunpu_move)*np.array(bunpu_new)
        b = np.sum(s)
        bunpu_update = [float(s[i]/b) for i in range(len(s))]

        #pred_update
        pre = [float(np.sum([bunpu_update[i] for i in j])) if not j == [] else 0 for j in self.class_data_bayes]
        rank = rankdata(-np.array(pre))
        state = list(np.reciprocal(rank))
        self.acc_state = state
        return state,bunpu_update

