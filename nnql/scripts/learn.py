#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import numpy as np
import cv2
import pandas as pd
import pickle
import glob
from scipy.stats import rankdata



class NNQL_class:
   def __init__(self,qdata_path):
       self.qdatabase_path = qdata_path

   def mk_Qdatabase(self,episode,train):
        if episode == 0:
            Qdatabase = {} #{state:Qdata}
        else:
            with open(self.qdatabase_path +'/Qdatabase_T'+str(episode)+'.pickle', mode='rb') as f:
                Qdatabase = pickle.load(f)
        return Qdatabase

   def save_Qdatabase(self,Qdatabase,episode,train):  #データを保存
       #os.makedirs(self.qdatabase_path,exist_ok=True) #python3
       with open(self.qdatabase_path +'/Qdatabase_T'+str(episode)+'.pickle', mode='wb') as f:
            pickle.dump(Qdatabase,f)


   #match
   def state_match(state,state_data_knn,Qdatabase_knn):
       if state in state_data_knn:
          match_index = state_data_knn.index(state)
          match_q = Qdatabase_knn[match_index][1]
          return match_q
       else:
          return None


   def knn(self,state,Qdatabase):
     knn_num = 4
     if tuple(state) in Qdatabase: #state match Qdatabase
        Q_value = list(Qdatabase[tuple(state)])
        knn_list = None
     else:  #state don't match Qdatabase
        des1 = np.array([state]).astype('uint8')
        des2 = [list(i) for i in list(Qdatabase.keys())]
        
        des2 = np.array(des2,dtype='u1')
       
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1,des2, k=knn_num)
        matches = matches[0]
       
        knn_list = []
        Q_sum = np.zeros(3)
        for i in matches:
            nn_index = i.trainIdx
            nn_state = list(Qdatabase.keys())[nn_index]
            knn_list.append(nn_state)
            Q_sum += np.array(list(Qdatabase.values())[nn_index])
        Q_value = Q_sum*(1/len(matches))
 
     return Q_value,knn_list #[状態1のindex, 状態2のindex, 状態3のindex]


   # Qデータベース追加
   def Q_data_add(self,state,q_value,Qdatabase):
      if not state in list(Qdatabase.keys()):
         Qdatabase.update({tuple(state):q_value})
      else:pass 
      return Qdatabase


   #報酬関数
   def make_reward(self,i,flag):

      if (i != 100) and (flag==1):
          reward = 1
      elif (i == 100) and (flag==0):
          reward = -1
      return reward

   #Q値を更新する関数
   def q_learning(self,now_state, next_state, action_index,reward,knn_list,q_list,Qdatabase): 
      eta = 0.1  # 学習率
      gamma = 0.9  # 時間割引率

      now_Q = np.max(q_list)

      now_Q_value_list = list(Qdatabase[tuple(now_state)])

      next_Q_value_list,_ = self.knn(next_state,Qdatabase)
      next_Q = np.nanmax(next_Q_value_list)

      new_Q = now_Q + eta*(reward + gamma *next_Q  - now_Q)

      now_Q_value_list[action_index] = new_Q

      Qdatabase[tuple(now_state)] = now_Q_value_list

      if not knn_list == None:
         for knn_state in knn_list:
            now_Q_value_list = list(Qdatabase[tuple(knn_state)])
            now_Q = now_Q_value_list[action_index]
            new_Q = now_Q + eta*(reward + gamma * next_Q - now_Q)
            now_Q_value_list[action_index] = new_Q

            Qdatabase[tuple(knn_state)] = now_Q_value_list

      return Qdatabase

