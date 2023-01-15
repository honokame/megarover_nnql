#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
#from sklearn.model_selection import train_test_split

# data = pd.read_csv(path +'wallstatus.csv', header=None)
data = pd.read_csv('data/statustest1.csv', header=None)
data = data.assign(place = 0,place60 = 0,place30 = 0)

for index,row in data.iterrows():
  x = row[1]
  y = row[2]
  t = row[3]
  #場所クラス
  if(y == 0):
    row[4] = 26 #26,30
  elif(y == 1):
    if((x == 3) | (x == 4)):
      row[4] = 25 #25,26
    if(x == 5):
      row[4] = 5 #5,27
    if(x == 6):
      row[4] = 6 #6,28
    if(x == 7):
      row[4] = 7 #7,29
  elif(y == 2):
    if(x == 0):
      row[4] = 1
    if(x == 1):
      row[4] = 2
    if(x == 2):
      row[4] = 3
    if(x == 3):
      row[4] = 4
    if(x == 4):
      row[4] = 5
    if(x == 5):
      row[4] = 6
    if(x == 6):
      row[4] = 7
    if(x == 7):
      row[4] = 8
  elif(y==3):
    if((x == 0) | (x == 1)):
      row[4] = 9
    if(x == 4):
      row[4] = 10
    if((x == 5) | (x == 6)):
      row[4] = 11
    if((x == 7)|(x == 8)):
      row[4] = 12
  elif(y==4):
    if((x == 0) | (x == 1)):
      row[4] = 13
    if(x == 4):
      row[4] = 14
    if(x == 5):
      row[4] = 15
    if(x == 7):
      row[4] = 16
    if(x == 8):
      row[4] = 17
  elif(y == 5):
    if((x == 0) | (x == 1)):
      row[4] = 18
    if(x == 2):
      row[4] = 19
    if(x == 3):
      row[4] = 20
    if(x == 4):
      row[4] = 21
    if(x == 7):
      row[4] = 22
    if(x == 8):
      row[4] = 23
  elif(y == 6):
    if((x == 7) | (x == 8)):
      row[4] = 24 #24,25
  else:
      row[4] = 0

  #角度クラス
  if(t < 23):
    row[5] = row[4]*8-7
  elif(t < 68):
    row[5] = row[4]*8-6
  elif(t < 113):
    row[5] = row[4]*8-5
  elif(t < 158):
    row[5] = row[4]*8-4
  elif(t < 203):
    row[5] = row[4]*8-3
  elif(t < 248):
    row[5] = row[4]*8-2
  elif(t < 293):
    row[5] = row[4]*8-1
  elif(t < 338):
    row[5] = row[4]*8
  else:
    row[5] = row[4]*8-7

  if(t < 15):
    row[6] = row[4]*12-11
  elif(t < 45):
    row[6] = row[4]*12-10
  elif(t < 75):
    row[6] = row[4]*12-9
  elif(t < 105):
    row[6] = row[4]*12-8
  elif(t < 135):
    row[6] = row[4]*12-7
  elif(t < 165):
    row[6] = row[4]*12-6
  elif(t < 195):
    row[6] = row[4]*12-5
  elif(t < 225):
    row[6] = row[4]*12-4
  elif(t < 255):
    row[6] = row[4]*12-3
  elif(t < 285):
    row[6] = row[4]*12-2
  elif(t < 315):
    row[6] = row[4]*12-1
  elif(t < 345):
    row[6] = row[4]*12
  else:
    row[6] = row[4]*12-11

data.to_csv('data/statusclasstest1_26.csv', header=None,index=None)
