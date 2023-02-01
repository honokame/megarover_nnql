#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import sys

input_file = 'status' + sys.argv[1] + '.csv' #入力ファイル
data = pd.read_csv(input_file, header=None) #入力ファイル読み込み
data = data.assign(place = 0,place60 = 0,place30 = 0) #場所クラスの列を作成

output_file = 'statusclass' + sys.argv[1] + '.csv' #出力ファイル

for index,row in data.iterrows():
  x = int(row[1])
  y = int(row[2])
  t = int(row[3])
  #場所クラス
  if(y == 0):
    row[4] = 26 
  elif(y == 1):
    if(x == 0):
      row[4] = 1
    if(x == 1):
      row[4] = 2
    if(x == 2):
      row[4] = 3
    if((x == 3) | (x == 4)):
      row[4] = 25 
    if(x == 5):
      row[4] = 5 
    if(x == 6):
      row[4] = 6 
    if(x == 7):
      row[4] = 7 
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
    if(x == 2):
      row[4] = 3
    if(x == 3):
      row[4] = 10
    if(x == 4):
      row[4] = 10
    if((x == 5) | (x == 6)):
      row[4] = 11
    if((x == 7)|(x == 8)):
      row[4] = 12
  elif(y==4):
    if((x == 0) | (x == 1)):
      row[4] = 13
    if(x == 3):
      row[4] = 14
    if(x == 4):
      row[4] = 14
    if(x == 5):
      row[4] = 15
    if(x == 6):
      row[4] = 16
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
    if(x == 5):
      row[4] = 15
    if(x == 6):
      row[4] = 22
    if(x == 7):
      row[4] = 22
    if(x == 8):
      row[4] = 23
  elif(y == 6):
    if(x == 1):
      row[4] = 18
    if(x == 2):
      row[4] = 19
    if(x == 3):
      row[4] = 20
    if(x == 4):
      row[6] = 21
    if((x == 7) | (x == 8)):
      row[4] = 24
  elif(y == 7):
    row[4] = 24
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

data.to_csv(output_file, header=None,index=None)
