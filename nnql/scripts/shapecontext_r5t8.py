# -*- coding: utf-8 -*-
# python shapecontext_r5t8.py ファイル番号 何行目
import math
import pandas as pd
import csv
import sys 

where = int(sys.argv[2]) #何行目からはじめるか 

input_file = 'scan' + sys.argv[1] + '.csv' #入力ファイル
scan = pd.read_csv(input_file, header=None) #入力ファイル読み込み

#１列目の時間を取り除く
scan = scan.drop(scan.columns[[0, 0]], axis=1)
sh = scan.shape
scan.columns = range(sh[1])

output_file = 'feature' + sys.argv[1] +'_' + sys.argv[2] + '.csv' #出力ファイル
f_file = open(output_file, 'w') 

pi = math.pi

x = [] #ロボットを中心にした座標
y = []
r = [] #障害物までの距離、スキャンデータの値
theta = [] #データ点の角度
deg = 0
rad = 180 / float(1008) #スキャンデータの数-1、データ間の角度
node = [56, 168, 280, 392, 504, 616, 728, 840, 952] #代表点、グラフのノードになる
pos_x = [] #代表点の座標
pos_y = []
diff_x = [] #代表点との座標の差
diff_y = []
distance = [] #代表点との距離
area = [] #どの領域か
feature = [] #ヒストグラム、特徴量になる

#データ点がある角度を求める
for data in range(1009): #スキャンデータの数
  deg = rad*data
  theta.insert(0, deg*pi/float(180))

#指定した行を含めた100行文の特徴量を求める
for data in range(1000):
  r = scan[data+where:data+where+1] #指定した行を含めた100行文のデータ
  x = []
  y = []
  #ロボットを中心にしたxy座標を求める、極座標＞直行座標
  for i in range(len(theta)):
    tmp_x = r[i] * math.cos(theta[i]) #x=rcosθ
    tmp_y = r[i] * math.sin(theta[i]) #y=rsinθ
    x.append(tmp_x)
    y.append(tmp_y)

  #代表点のxy座標を抜き出す
  pos_x = [x[node[0]], x[node[1]], x[node[2]], x[node[3]],
        x[node[4]], x[node[5]], x[node[6]], x[node[7]], x[node[8]]]
  pos_y = [y[node[0]], y[node[1]], y[node[2]], y[node[3]],
        y[node[4]], y[node[5]], y[node[6]], y[node[7]], y[node[8]]]

  #代表点ごとに特徴量を求める
  for n in range(9): #代表点の数
    area = []
    #代表点との距離、角度を求めてどの領域の点か求める
    for j in range(len(x)): #スキャンデータの数
      tmp_x = x[j] - pos_x[n] #代表点との座標の差
      tmp_y = y[j] - pos_y[n]
      tmp_d = ((tmp_x**2) + (tmp_y**2))**0.5 #代表点との座標差から距離を求める
      tmp_t = math.degrees(math.atan2(tmp_y, tmp_x)) #代表点から見た角度を求める

      #代表点から見た角度ごとに領域番号をつける
      if ((tmp_t >= 0) & (tmp_t < 45)):
        tmp_theta = 1
      elif (tmp_t >= 45) & (tmp_t < 90):
        tmp_theta = 2
      elif (tmp_t >= 90) & (tmp_t < 135):
        tmp_theta = 3
      elif (tmp_t >= 135) & (tmp_t < 180):
       tmp_theta = 4
      elif (tmp_t >= -180) & (tmp_t < -135):
        tmp_theta = 5
      elif (tmp_t >= -135) & (tmp_t < -90):
        tmp_theta = 6
      elif (tmp_t >= -90) & (tmp_t < -45):
        tmp_theta = 7
      else:
        tmp_theta = 8
      
      if(str(tmp_d) == "inf"): #スキャンデータがinfだったとき用のエラー処理
        tmp_dis = 100 #外れ値として与えるだけなので被らなければ何でも良い
      else:
        tmp_dis = int(tmp_d)+1 #下記の領域番号の計算がしやすいように+1してる

      #領域番号割り当て、距離と上で求めた角度の領域番号から計算する
      if (tmp_dis == 1):
        tmp_b = tmp_theta * 5-4
      elif (tmp_dis == 2):
        tmp_b = tmp_theta * 5-4
      elif (tmp_dis == 3):
        tmp_b = tmp_theta * 5-3
      elif (tmp_dis == 4):
        tmp_b = tmp_theta * 5-3
      elif (tmp_dis == 5):
        tmp_b = tmp_theta * 5-2
      elif (tmp_dis == 6):
        tmp_b = tmp_theta * 5-2
      elif (tmp_dis == 7):
        tmp_b = tmp_theta * 5-1
      elif (tmp_dis == 8):
        tmp_b = tmp_theta * 5-1
      elif (tmp_dis == 9):
        tmp_b = tmp_theta * 5
      elif (tmp_dis == 10):
        tmp_b = tmp_theta * 5
      else:
        tmp_b = 45 #エラー処理、被らなければ何でも良い、r5t8は40まで領域番号があるのでそれ以上の数値

      area.append(tmp_b) #領域番号をデータごとに格納
    
    #各領域番号の数を数える、featureがヒストグラム
    for c in range(40): #領域番号の最高値
      if (c == 0): #代表点を含んでいるので1引く
        feature.append(area.count(c+1)-1)
      else:
        feature.append(area.count(c+1))
  
  #ファイル書き込み
  f_file.write(str(data+where)+',')
  writer = csv.writer(f_file)
  writer.writerow(feature)
  
  feature = [] #初期化
