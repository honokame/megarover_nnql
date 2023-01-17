# -*- coding: utf-8 -*-

import math
import pandas as pd
import csv
import sys 

#path = 'data/'
#scan = pd.read_csv(path +'scan7.csv', header=None)
where = int(sys.argv[2])
input_file = 'scan' + sys.argv[1] + '.csv'
output_file = 'feature' + sys.argv[1] +'_' + sys.argv[2] + '.csv'
scan = pd.read_csv(input_file, header=None)
scan = scan.drop(scan.columns[[0, 0]], axis=1)
sh = scan.shape
scan.columns = range(sh[1])
# scan
f_file = open(output_file, 'w') 

pi = math.pi

x = []
y = []
r = []
theta = []
deg = 0
rad = 180 / float(1010)
node = [57, 169, 281, 393, 505, 617, 729, 841, 953]
pos_x = []
pos_y = []
diff_x = []
diff_y = []
distance = []
tan = []
tan_theta = []
area = []
feature = []
all_feature = []

for data in range(1011):
  deg = rad*data
  theta.insert(0, deg*pi/float(180))

for data in range(100):
  #print(data+where)  # 消してもいい
  r = scan[data+where:data+where+1]
  x = []
  y = []
  for i in range(len(theta)):
    tmp_x = r[i] * math.cos(theta[i])
    tmp_y = r[i] * math.sin(theta[i])
    x.append(tmp_x)
    y.append(tmp_y)

  pos_x = [x[node[0]], x[node[1]], x[node[2]], x[node[3]],
        x[node[4]], x[node[5]], x[node[6]], x[node[7]], x[node[8]]]
  pos_y = [y[node[0]], y[node[1]], y[node[2]], y[node[3]],
        y[node[4]], y[node[5]], y[node[6]], y[node[7]], y[node[8]]]
    #diff_x = []
    #diff_y = []
    #distance = []
    #tan = []
  for n in range(9):
    # diff_x = []
    #diff_y = []
    #distance = []
    #tan = []
    area = []
    for j in range(len(x)):
      tmp_x = x[j] - pos_x[n]
      tmp_y = y[j] - pos_y[n]
      tmp_d = ((tmp_x**2) + (tmp_y**2))**0.5
      tmp_t = math.degrees(math.atan2(tmp_y, tmp_x))
      #diff_x.append(tmp_x)
      #diff_y.append(tmp_y)
      #distance.append(tmp_d)
      #tan.append(tmp_t)

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
      #tan_theta.append(tmp_theta)
      tmp_dis = int(tmp_d)+1
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
        tmp_b = 45
        print(tmp_dis)
        print(data+where)# error
      area.append(tmp_b)
    for c in range(40):
      if (c == 0):
        feature.append(area.count(c+1)-1)
      else:
        feature.append(area.count(c+1))
    #a_feature.append(feature)
  f_file.write(str(data+where)+',')
  writer = csv.writer(f_file)
  writer.writerow(feature)
  feature = []

#feature_list = path + 'feature3.csv'
#feature_list = 'feature7.csv'
#df = pd.DataFrame(all_feature)
#df.to_csv(feature_list, index=None)
# df
