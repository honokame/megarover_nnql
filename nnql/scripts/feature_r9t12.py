# -*- coding: utf-8 -*-

import math
import pandas as pd
import csv

def shapecontext(scan):
  pi = math.pi
  output_file = 'temp_feature.csv'
  f_file = open(output_file, 'w') 

  x = []
  y = []
  r = scan
  theta = []
  deg = 0
  rad = 180 / float(1008)
  node = [56, 168, 280, 392, 504, 616, 728, 840, 952]
  pos_x = []
  pos_y = []
  diff_x = []
  diff_y = []
  distance = []
  tan = []
  tan_theta = []
  area = []
  feature = []

  for data in range(1009):
    deg = rad*data
    theta.insert(0, deg*pi/float(180))

  for i in range(len(theta)):
    tmp_x = r[i] * math.cos(theta[i])
    tmp_y = r[i] * math.sin(theta[i])
    x.append(tmp_x)
    y.append(tmp_y)

  pos_x = [x[node[0]], x[node[1]], x[node[2]], x[node[3]],
        x[node[4]], x[node[5]], x[node[6]], x[node[7]], x[node[8]]]
  pos_y = [y[node[0]], y[node[1]], y[node[2]], y[node[3]],
        y[node[4]], y[node[5]], y[node[6]], y[node[7]], y[node[8]]]
  for n in range(9):
    area = []
    for j in range(len(x)):
      tmp_x = x[j] - pos_x[n]
      tmp_y = y[j] - pos_y[n]
      tmp_d = ((tmp_x**2) + (tmp_y**2))**0.5
      tmp_t = math.degrees(math.atan2(tmp_y, tmp_x))

      if ((tmp_t >= 0) & (tmp_t < 30)):
        tmp_theta = 1
      elif (tmp_t >= 30) & (tmp_t < 60):
        tmp_theta = 2
      elif (tmp_t >= 60) & (tmp_t < 90):
        tmp_theta = 3
      elif (tmp_t >= 90) & (tmp_t < 120):
        tmp_theta = 4
      elif (tmp_t >= 120) & (tmp_t < 150):
        tmp_theta = 5
      elif (tmp_t >= 150) & (tmp_t < 180):
        tmp_theta = 6
      elif (tmp_t >= -180) & (tmp_t < -150):
        tmp_theta = 7
      elif (tmp_t >= -150) & (tmp_t < -120):
        tmp_theta = 8
      elif (tmp_t >= -120) & (tmp_t < -90):
        tmp_theta = 9
      elif (tmp_t >= -90) & (tmp_t < -60):
        tmp_theta = 10
      elif (tmp_t >= -60) & (tmp_t < -30):
        tmp_theta = 11
      else:
        tmp_theta = 12
    
      if(str(tmp_d) == "inf"):
        tmp_dis = 100
      elif(str(tmp_d) == "nan"):
        tmp_dis = 100
      else:
        tmp_dis = int(tmp_d)+1

      if (tmp_dis == 1):
        tmp_b = tmp_theta * 9-8
      elif (tmp_dis == 2):
        tmp_b = tmp_theta * 9-7
      elif (tmp_dis == 3):
        tmp_b = tmp_theta * 9-6
      elif (tmp_dis == 4):
        tmp_b = tmp_theta * 9-5
      elif (tmp_dis == 5):
        tmp_b = tmp_theta * 9-4
      elif (tmp_dis == 6):
        tmp_b = tmp_theta * 9-3
      elif (tmp_dis == 7):
        tmp_b = tmp_theta * 9-2
      elif (tmp_dis == 8):
        tmp_b = tmp_theta * 9-1
      elif (tmp_dis == 9):
        tmp_b = tmp_theta * 9
      else:
        tmp_b = 110
    
      area.append(tmp_b)
    for c in range(108):
      if (c == 0):
        feature.append(area.count(c+1)-1)
      else:
        feature.append(area.count(c+1))
  
  f_file.write(str(100)+',')
  writer = csv.writer(f_file)
  writer.writerow(feature)
  #print(feature)
  #print('calc feature')
  feature = []
