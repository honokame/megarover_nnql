# -*- coding: utf-8 -*-
import pandas as pd
import random
import sys


path = 'data/'
feature_file = path + 'feature' + sys.argv[1] + '.csv'
statusclass_file = path + 'statusclass' + sys.argv[1] + '_26.csv'
scan = pd.read_csv(feature_file, header=None)
status = pd.read_csv(statusclass_file, header=None)
len_data = len(status) #行数
print(len_data)
status = status.append(status[0:50])
scan = scan.append(scan[0:50])

new_scan = path + 'train' + sys.argv[1] + '_feature.csv'
new_status = path + 'train' + sys.argv[1] + '_statusclass.csv'
no_file = path + 'no' + sys.argv[1] + '.txt'
nofile = open(no_file, 'w')

no = random.randint(0,len_data-1)
big_status = status[no:no+50] 
big_scan = scan[no:no+50]
nofile.write(str(no))
nofile.write('\n')

count = 399
if(sys.argv[1] == 'test1'):
  count = 199
  new_scan = path + sys.argv[1] + '_feature.csv'
  new_status = path + sys.argv[1] + '_statusclass.csv'
  no_file = path + 'no' + sys.argv[1] + '.txt'

for i in range(count): #199 or 399
  no = random.randint(0,len_data-1) #(a,b):a以上b以下
  small_status = status[no:no+50] 
  small_scan = scan[no:no+50]
  big_status = big_status.append(small_status)
  big_scan = big_scan.append(small_scan)
  nofile.write(str(no))
  nofile.write('\n')
big_status.to_csv(new_status, header=None)
big_scan.to_csv(new_scan, header=None, index=None)
