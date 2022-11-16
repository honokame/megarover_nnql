import pandas as pd
from sklearn.model_selection import train_test_split
import numpy as np

#全データの結合
all = pd.DataFrame(index=[], columns=[])
for num in range(1,25):
  data = pd.read_csv(str(num) + '.csv', header=None)
  print(len(data))
  all = pd.concat([all, data])
print(len(all))

#訓練データとテストデータに分割して保存
train, test = train_test_split(all, train_size=0.9, test_size=0.1, shuffle=True)
train.to_csv('train.txt', header=None, index=None, sep=" ")
test.to_csv('test.txt', header=None, index=None, sep=" ")

#csv
#train.to_csv('train.csv', header=None)
#test.to_csv('test.csv', header=None)

  
