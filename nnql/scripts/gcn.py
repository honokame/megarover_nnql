#!/usr/bin/env python3

import dgl
import torch 
from dgl.nn.pytorch import GraphConv
import torch.nn as nn
import networkx as nx
import numpy as np
import torch.nn.functional as F
import torch.optim as optim 
from torch.utils.data import DataLoader 
from dgl.data import MiniGCDataset
import dgl.function as fn
import matplotlib.pyplot as plt
import pandas as pd

#訓練データ読み込み
train_data = np.loadtxt(fname="data/r9t12/train347_feature.csv",  dtype="int",  delimiter=",")
train_place = np.loadtxt(fname="data/r9t12/train347_statusclass.csv", dtype="int", delimiter=",")

#訓練データグラフ作成
trainset = []
for data, place in zip(train_data, train_place):
  label = place[7]-1#5,6,7                            # ground truth
  data = data.tolist()  
  #feature_list = [data[1:73],data[73:145],data[145:217],data[217:289],data[289:361],data[361:433],data[433:505],data[505:577],data[577:649]] #r9t8 
  feature_list = [data[1:109],data[109:217],data[217:325],data[325:433],data[433:541],data[541:649],data[649:757],data[757:865],data[865:973]] #r9t12
  #feature_list = [data[1:41],data[41:81],data[81:121],data[121:161],data[161:201],data[201:241],data[241:281],data[281:321],data[321:361]] #r5t8 
  #print(feature_list)
  #print(label)
  g = dgl.graph(([0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8], [1,0,2,1,3,2,4,3,5,4,6,5,7,6,8,7]))      # edge
  g.ndata['feat'] = torch.from_numpy(np.array( feature_list).astype(np.float32)).clone() 
  #print(g)
  trainset.append([g,label])
  
print("train_data:",len(trainset))
#nx.draw(g.to_networkx().to_undirected(), with_labels=True)
#plt.show()

#テストデータ読み込み
test_data = np.loadtxt(fname="data/r9t12/test1_feature.csv",  dtype="int",  delimiter=",")
test_place = np.loadtxt(fname="data/r9t12/test1_statusclass.csv", dtype="int", delimiter=",")

#テストデータグラフ作成
testset = []
for data, place in zip(test_data, test_place):
  label = place[7]-1                            # ground truth
  data = data.tolist() 
  #feature_list = [data[1:73],data[73:145],data[145:217],data[217:289],data[289:361],data[361:433],data[433:505],data[505:577],data[577:649]] #r9t8 
  feature_list = [data[1:109],data[109:217],data[217:325],data[325:433],data[433:541],data[541:649],data[649:757],data[757:865],data[865:973]]#r9t12
  #feature_list = [data[1:41],data[41:81],data[81:121],data[121:161],data[161:201],data[201:241],data[241:281],data[281:321],data[321:361]] #r5t8  
  #print(feature_list)
  #print(label)
  g = dgl.graph(([0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8], [1,0,2,1,3,2,4,3,5,4,6,5,7,6,8,7]))      # edge
  g.ndata['feat'] = torch.from_numpy(np.array( feature_list).astype(np.float32)).clone() 
  #print(g)
  testset.append([g,label])
  
print("test_data:",len(testset))
#nx.draw(g.to_networkx().to_undirected(), with_labels=True)
#plt.show()

####   GCN   ########################################################

def collate(samples):
    # The input `samples` is a list of pairs
    #  (graph, label).
    graphs, labels = map(list, zip(*samples))
    batched_graph = dgl.batch(graphs)
    return batched_graph, torch.tensor(labels)

msg = fn.copy_src(src='h', out='m')

def reduce(nodes):
    """Take an average over all neighbor node features hu and use it to
    overwrite the original node feature."""
    
    accum = torch.sum(nodes.mailbox['m'], 1)
    #print(accum)
    return {'h': accum}

class NodeApplyModule(nn.Module):
    """Update the node feature hv with ReLU(Whv+b)."""
    def __init__(self, in_feats, out_feats, activation):
        super(NodeApplyModule, self).__init__()
        self.linear = nn.Linear(in_feats, out_feats)
        self.activation = activation

    def forward(self, node):
        h = self.linear(node.data['h'])
        h = self.activation(h)
        return {'h' : h}

class GCN(nn.Module):
    def __init__(self, in_feats, out_feats, activation):
        super(GCN, self).__init__()
        self.apply_mod = NodeApplyModule(in_feats, out_feats, activation)
    def forward(self, g, feature):
        # Initialize the node features with h.
        g.ndata['h'] = feature
        g.update_all(msg, reduce)
        g.apply_nodes(func=self.apply_mod)
        return g.ndata.pop('h')

class Classifier(nn.Module):
    def __init__(self, in_dim, hidden_dim, n_classes):
        super(Classifier, self).__init__()

        self.layers = nn.ModuleList([
            GCN(in_dim, hidden_dim, F.relu),
            GCN(hidden_dim, hidden_dim, F.relu)])
        self.classify = nn.Linear(hidden_dim, n_classes)

    def forward(self, g):
        # For undirected graphs, in_degree is the same as
        # out_degree.
        h = g.ndata['feat']
        for conv in self.layers:
            h = conv(g, h)
        g.ndata['h'] = h
        hg = dgl.mean_nodes(g, 'h')
        return self.classify(hg)

####  train  ###############################

data_loader = DataLoader(
    trainset, 
    batch_size=30, 
    shuffle=True,
    collate_fn=collate)

feature = 108  #r9t12:108 r9t8:72 r5:t8:40
classes = 312  #26,208,312

# Create model
model = Classifier(feature, 256, classes)    #特徴ベクトルの次元、場所クラス数
loss_func = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(),lr=0.001) #0.001
model.train()

epoch_losses = []
for epoch in range(50):
    epoch_loss = 0
    for iter, (bg, label) in enumerate(data_loader):
        prediction = model(bg)
        loss = loss_func(prediction, label)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        epoch_loss += loss.detach().item()
    epoch_loss /= (iter + 1)
    print('Epoch {}, loss {:.4f}'.format(epoch, epoch_loss))
    #if epoch_loss<0.05:
    #  break
    epoch_losses.append(epoch_loss)
    torch.save(model.state_dict(),'data/r9t12/model347_312_e50.pth')

####  test ###################################
model.eval()
test_X, test_Y = map(list, zip(*testset))
test_bg = dgl.batch(test_X)
test_Y = torch.tensor(test_Y).float().view(-1, 1)
probs_Y = torch.softmax(model(test_bg), 1)
sampled_Y = torch.multinomial(probs_Y, 1)
argmax_Y = torch.max(probs_Y, 1)[1].view(-1, 1)

#f=open('data/result34.txt','w')
#for arg in argmax_Y:
#  f.write(str(arg))
#  f.write('\n')
#f.close()

print('Accuracy of sampled predictions on the test set: {:.4f}%'.format(
    (test_Y == sampled_Y.float()).sum().item() / len(test_Y) * 100))
print('Accuracy of argmax predictions on the test set: {:4f}%'.format(
    (test_Y == argmax_Y.float()).sum().item() / len(test_Y) * 100))  


