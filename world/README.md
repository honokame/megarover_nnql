# world
## worlds
地図から作成した仮想環境  
使用したいworld名を[nnql.launch](https://github.com/honokame/megarover_nnql/tree/master/nnql/launch)等に書く　　

## models
gazeboで使用できる3Dモデル  
モデルを複数配置（地面、壁）したものがworldとなる

## launch/gmapping_offline.launch
gmappingを用いた地図作成はリアルタイムだけでなく、保存したbagファイルを用いる方法（オフライン）でも行える  
roslaunchで実行した後に、rosbag playでbagファイルの再生を行う 
