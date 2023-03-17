## wall_follower
[follow_wall.py](https://github.com/honokame/megarover_nnql/blob/master/wall_follower/scripts/follow_wall.py)：使用していたプログラム  

`51-57行目`：LRFの範囲設定  
`79行目`：壁との距離、閾値  
`221-248行目`：並進・回転速度の設定  
`250-266行目`：現在位置を求める関数 、実機用に書き換える必要がある  
`286行目`：移動距離を記録するファイルのパス  
`323-325行目`：移動距離の計算  
`329行目`：速度送信回数の設定、論文は1500、行動回数1回の時は3000とした

元コード：[rfzeg/wall_follower](https://github.com/rfzeg/wall_follower)
