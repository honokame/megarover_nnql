# move_base
[move_base.cpp](https://github.com/honokame/megarover_nnql/blob/master/navigation/move_base/src/move_base.cpp)：使用していたプログラム  

`700-705行目`：gazeboでの位置取得のための変数宣言  
`708-713行目`：サービスの定義、移動距離を記録するファイルのパス  
`715-732行目`：移動距離の計算  
`980行目`：フロンティア法とRRTの速度送信回数の設定、論文は350、行動回数1回の時は700とした  
