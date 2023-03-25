# RRTstar-planner-for-ROS-navigation
This is a ROS package of RRT* Path Planner, This planner can plugin the ROS navigation packages as global planner

This is the video of implementation
https://www.youtube.com/watch?v=FsZ9b6fsQUg&t=3s

---
RRT star plugin for ros-melodic

# 追記
元コードが動作せず、元コードを改良したWhiteCriのプログラムをベースにして、さらに改良した  
[old_rrtstarplan.cpp](https://github.com/honokame/megarover_nnql/blob/master/RRTstar-planner-for-ROS-navigation/rrtstar_planner/src/old_rrtstarplan.cpp)：卒論時に使用していたプログラム  
[rrtstarplan.cpp](https://github.com/honokame/megarover_nnql/blob/master/RRTstar-planner-for-ROS-navigation/rrtstar_planner/src/rrtstarplan.cpp)：卒論提出後に改良したプログラム、乱数のシード値を固定し同じ木を生成する  

元コードを改良したコード：[WhiteCri/RRTstar-planner-for-ROS-navigation](https://github.com/WhiteCri/RRTstar-planner-for-ROS-navigation)  
元コード：[Zhi29/RRTstar-planner-for-ROS-navigation](https://github.com/Zhi29/RRTstar-planner-for-ROS-navigation)
