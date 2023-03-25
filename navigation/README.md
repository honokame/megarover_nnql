ROS Navigation Stack
====================

A 2D navigation stack that takes in information from odometry, sensor
streams, and a goal pose and outputs safe velocity commands that are sent
to a mobile base.

 * AMD64 Debian Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__navigation__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__navigation__ubuntu_bionic_amd64__binary/)

Related stacks:

 * http://github.com/ros-planning/navigation_msgs (new in Jade+)
 * http://github.com/ros-planning/navigation_tutorials
 * http://github.com/ros-planning/navigation_experimental

For discussion, please check out the
https://groups.google.com/group/ros-sig-navigation mailing list.

# 追記  
使用しているパッケージ  
`amcl`：自己位置推定  
`clear_costmap_recovery`：recovery動作、ロボット周辺のコストマップをクリア  
`costmap_2d`：コストマップの管理  
`dwa_local_planner`：DWA、障害物回避  
`global_planner`：A*  
`map_server`：地図の配信  
`move_base`：メインプログラム  
`move_slow_and_clear`：recovery動作、ロボットの速度を落とす、コストマップをクリア  


