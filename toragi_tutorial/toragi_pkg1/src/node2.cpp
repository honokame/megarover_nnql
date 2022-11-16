/*
torsgi_pkg1パッケージのnode2の実装
subscriber:node2,receive messeage from topic1
topic1(String)
*/

#include "ros/ros.h"
#include "std_msgs/String.h"

//データを購読したときに実行されるコールバック関数
//ノード名を取得し、ノード名と購読した文字列を出力
void topic1Callback(const std_msgs::String::ConstPtr &msg){ 
  std::string node_name = ros::this_node::getName();
  ROS_INFO("%s heard: [%s]", node_name.c_str(), msg->data.c_str());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "node2"); //ros init

  ros::NodeHandle nh; //ノードハンドルnh
  ros::Subscriber sub = nh.subscribe("topic1", 1000, topic1Callback); //Subscriber,topic,collback

  ros::spin(); //ROSのイベント発生やシグナル発行などを無限に待つ
  
  return 0;
}

