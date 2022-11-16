/*
torsgi_pkg1パッケージのnode1の実装
publisher:node1,send messeage to topic1
topic1(String)
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc,char **argv){
  ros::init(argc, argv, "node3"); //ros init

  ros::NodeHandle nh; //ノードハンドルnh
  ros::Publisher pub1 = nh.advertise<std_msgs::String>("topic1", 1000); //Publisher,topic
  ros::Publisher pub2 = nh.advertise<std_msgs::String>("topic2", 1000); //Publisher,topic


  ros::Rate loop_rate(0.2); //ループの実行頻度(Hz),5s
  int count = 0;
  std::string node_name = ros::this_node::getName(); //get nodename

  while(ros::ok()){
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello from " << node_name << " count=" << count;
    msg.data = ss.str();

    ROS_INFO("%s",msg.data.c_str()); //debug

    pub1.publish(msg); //node1がtopic1に作成した文字列を出版
    pub2.publish(msg);

    ros::spinOnce(); //ROSのイベント発生やシグナル発行などがあるか確認、なければ処理を続ける
    loop_rate.sleep(); //設定した時間がすぎるまでスリープ

    if((argc >= 2) && (count >= atoi(argv[1]))){
      pub1.shutdown();
      pub2.shutdown();
      ros::shutdown();
    }
    else{
      ++count;
    }
  }
  return 0;
}

