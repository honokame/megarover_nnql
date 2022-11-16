/*
toragi_pkg1パッケージのnode5の実装
publisher:node5,send messeage to topic3
topic1(String)
*/

#include "ros/ros.h"
#include "toragi_pkg1/Human.h"
#include <iostream>
using namespace std;

int main(int argc,char **argv){
  ros::init(argc, argv, "node5"); //ros init

  ros::NodeHandle nh; //ノードハンドルnh
  ros::Publisher pub = nh.advertise<toragi_pkg1::Human>("topic3", 1000); //Publisher,topic

  ros::Rate loop_rate(0.2); //ループの実行頻度(Hz),5s

  while(ros::ok()){
    toragi_pkg1::Human msg;
    cout << "Enter Name[str]:" << endl;
    cin >> msg.name;
    cout << "Enter Height[int/cm]:" << endl;
    cin >> msg.height;
    cout << "Enter Weight[float/kg]:" << endl;
    cin >> msg.weight;

    ROS_INFO("published info:");
    ROS_INFO("name: %s height: %d weight: %.2f",msg.name.c_str(),msg.height,msg.weight); 

    pub.publish(msg); //node5がtopic3にデータを出版

    ros::spinOnce(); //ROSのイベント発生やシグナル発行などがあるか確認、なければ処理を続ける
    loop_rate.sleep(); //設定した時間がすぎるまでスリープ
  }
  return 0;
}

