/*
toragi_pkg1パッケージのnode6の実装
publisher:node6,send messeage to topic1
Subscriber:node6,receive messeage from topic3
topic1(String)
topic3(Human)
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "toragi_pkg1/Human.h"

std::string human_name;
float bmi = 0.0;
int flag = 0;

//データを購読したときに実行されるコールバック関数
//name,height,weightを受信し、BMIを計算
void topic3Callback(const toragi_pkg1::Human::ConstPtr &msg){ 
  human_name = msg->name;
  ROS_INFO("I heard %s's height and weight", human_name.c_str());
  bmi = msg->weight / ((msg->height / 100.0) * (msg->height / 100.0));
  flag = 1; //finish
}

int main(int argc,char **argv){
  ros::init(argc, argv, "node6"); //ros init

  ros::NodeHandle nh; //ノードハンドルnh
  ros::Subscriber sub = nh.subscribe("topic3", 1000, topic3Callback); //Subscriber,topic,callback
  ros::Publisher pub = nh.advertise<std_msgs::String>("topic1", 1000); //Publisher,topic

  ros::Rate loop_rate(0.2); //ループの実行頻度(Hz),5s

  while(ros::ok()){
    if(flag){
      std_msgs::String msg;
      std::stringstream ss;
      ss << human_name << "'s BMI is'" << bmi;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());
      pub.publish(msg); //to topic1

      flag = 0; //reset
    }

    ros::spinOnce(); //ROSのイベント発生やシグナル発行などがあるか確認、なければ処理を続ける
    loop_rate.sleep(); //設定した時間がすぎるまでスリープ
  }
  return 0;
}

