#ifndef rrtstarplan_h
#define rrtstarplan_h
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>

using std::string;
using namespace std;
namespace rrtstar_planner{
  class RRT : public nav_core::BaseGlobalPlanner{
    public:
      RRT();
      RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      RRT(double input_PosX, double input_PosY);

      struct rrtNode{
        int nodeID;
        double posX;
        double posY;
        int parentID;
        double cost;
        int leaf;
        int leaf_cost;
        vector<int> children;
      };

      vector<rrtNode> getTree();
      vector<rrtNode> getNearestNeighbor(int tempNodeID);//新しく追加した関数
      void setTree(vector<rrtNode> input_rrtTree);
      int getTreeSize();

      void addNewNode(rrtNode node);
      void deleteNewNode();
      rrtNode removeNode(int nodeID);
      rrtNode getNode(int nodeID);

      double getPosX(int nodeID);
      double getPosY(int nodeID);
      void setPosX(int nodeID, double input_PosX);
      void setPosY(int nodeID, double input_PosY);

      rrtNode getParent(int nodeID);
      void setParentID(int nodeID, int parentID);

      void addChildID(int nodeID, int childID);
      vector<int> getChildren(int nodeID);
      int getChildrenSize(int nodeID);

      int getNearestNodeID(double X, double Y);
      vector<int> getRootToEndPath(int endNodeID);

      bool judgeangle1(RRT myRRT, rrtNode tempNode);

      void initializeMarkers(visualization_msgs::Marker &sourcePoint,
          visualization_msgs::Marker &goalPoint,
          visualization_msgs::Marker &randomPoint,
          visualization_msgs::Marker &rrtTreeMarker,
          visualization_msgs::Marker &rrtTreeMarker1,
          visualization_msgs::Marker &rrtTreeMarker2,
          visualization_msgs::Marker &finalPath);

      void initNode(RRT::rrtNode &newNode, const geometry_msgs::PoseStamped& start);

      void generateTempPoint(RRT::rrtNode &tempNode,costmap_2d::Costmap2D* costmap_,unsigned int seed,double startX,double startY);
      bool judgeangle1(RRT::rrtNode tempNode);
      bool addNewPointtoRRT(RRT::rrtNode &tempNode, double rrtStepSize);
      bool checkIfInsideBoundary(RRT::rrtNode &tempNode);
      bool checkIfOutsideObstacles(RRT::rrtNode tempNode);
      int getCost(RRT::rrtNode tempNode);
      void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode);
      bool checkNodetoGoal(double X, double Y, RRT::rrtNode &tempNode);
      void setFinalPathData(vector< vector<int> > &rrtPaths,  int i, visualization_msgs::Marker &finalpath);

      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      bool makePlan(const geometry_msgs::PoseStamped& start,
          const geometry_msgs::PoseStamped& goal,
          std::vector<geometry_msgs::PoseStamped>& plan
          );

      vector<rrtNode> rrtTree;
      ros::NodeHandle pn;

    private:
      double getEuclideanDistance(double sourceX, double sourceY, double destinationX, double destinationY);
      bool initialized_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      base_local_planner::WorldModel* world_model_;
      std::vector<geometry_msgs::Point> footprint;
      ros::Publisher rrt_publisher;
  };
};

#endif

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include <pluginlib/class_list_macros.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <cstddef>

#define success false
#define running true
#define PI 3.1415926

// プランナーをプラグインに登録(class,interface class)
PLUGINLIB_EXPORT_CLASS(rrtstar_planner::RRT, nav_core::BaseGlobalPlanner)

  namespace rrtstar_planner{
    bool status=running;

    using namespace std;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;
    using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    using costmap_2d::LETHAL_OBSTACLE;

    RRT::RRT(){ //はじめに実行、initializeを呼び出す
      ROS_INFO("init1");
    }

    RRT::RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros){ //呼び出されない
      initialize(name, costmap_ros);
      ROS_INFO("init0");
    }

    void RRT::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
      ROS_INFO("init2");
      rrt_publisher = pn.advertise<visualization_msgs::Marker> ("path_planner_rrt",1000); //マーカーのPublisher
      if(!initialized_){
        costmap_ros_ = costmap_ros; //costmap_ros_の初期化
        costmap_ = costmap_ros_->getCostmap(); //すべてのレイヤーのcostmap_rosの更新を受け取る
        footprint = costmap_ros_->getRobotFootprint(); //ロボットの現在のフットプリントを取得

        // initialize other planner parameters
        /*ros::NodeHandle private_nh("~/" + name);
          private_nh.param("step_size", step_size_, costmap_->getResolution());
          private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);*/
        world_model_ = new base_local_planner::CostmapModel(*costmap_);
        initialized_ = true;
      }else{
        ROS_WARN("This planner has already been initialized... doing nothing");
      }
    }

    //ノードの初期化、はじめのノードを現在位置にし、rrtTreeに追加
    void RRT::initNode(RRT::rrtNode &newNode, const geometry_msgs::PoseStamped& start){
      ROS_INFO("init4");
      newNode.posX=start.pose.position.x; //現在位置
      newNode.posY=start.pose.position.y;
      newNode.parentID = 0;
      newNode.nodeID = 0;
      newNode.cost=0;
      newNode.leaf = 1; //leafなら1
      newNode.leaf_cost = 0;
      rrtTree.push_back(newNode);
    }

    // Returns the current RRT tree @return RRT Tree
    vector<RRT::rrtNode> RRT::getTree(){
      return rrtTree;
    }

    vector<RRT::rrtNode> RRT::getNearestNeighbor(int tempNodeID){//特定の範囲内で新しいノードの最も近い隣人を見つける
      double win_r=0.15;
      vector<RRT::rrtNode> rrtNeighbor;
      for(int i=0;i<getTreeSize();i++){
        if (getEuclideanDistance(getPosX(tempNodeID),getPosY(tempNodeID),getPosX(i),getPosY(i))<=win_r){//新しいノードとすべてのノード
          rrtNeighbor.push_back(getNode(i)); //隣接ノードを格納するための配列
        }
      }
      return rrtNeighbor;
    }

    // For setting the rrtTree to the inputTree @param rrtTree
    void RRT::setTree(vector<RRT::rrtNode> input_rrtTree){
      rrtTree = input_rrtTree;
    }

    // 現在のrrtTreeの大きさを返す
    int RRT::getTreeSize(){
      return rrtTree.size();
    }

    // adding a new node to the rrt Tree
    void RRT::addNewNode(RRT::rrtNode node){
      rrtTree.push_back(node);
    }

    void RRT::deleteNewNode(){
      rrtTree.pop_back();
    }

    // removing a node from the RRT Tree @return the removed tree
    RRT::rrtNode RRT::removeNode(int id){
      RRT::rrtNode tempNode = rrtTree[id];
      rrtTree.erase(rrtTree.begin()+id);
      return tempNode;
    }

    // 指定したノードIDの構造体を返す
    RRT::rrtNode RRT::getNode(int id){
      return rrtTree[id];
    }

    // 近くにあるノードのIDを返す
    int RRT::getNearestNodeID(double X, double Y){
      int i, returnID;
      double distance = 9999, tempDistance;
      for(i=0; i<this->getTreeSize(); i++){ //rrtTreeの大きさを返す、ノード数
        tempDistance = getEuclideanDistance(X,Y, getPosX(i),getPosY(i)); //ノード間のユークリッド距離を取得
        if (tempDistance < distance){ //全探索して最も近いノードを求める
          distance = tempDistance;
          returnID = i;
        }
      }
      return returnID;
    }

    // ノードのx座標を返す
    double RRT::getPosX(int nodeID){
      return rrtTree[nodeID].posX;
    }

    // ノードのy座標を返す
    double RRT::getPosY(int nodeID){
      return rrtTree[nodeID].posY;
    }

    // set X coordinate of the given node
    void RRT::setPosX(int nodeID, double input_PosX){
      rrtTree[nodeID].posX = input_PosX;
    }

    // set Y coordinate of the given node
    void RRT::setPosY(int nodeID, double input_PosY){
      rrtTree[nodeID].posY = input_PosY;
    }

    // 親ノードのIDを返す
    RRT::rrtNode RRT::getParent(int id){
      return rrtTree[rrtTree[id].parentID];
    }

    // set parentID of the given node
    void RRT::setParentID(int nodeID, int parentID){
      rrtTree[nodeID].parentID = parentID;
    }

    // add a new childID to the children list of the given node
    void RRT::addChildID(int nodeID, int childID){
      rrtTree[nodeID].children.push_back(childID);
    }

    // returns the children list of the given node
    vector<int> RRT::getChildren(int id){
      return rrtTree[id].children;
    }

    // returns number of children of a given node
    int RRT::getChildrenSize(int nodeID){
      return rrtTree[nodeID].children.size();
    }

    // ノード間のユークリッド距離を返す
    double RRT::getEuclideanDistance(double sourceX, double sourceY, double destinationX, double destinationY){
      return sqrt(pow(destinationX - sourceX,2) + pow(destinationY - sourceY,2));
    }

    // はじめのノードからゴールまでのパスを返す
    vector<int> RRT::getRootToEndPath(int endNodeID){
      vector<int> path;
      path.push_back(endNodeID); //ゴールにしたIDをpathに格納

      // path.front():はじめの要素（ノードID）返す
      // path.begin():はじめの要素（ノードID）のイテレータを返す
      // path.insert():指定したイテレータに要素を挿入
      while(rrtTree[path.front()].nodeID != 0){ //はじめのノードまで親ノードを辿る
        //std::cout<<rrtTree[path.front()].nodeID<<endl;
        path.insert(path.begin(),rrtTree[path.front()].parentID); //pathの先頭に親ノードを格納
      }
      return path; //ゴールまでのpath
    }

    void RRT::initializeMarkers(visualization_msgs::Marker &sourcePoint,
        visualization_msgs::Marker &goalPoint,visualization_msgs::Marker &randomPoint,
        visualization_msgs::Marker &rrtTreeMarker,visualization_msgs::Marker &rrtTreeMarker1,
        visualization_msgs::Marker &rrtTreeMarker2,visualization_msgs::Marker &finalPath){//新しいタグを追加するだけでいい、rvizの話?

      //init headers
      sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = rrtTreeMarker1.header.frame_id    = rrtTreeMarker2.header.frame_id    =finalPath.header.frame_id    = "map";
      sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = rrtTreeMarker1.header.stamp       = rrtTreeMarker2.header.stamp       =finalPath.header.stamp       = ros::Time::now();
      sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = rrtTreeMarker1.ns                 = rrtTreeMarker2.ns                 =finalPath.ns                 = "map";
      sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = rrtTreeMarker1.action             = rrtTreeMarker2.action             =finalPath.action             = visualization_msgs::Marker::ADD;
      sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = rrtTreeMarker1.pose.orientation.w = rrtTreeMarker2.pose.orientation.w =finalPath.pose.orientation.w = 1.0;

      //setting id for each marker
      sourcePoint.id    = 0;
      goalPoint.id      = 1;
      randomPoint.id    = 2;
      rrtTreeMarker.id  = 3;
      finalPath.id      = 4;
      rrtTreeMarker1.id = 5;//新しく追加した
      rrtTreeMarker2.id = 6;

      //defining types
      rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
      rrtTreeMarker1.type                                   = visualization_msgs::Marker::LINE_LIST;
      rrtTreeMarker2.type                                   = visualization_msgs::Marker::LINE_LIST;
      finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
      sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

      //setting scale
      rrtTreeMarker.scale.x = 0.04;
      rrtTreeMarker1.scale.x= 0.02;
      rrtTreeMarker2.scale.x= 0.02;
      finalPath.scale.x     = 0.01;
      sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 2;
      sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 2;
      sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 1;

      //assigning colors
      sourcePoint.color.r   = 1.0f;
      goalPoint.color.g     = 1.0f;
      randomPoint.color.b   = 1.0f;

      rrtTreeMarker.color.r = 0.8f;
      rrtTreeMarker.color.g = 0.4f;

      rrtTreeMarker1.color.r = 0;
      rrtTreeMarker1.color.g = 1.0f;

      rrtTreeMarker2.color.r = 0;
      rrtTreeMarker2.color.g = 0;
      rrtTreeMarker2.color.b = 1.0f;

      finalPath.color.r = 0.2f;
      finalPath.color.g = 0.2f;
      finalPath.color.b = 1.0f;

      sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = rrtTreeMarker1.color.a = rrtTreeMarker2.color.a = finalPath.color.a = 1.0f;
    }

    //新しい仮ノードをtempNodeに追加する
    void RRT::generateTempPoint(RRT::rrtNode &tempNode,costmap_2d::Costmap2D* costmap_,unsigned int seed,double startX,double startY){
      //地図の大きさを設定、地図の大きさ-地図の原点
      double maxx = costmap_->getSizeInMetersX() - costmap_->getOriginX();
      double minx = costmap_->getOriginX();
      double maxy = costmap_->getSizeInMetersY() - costmap_->getOriginY();
      double miny = costmap_->getOriginY();

      //マップ内にランダムな仮ノードを決め
      //unsigned int a = time(NULL);
      srand(seed);
      //std::cout<<seed<<endl;
      double x = double(rand())/double(RAND_MAX)*(maxx - minx)+startX;// + minx;
      double y = double(rand())/double(RAND_MAX)*(maxy - miny)+startY;// + miny;
      //std::cout<<"Random X: "<<x <<endl<<"Random Y: "<<y<<endl;
      tempNode.posX = x;
      tempNode.posY = y;
      //std::cout<<"rand: "<<tempNode.posX<<" "<<tempNode.posY<<endl;
    }

    //角度のチェック、親ノードとの角度が180度のときは仮ノードを求め直す
    bool RRT::judgeangle1(RRT::rrtNode tempNode){
      int nearestNodeID = getNearestNodeID(tempNode.posX,tempNode.posY); //近いノードのIDを取得
      RRT::rrtNode nearestNode = getNode(nearestNodeID); //近いノードの構造体を取得、nearestNodeとする

      vector<double> n1,n2;
      if(nearestNode.parentID==0){ //はじめのノードの場合
        n1.push_back(tempNode.posX - nearestNode.posX); //近いノードとの座標差
        n1.push_back(tempNode.posY - nearestNode.posY);
        n2.push_back(0.0001);
        n2.push_back(0.0001);
      }else{
        n1.push_back(tempNode.posX - nearestNode.posX);
        n1.push_back(tempNode.posY - nearestNode.posY);
        n2.push_back(nearestNode.posX-getPosX(nearestNode.parentID));
        n2.push_back(nearestNode.posY-getPosY(nearestNode.parentID));
      }
      //近いノードとの角度を求める
      double phy = acos((n1[0]*n2[0]+n1[1]*n2[1])/(sqrt(n1[0]*n1[0]+n1[1]*n1[1])*sqrt(n2[0]*n2[0]+n2[1]*n2[1])));
      return (abs(phy)<PI/2) ? true : false; //角度180度未満ならTrue //3:60,2.5:70,2:90
    }

    bool RRT::addNewPointtoRRT(RRT::rrtNode &tempNode, double rrtStepSize){
      int nearestNodeID = getNearestNodeID(tempNode.posX,tempNode.posY);//近いノードのIDを取得

      RRT::rrtNode nearestNode = getNode(nearestNodeID);//近いノードの構造体を取得

      //仮ノードの位置決定、近いノードから1ステップ分伸ばす
      double theta = atan2(tempNode.posY - nearestNode.posY,tempNode.posX - nearestNode.posX);//近いノードとの角度
      tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));
      tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));

      //仮ノードの位置がマップ内かつ障害物なし
      if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(tempNode)){
        tempNode.parentID = nearestNodeID; //近いノードを親ノードにする
        tempNode.nodeID = rrtTree.size(); //一番最後のノードIDをノードIDにする
        tempNode.cost=sqrt(pow(nearestNode.posX - tempNode.posX,2) + pow(nearestNode.posY - tempNode.posY,2))+\
                      nearestNode.cost; //はじめのノードまでの距離をコストにする
        tempNode.leaf = 1; //仮ノードを葉に設定する
        tempNode.leaf_cost = getCost(tempNode);
        rrtTree[nearestNodeID].leaf = 0; //親ノードは葉でないので0
        //std::cout<<"tempNode.cost= "<<tempNode.cost<<endl;
        //std::cout<<"tempNode.posX: "<<tempNode.posX<<"  tempNode.posY"<<tempNode.posY<<endl;
        rrtTree.push_back(tempNode); //rrtTreeに仮ノードを新しいノードノードとして追加する

        return true;
      }else
        return false;
    }

    // 仮ノードがマップ内 True
    bool RRT::checkIfInsideBoundary(RRT::rrtNode &tempNode){
      if(tempNode.posX < costmap_->getOriginX() || tempNode.posY < costmap_->getOriginY()  \
          || tempNode.posX > costmap_->getSizeInMetersX() - costmap_->getOriginX() \
          || tempNode.posY > costmap_->getSizeInMetersY() - costmap_->getOriginY() )
        return false;
      else
        return true;
    }

    // 仮ノードが空きor未知 True
    bool RRT::checkIfOutsideObstacles(RRT::rrtNode tempNode){
      unsigned int gridx,gridy;
      unsigned char* grid = costmap_->getCharMap(); //コストマップのコスト取得
      if(costmap_->worldToMap(tempNode.posX, tempNode.posY, gridx, gridy)){ //world座標をマップ座標に変換
        int index = costmap_->getIndex(gridx, gridy); //仮ノードの位置のコストを取得
        if(grid[index]!=FREE_SPACE&&grid[index]!=NO_INFORMATION){ //障害物あり
          return false;
        }else //空きor未知
          return true;
      }else
        return false;
    }

    // 仮ノードが空き:コスト1、未知:コスト2
    int RRT::getCost(RRT::rrtNode tempNode){
      unsigned int gridx,gridy;
      unsigned char* grid = costmap_->getCharMap(); //コストマップのコスト取得
      if(costmap_->worldToMap(tempNode.posX, tempNode.posY, gridx, gridy)){ //world座標をマップ座標に変換
        int index = costmap_->getIndex(gridx, gridy); //仮ノードの位置のコストを取得
        if(grid[index]==FREE_SPACE){
          return 1;
        }
        if(grid[index]==NO_INFORMATION){
          return 2;
        }
        return 0;
      }
      else
        return 0;
    }


    // 新しいノードを描画用に追加
    void RRT::addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode){
      geometry_msgs::Point point;

      //新しいノードの位置を与えて、マーカーに追加
      point.x = tempNode.posX;
      point.y = tempNode.posY;
      point.z = 0;
      rrtTreeMarker.points.push_back(point);

      //新しいノードの親ノードをマーカーに追加
      RRT::rrtNode parentNode = getParent(tempNode.nodeID); //親ノードのIDを取得
      point.x = parentNode.posX;
      point.y = parentNode.posY;
      point.z = 0;
      rrtTreeMarker.points.push_back(point);
    }

    void RRTStarprocess1(visualization_msgs::Marker &rrtTreeMarker1, RRT::rrtNode &q_min,RRT::rrtNode &tempNode){
      geometry_msgs::Point point;

      point.x = tempNode.posX;//tenpNodeは新しく生成されたノード
      point.y = tempNode.posY;
      point.z = 0;
      rrtTreeMarker1.points.push_back(point);

      point.x = q_min.posX;
      point.y = q_min.posY;
      point.z = 0;
      rrtTreeMarker1.points.push_back(point);
    }

    void RRTStarprocess2(visualization_msgs::Marker &rrtTreeMarker2, RRT::rrtNode &q_min1, RRT::rrtNode &tempNode){
      geometry_msgs::Point point;

      point.x=tempNode.posX;
      point.y=tempNode.posY;
      point.z=0;
      rrtTreeMarker2.points.push_back(point);

      point.x=q_min1.posX;
      point.y=q_min1.posY;
      point.z=0;
      rrtTreeMarker2.points.push_back(point);
    }

    //　ノードが指定したゴールと近い、True
    bool RRT::checkNodetoGoal(double X, double Y, RRT::rrtNode &tempNode){
      double distance = sqrt(pow(X-tempNode.posX,2)+pow(Y-tempNode.posY,2)); //ノードとゴールとの距離
      //std::cout<<"終点からの距離： "<<distance<<endl; //
      if(distance < 0.5){ //ゴールとの距離が0.5以内ならTrue
        return true;
      }
      return false;
    }

    void RRT::setFinalPathData(vector< vector<int> > &rrtPaths,  int i, visualization_msgs::Marker &finalpath){
      RRT::rrtNode tempNode;
      geometry_msgs::Point point;
      for(int j=0; j<rrtPaths[i].size();j++){
        tempNode = getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = 0;

        finalpath.points.push_back(point);
      }
    }

    double caldistance(double sourceX, double sourceY, double destinationX, double destinationY){
      return sqrt(pow(destinationX - sourceX,2) + pow(destinationY - sourceY,2));
    }

    //ゴールが送信されると実行する
    bool RRT::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
      ROS_INFO("init3");
      plan.clear();//パスの初期化
      rrtTree.clear();//rrtTreeの初期化

      //描画用のマーカーを定義
      visualization_msgs::Marker sourcePoint;
      visualization_msgs::Marker goalPoint;
      visualization_msgs::Marker randomPoint;
      visualization_msgs::Marker rrtTreeMarker;
      visualization_msgs::Marker rrtTreeMarker1;
      visualization_msgs::Marker rrtTreeMarker2;
      visualization_msgs::Marker finalPath;

      //マーカーの初期化
      initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, rrtTreeMarker1, rrtTreeMarker2, finalPath);

      srand(time(NULL)); //乱数の種を生成、毎回ランダムに行うため

      RRT::rrtNode newNode; //newNodeという構造体を生成
      /*
       int nodeID
       double posX
       double posY
       int parentID
       double cost
       int leaf
       int leaf_cost
       vector<int> children
       */

      initNode(newNode,start);//ノードの初期化、現在位置をはじめのノードとし、rrtTreeに追加

      double rrtStepSize = 0.2;//一回で伸ばすパスの長さ

      vector< vector<int> > rrtPaths; //二次元配列、最終パス
      vector<int> path;
      int rrtPathLimit = 1;

      int shortestPathLength = 9999; //9999
      int shortestPath = -1; //-1
      int count = 0;
      unsigned int seed = 1;

      RRT::rrtNode tempNode; //newNodeと同じような構造体生成
      bool addNodeResult = false, nodeToGoal = false;
      std::cout<<"start: "<<start.pose.position.x<<"  "<<start.pose.position.y<<endl; //現在位置とゴールを出力
      std::cout<<"goal: "<<goal.pose.position.x<<"  "<<goal.pose.position.y<<endl;
      status=running;
      while(ros::ok() && status){ //ループ
        double startX=start.pose.position.x;
        double startY=start.pose.position.y;
        double goalX=goal.pose.position.x;
        double goalY=goal.pose.position.y;

        if(rrtPaths.size() < rrtPathLimit){ //最終パスがないとき
          //ros::Time start_t = ros::Time::now();
          do{ //仮ノードのチェック
            generateTempPoint(tempNode,costmap_,seed,startX,startY); //仮ノードをtempNodeに追加する
            //std::cout<<"tempnode generated"<<endl;
            seed++;
            judgeangle1(tempNode); //仮ノードの角度チェック,180度未満ならTrue
          }while(!judgeangle1(tempNode)); //180度なら仮ノードを求め直す
          addNodeResult = addNewPointtoRRT(tempNode,rrtStepSize); //仮ノードを新しいノードとして追加する

          if(addNodeResult){ //仮ノードを新しいノードとして追加できたら
            //std::cout<<"tempnode accepted"<<endl;
            addBranchtoRRTTree(rrtTreeMarker,tempNode);//追加したノードをマーカー追加
            //std::cout<<"tempnode printed"<<endl;

          //ゴール判定、使用しない
          //nodeToGoal = checkNodetoGoal(goalX, goalY,tempNode);
          //std::cout<<"nodeToGoal： "<<nodeToGoal<<endl;
          if(getTreeSize() == 100){ //ゴール判定Trueなら
            int i,returnID;
            int cost_max = 0;

            for(i=0; i<this->getTreeSize(); i++){ //rrtTreeの大きさを返す、ノード数
              // ノードが葉ならコストを求める
              if(rrtTree[i].leaf == 1){
                vector<int> cost_path; //コストを求めるためのpath
                cost_path.push_back(i);
                int treecost = 0;
                while(rrtTree[cost_path.front()].nodeID != 0){ //はじめのノードまでコストを求める
                  treecost += rrtTree[cost_path.front()].leaf_cost; //葉の木のコストをもとめる
                  cost_path.insert(cost_path.begin(),rrtTree[cost_path.front()].parentID); //親ノードをcost_pathに挿入
                }
                std::cout<<i<<" "<<rrtTree[i].leaf_cost<<" "<<treecost<<endl; //葉のノードID、コスト、木のコスト

                // 木のコストが大きいものをゴールIDにする
                if(treecost  > cost_max){
                  cost_max = treecost;
                  returnID = i;
                }
                cost_path.clear();
              }
            }
            std::cout<<returnID<<" "<<cost_max<<endl;
            path = getRootToEndPath(returnID); //ゴールまでのパスを取得
            rrtPaths.push_back(path); //ゴールまでのpathを格納
            //std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
            int j=0;

            do{ //pathの座標をplanの格納、planはglobal path
              geometry_msgs::PoseStamped pose=start;
              RRT::rrtNode pathNode=getNode(path[j]); //ノードの構造体取得
              pose.pose.position.x=pathNode.posX; //ノードの位置を格納
              pose.pose.position.y=pathNode.posY;
              plan.push_back(pose); //planにノード位置を格納
              j++;
            }while(path[j]!=returnID);
            //plan.push_back(goal); //ゴール座標をpathに格納
            //ros::Duration(10).sleep();
            std::cout<<"got Root Path"<<endl;
          }
        }
      }else{ //if(rrtPaths.size() >= rrtPathLimit)
        status = success;
        std::cout<<"Finding Optimal Path"<<endl;

        for(int i=0; i<rrtPaths.size();i++){
          if(rrtPaths[i].size() < shortestPath){
            shortestPath = i;
            shortestPathLength = rrtPaths[i].size();
          }
        }
        setFinalPathData(rrtPaths, shortestPath, finalPath);//パスの描画
        rrt_publisher.publish(finalPath);
        return  true;
      }
      rrt_publisher.publish(rrtTreeMarker);
      rrt_publisher.publish(rrtTreeMarker1);
      rrt_publisher.publish(rrtTreeMarker2);

      //ros::spinOnce();
      //ros::Duration(0.01).sleep();
    }
  }
}
/*
   int main(int argc, char** argv){
   ros::init(argc, argv, "rrt");


   ros::spinOnce();
   return 0;
   }*/