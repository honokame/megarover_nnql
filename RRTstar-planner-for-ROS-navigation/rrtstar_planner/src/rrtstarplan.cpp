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

      void generateTempPoint(RRT::rrtNode &tempNode,double goalX, double goalY,costmap_2d::Costmap2D* costmap_);
      bool judgeangle1(RRT::rrtNode tempNode);
      bool addNewPointtoRRT(RRT::rrtNode &tempNode, double rrtStepSize);
      bool checkIfInsideBoundary(RRT::rrtNode &tempNode);
      bool checkIfOutsideObstacles(RRT::rrtNode tempNode);
      void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode);
      bool checkNodetoGoal(double X, double Y, RRT::rrtNode &tempNode);
      void setFinalPathData(vector< vector<int> > &rrtPaths,  int i, visualization_msgs::Marker &finalpath, double goalX, double goalY);

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

    RRT::RRT(){ //はじめに実行、initializeを呼び出す
      ROS_INFO("init1");
    }

    RRT::RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros){ //呼び出されない 
      initialize(name, costmap_ros);
      ROS_INFO("init2");
    }

    void RRT::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
      ROS_INFO("init3");
      rrt_publisher = pn.advertise<visualization_msgs::Marker> ("path_planner_rrt",1000);
      if(!initialized_){
        costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
        costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_
        footprint = costmap_ros_->getRobotFootprint();

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

    void RRT::initNode(RRT::rrtNode &newNode, const geometry_msgs::PoseStamped& start){
      ROS_INFO("init4");
      newNode.posX=start.pose.position.x;
      newNode.posY=start.pose.position.y;
      newNode.parentID = 0;
      newNode.nodeID = 0;
      newNode.cost=0;
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
        if ( getEuclideanDistance(getPosX(tempNodeID),getPosY(tempNodeID),getPosX(i),getPosY(i))<=win_r){//新しいノードとすべてのノード
          rrtNeighbor.push_back(getNode(i)); //隣接ノードを格納するための配列
        }   
      }
      return rrtNeighbor;
    }

    // For setting the rrtTree to the inputTree @param rrtTree
    void RRT::setTree(vector<RRT::rrtNode> input_rrtTree){
      rrtTree = input_rrtTree;
    }

    // to get the number of nodes in the rrt Tree @return tree size
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

    // getting a specific node @param node id for the required node @return node in the rrtNode structure
    RRT::rrtNode RRT::getNode(int id){
      return rrtTree[id];
    }

    // return a node from the rrt tree nearest to the given point @param X position in X cordinate @param Y position in Y cordinate @return nodeID of the nearest Node
    int RRT::getNearestNodeID(double X, double Y){
      int i, returnID;
      double distance = 9999, tempDistance;
      for(i=0; i<this->getTreeSize(); i++){
        tempDistance = getEuclideanDistance(X,Y, getPosX(i),getPosY(i));
        if (tempDistance < distance){
          distance = tempDistance;
          returnID = i;
        }
      }
      return returnID;
    }

    // returns X coordinate of the given node
    double RRT::getPosX(int nodeID){
      return rrtTree[nodeID].posX;
    }

    // returns Y coordinate of the given node
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

    // returns parentID of the given node
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

    // returns euclidean distance between two set of X,Y coordinates
    double RRT::getEuclideanDistance(double sourceX, double sourceY, double destinationX, double destinationY){
      return sqrt(pow(destinationX - sourceX,2) + pow(destinationY - sourceY,2));
    }

    // returns path from root to end node @param endNodeID of the end node @return path containing ID of member nodes in the vector form
    vector<int> RRT::getRootToEndPath(int endNodeID){
      vector<int> path;
      path.push_back(endNodeID);
      while(rrtTree[path.front()].nodeID != 0){//path.front()はIDを返す
        //std::cout<<rrtTree[path.front()].nodeID<<endl;
        path.insert(path.begin(),rrtTree[path.front()].parentID);//ここのIDに問題があり、ループが飛び出している?
        //path.begin()は最後の新しいノードのID、その後に挿入されるノードの親ノード
      }
      return path;
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

    void RRT::generateTempPoint(RRT::rrtNode &tempNode,double goalX, double goalY,costmap_2d::Costmap2D* costmap_){
      float probability=0.2;
      double maxx = costmap_->getSizeInMetersX() - costmap_->getOriginX();
      double minx = costmap_->getOriginX();
      double maxy = costmap_->getSizeInMetersY() - costmap_->getOriginY();
      double miny = costmap_->getOriginY();

      if (rand()/double(RAND_MAX)<probability){
        tempNode.posX=goalX;
        tempNode.posY=goalY;
        //std::cout<<"rand: "<<tempNode.posX<<" "<<tempNode.posY<<endl;
      }else{
        double x = double(rand())/double(RAND_MAX)*(maxx - minx) + minx;
        double y = double(rand())/double(RAND_MAX)*(maxy - miny) + miny;
        //std::cout<<"Random X: "<<x <<endl<<"Random Y: "<<y<<endl;
        tempNode.posX = x;
        tempNode.posY = y;
        //std::cout<<"rand: "<<tempNode.posX<<" "<<tempNode.posY<<endl;
      }
    }

    bool RRT::judgeangle1(RRT::rrtNode tempNode){
      int nearestNodeID = getNearestNodeID(tempNode.posX,tempNode.posY);
      RRT::rrtNode nearestNode = getNode(nearestNodeID);

      double theta = atan2(tempNode.posY - nearestNode.posY,tempNode.posX - nearestNode.posX);//この角度意味不明?、使ってないかも
      vector<double> n1,n2;
      if(nearestNode.parentID==0){
        n1.push_back(tempNode.posX - nearestNode.posX);
        n1.push_back(tempNode.posY - nearestNode.posY);
        n2.push_back(0.0001);
        n2.push_back(0.0001);
      }else{
        n1.push_back(tempNode.posX - nearestNode.posX);
        n1.push_back(tempNode.posY - nearestNode.posY);
        n2.push_back(nearestNode.posX-getPosX(nearestNode.parentID));
        n2.push_back(nearestNode.posY-getPosY(nearestNode.parentID));
      }

      double phy = acos((n1[0]*n2[0]+n1[1]*n2[1])/(sqrt(n1[0]*n1[0]+n1[1]*n1[1])*sqrt(n2[0]*n2[0]+n2[1]*n2[1])));
      return (abs(phy)<=PI) ? true : false;
    }

    bool RRT::addNewPointtoRRT(RRT::rrtNode &tempNode, double rrtStepSize){
      int nearestNodeID = getNearestNodeID(tempNode.posX,tempNode.posY);//触発された、myrrt.で書くことは可能か?、特定の関数はrrt.cppに書かれてる

      RRT::rrtNode nearestNode = getNode(nearestNodeID);//getNodeで多くの情報を含む構造体を取得

      double theta = atan2(tempNode.posY - nearestNode.posY,tempNode.posX - nearestNode.posX);

      //if(theta<=PI/4)条件が満たされなかった場合、マーカーの値をリンクするアクションがないため問題が発生する
      tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));//tempNodeが新しいノードになる
      tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));

      if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(tempNode)){//checkIfOutsideObstacles(obstArray,tempNode))
        tempNode.parentID = nearestNodeID;
        tempNode.nodeID = rrtTree.size();
        //myRRT.addNewNode(tenpNode);//tempNodeは新しいノード、RRTはこのノードで行われる
        tempNode.cost=sqrt(pow(nearestNode.posX - tempNode.posX,2) + pow(nearestNode.posY - tempNode.posY,2))+\
                      nearestNode.cost;//新しいノードごとにコストを計算、プライベートメンバーは自由に呼び出せない
        //std::cout<<"tempNode.cost= "<<tempNode.cost<<endl;
        //std::cout<<"tempNode.posX: "<<tempNode.posX<<"  tempNode.posY"<<tempNode.posY<<endl;
        rrtTree.push_back(tempNode);

        return true;
      }else
        return false;
    }

    bool RRT::checkIfInsideBoundary(RRT::rrtNode &tempNode){
      if(tempNode.posX < costmap_->getOriginX() || tempNode.posY < costmap_->getOriginY()  \
          || tempNode.posX > costmap_->getSizeInMetersX() - costmap_->getOriginX() \
          || tempNode.posY > costmap_->getSizeInMetersY() - costmap_->getOriginY() ) 
        return false;
      else 
        return true;
    }

    bool RRT::checkIfOutsideObstacles(RRT::rrtNode tempNode){
      unsigned int gridx,gridy;
      unsigned char* grid = costmap_->getCharMap();
      if(costmap_->worldToMap(tempNode.posX, tempNode.posY, gridx, gridy)){     
        int index = costmap_->getIndex(gridx, gridy);
        if(grid[index]!=FREE_SPACE&&grid[index]!=NO_INFORMATION){
          return false;
        }else
          return true;
      }else
        return false;
    }

    void RRT::addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode){//RRT用の別の関数を作成、描画に使用
      geometry_msgs::Point point;

      point.x = tempNode.posX;//tempNodeは新しく生成されたノード
      point.y = tempNode.posY;
      point.z = 0;
      rrtTreeMarker.points.push_back(point);

      RRT::rrtNode parentNode = getParent(tempNode.nodeID);

      point.x = parentNode.posX;
      point.y = parentNode.posY;
      point.z = 0;

      rrtTreeMarker.points.push_back(point);//新しく生成されたノードと親ノードを追加、新しいノードをリンクする
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

    bool RRT::checkNodetoGoal(double X, double Y, RRT::rrtNode &tempNode){
      double distance = sqrt(pow(X-tempNode.posX,2)+pow(Y-tempNode.posY,2));
      //std::cout<<"終点からの距離： "<<distance<<endl; //
      if(distance < 0.05){
        return true;
      }
      return false;
    }

    void RRT::setFinalPathData(vector< vector<int> > &rrtPaths,  int i, visualization_msgs::Marker &finalpath, double goalX, double goalY){
      RRT::rrtNode tempNode;
      geometry_msgs::Point point;
      for(int j=0; j<rrtPaths[i].size();j++){
        tempNode = getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = 0;

        finalpath.points.push_back(point);
      }

      point.x = goalX;
      point.y = goalY;
      finalpath.points.push_back(point);
    }

    double caldistance(double sourceX, double sourceY, double destinationX, double destinationY){
      return sqrt(pow(destinationX - sourceX,2) + pow(destinationY - sourceY,2));
    }
    
    bool RRT::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
      ROS_INFO("init5");
      plan.clear();
      rrtTree.clear();

      //defining markers
      visualization_msgs::Marker sourcePoint;
      visualization_msgs::Marker goalPoint;
      visualization_msgs::Marker randomPoint;
      visualization_msgs::Marker rrtTreeMarker;
      visualization_msgs::Marker rrtTreeMarker1;
      visualization_msgs::Marker rrtTreeMarker2;
      visualization_msgs::Marker finalPath;

      initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, rrtTreeMarker1, rrtTreeMarker2, finalPath);

      srand(time(NULL));

      RRT::rrtNode newNode;

      initNode(newNode,start);//ノードの初期化

      double rrtStepSize = 0.05;//

      vector< vector<int> > rrtPaths;
      vector<int> path;
      int rrtPathLimit = 1;

      int shortestPathLength = 9999;
      int shortestPath = -1;

      RRT::rrtNode tempNode;

      bool addNodeResult = false, nodeToGoal = false;
      std::cout<<"start: "<<start.pose.position.x<<"  "<<start.pose.position.y<<endl;
      std::cout<<"goal: "<<goal.pose.position.x<<"  "<<goal.pose.position.y<<endl;
      status=running;
      while(ros::ok() && status){
        double goalX=goal.pose.position.x;
        double goalY=goal.pose.position.y;
        if(rrtPaths.size() < rrtPathLimit){
          do{
            generateTempPoint(tempNode,goalX,goalY,costmap_);
            //std::cout<<"tempnode generated"<<endl;
            judgeangle1(tempNode);
          }
          while(!judgeangle1(tempNode));

          addNodeResult = addNewPointtoRRT(tempNode,rrtStepSize);//addNewPointtoRRT(myRRT,tempNode,rrtStepSize,obstacleList);

          if(addNodeResult){
            //std::cout<<"tempnode accepted"<<endl;
            addBranchtoRRTTree(rrtTreeMarker,tempNode);//エラーが発生した場合は、tenpNodeが新しいノードか確認する
            //std::cout<<"tempnode printed"<<endl;

            //RRT*プログラム
            //触発された、myrrt.で書くことは可能か?、特定の関数はrrt.cppに書かれてる
            //親ノードのプロセスを再選択
            vector<RRT::rrtNode> rrtNeighbor=getNearestNeighbor(tempNode.nodeID);//エラーが発生した場合は、tenpNodeが新しいノードか確認する
            //std::cout<<"rrtNeighbor.size= "<<rrtNeighbor.size()<<endl;///////////////////////////////////////////////
            int nearestNodeID = getNearestNodeID(tempNode.posX,tempNode.posY);
            RRT::rrtNode nearestNode = getNode(nearestNodeID);
            RRT::rrtNode q_min=nearestNode;
            double C_min=tempNode.cost;//tempNode.costの前にコストに対する操作がないことに注意
            for(int k=0;k<rrtNeighbor.size();k++){
              if(checkIfInsideBoundary(rrtNeighbor[k]) && checkIfOutsideObstacles(rrtNeighbor[k])\
                  &&rrtNeighbor[k].cost+\
                  caldistance(tempNode.posX,tempNode.posY,rrtNeighbor[k].posX,rrtNeighbor[k].posY)<C_min){
                q_min = rrtNeighbor[k];
                C_min = rrtNeighbor[k].cost+\
                        caldistance(tempNode.posX,tempNode.posY,rrtNeighbor[k].posX,rrtNeighbor[k].posY);
              }
            }
            RRTStarprocess1(rrtTreeMarker1,q_min,tempNode);//addBranchtoRRTTreeのような新しい関数を追加、rrtTreeMarker1は新しいマーカー
            tempNode.cost=C_min;

            //問題の原因：parentIDとnodeIDが等しい場合がある
            if(tempNode.nodeID!=q_min.nodeID)
              tempNode.parentID=q_min.nodeID;//RRT*の最初のプロセス、親ノードを再選択

            rrtTree.pop_back();
            rrtTree.push_back(tempNode);//tempNodeは新しいノード、RRTはこのノードで行われる
            //std::cout<<"tempNode.nodeID: "<<tempNode.nodeID<<"    tempNode.parentID: "<<tempNode.parentID<<endl;

            //再敗戦処理
            //新しいノードごとに隣接ノードは前のプロセスで検出されているため、隣接ノードを再度検出する必要はない
            RRT::rrtNode q_min1=nearestNode;
            double C_min1=tempNode.cost+caldistance(q_min.posX,q_min.posY,tempNode.posX,tempNode.posY);

            for(int k=0;k<rrtNeighbor.size();k++){
              if(checkIfInsideBoundary(rrtNeighbor[k]) && checkIfOutsideObstacles(rrtNeighbor[k])\
                  &&(tempNode.cost+caldistance(tempNode.posX,tempNode.posY,rrtNeighbor[k].posX,rrtNeighbor[k].posY))\
                  < rrtNeighbor[k].cost){//rrtNeighbor==tempNodeのときのみ、q_min1==tempNodeが発生
                q_min1=rrtNeighbor[k];
                C_min1=tempNode.cost+\
                       caldistance(tempNode.posX,tempNode.posY,rrtNeighbor[k].posX,rrtNeighbor[k].posY);
                if(q_min1.nodeID!=tempNode.nodeID)
                  setParentID(q_min1.nodeID, tempNode.nodeID);
              }
            }
            RRTStarprocess2(rrtTreeMarker2,q_min1,tempNode);
            //判定終了
            nodeToGoal = checkNodetoGoal(goalX, goalY,tempNode);
            //std::cout<<"nodeToGoal： "<<nodeToGoal<<endl;
            if(nodeToGoal){
              //std::cout<<"最後のポイントのID： "<<tempNode.nodeID<<endl;
              path = getRootToEndPath(tempNode.nodeID);//pathは最終的なパスを構成するノードIDをすべて含む配列、ゴールIDはない
              rrtPaths.push_back(path);
              //std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
              int i=0;
              do{
                geometry_msgs::PoseStamped pose=start;
                RRT::rrtNode pathNode=getNode(path[i]);
                pose.pose.position.x=pathNode.posX;
                pose.pose.position.y=pathNode.posY;
                plan.push_back(pose);
                i++;
              }while(path[i]!=tempNode.nodeID);
              plan.push_back(goal);
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
          setFinalPathData(rrtPaths, shortestPath, finalPath, goalX, goalY);//パスの描画
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
