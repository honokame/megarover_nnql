// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

/** \file example_node.cpp
 *  \brief Example of the package utilisation.
 */ 

#include<andres/graph/graph.hxx>
#include"andres/graph/multicut/greedy-additive.hxx"
#include"andres/graph/multicut/kernighan-lin.hxx"
#include "graph_utils/graph_utils_functions.h"
#include "pairwise_consistency/pairwise_consistency.h"
#include "robot_local_map/robot_local_map.h"
#include "global_map_solver/global_map_solver.h"
#include "findClique.h"
#include <string>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include<fstream>
#include<vector>

using namespace std;

vector<string> split(string& input, char delimiter){
  istringstream stream(input);
  string field;
  vector<string> result;
  while (getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

/** \brief Main function of an example program using this package.
 * 
 * In this example, we use 3 input files  <trajectory robot1 .g2o file> <trajectory robot2 .g2o file> <inter robot loop closures .g2o file>
 * to produce a resulting global pose graph.
 */ 
int main(int argc, char* argv[])
{
  std::cout << "---------------------------------------------------------" << std::endl;
  // Parse arguments
  std::string robot1_file_name, robot2_file_name, interrobot_file_name;
  if (argc < 4) {
    std::cout << "Not enough arguments, please specify at least 3 input files. (format supported : .g2o)" << std::endl;
    return -1;
  } else {
    robot1_file_name = "/home/chinourobot/pose_graph_datasets/"+string(argv[1]);
    robot2_file_name = "/home/chinourobot/pose_graph_datasets/"+string(argv[2]);
    interrobot_file_name = "/home/chinourobot/pose_graph_datasets/"+string(argv[3]);
  }

  std::cout << "Construction of local maps from the following files : " << robot1_file_name << ", " << std::endl << robot2_file_name << ", " << std::endl << interrobot_file_name<<endl;
  auto start = std::chrono::high_resolution_clock::now(); //現在日時取得

  //--- Map construction
  auto robot1_local_map = robot_local_map::RobotLocalMap(robot1_file_name);
  auto robot2_local_map = robot_local_map::RobotLocalMap(robot2_file_name);
  auto interrobot_measurements = robot_local_map::RobotMeasurements(interrobot_file_name, true);
  //---
  
  auto finish = std::chrono::high_resolution_clock::now();
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;
  
  std::cout << "Solving global map." ;
  start = std::chrono::high_resolution_clock::now();

  //--- Solve global map
  auto solver = global_map_solver::GlobalMapSolver(robot1_local_map, robot2_local_map, interrobot_measurements);
  int max_clique_size = solver.solveGlobalMap();
  //---

  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;
  std::cout << "Maximum clique size = " << max_clique_size << std::endl;

  ifstream ifs1(robot1_file_name);
  ifstream ifs2(robot2_file_name);
  ifstream ifs3(interrobot_file_name);
  ifstream ifs4("/home/chinourobot/results/consistent_loop_closures.txt");
  vector<int>i,j,PCM_i,PCM_j;
  vector<double>x,y,t;
  string line;
  while(getline(ifs1,line)){
    vector<string>strvec=split(line,' ');
    i.push_back(stoi(strvec[1]));
    j.push_back(stoi(strvec[2]));
    x.push_back(stod(strvec[3]));
    y.push_back(stod(strvec[4]));
    t.push_back(stod(strvec[5]));
  }
  while(getline(ifs2,line)){
    vector<string>strvec=split(line,' ');
    i.push_back(stoi(strvec[1]));
    j.push_back(stoi(strvec[2]));
    x.push_back(stod(strvec[3]));
    y.push_back(stod(strvec[4]));
    t.push_back(stod(strvec[5]));
  }
  while(getline(ifs3,line)){
    vector<string>strvec=split(line,' ');
    i.push_back(stoi(strvec[1]));
    j.push_back(stoi(strvec[2]));
    x.push_back(stod(strvec[3]));
    y.push_back(stod(strvec[4]));
    t.push_back(stod(strvec[5]));
  }
  while(getline(ifs4,line)){
    vector<string>strvec=split(line,' ');
    PCM_i.push_back(stoi(strvec[0]));
    PCM_j.push_back(stoi(strvec[1]));
  }
  vector<double> weights;
  andres::graph::Graph<>graph;
  graph.insertVertices(i.size());

  for(int k=0;k<i.size()-1;k++){
    if(i[k]+1==j[k]){
      graph.insertEdge(i[k],j[k]);
      weights.push_back(-1);
    }
  }
  vector<pair<int,int>>PCM,ODO;
  for(int k=0;k<int(PCM_i.size());k++){
    PCM.push_back(make_pair(PCM_i[k],PCM_j[k]));
  }
  for(int k=0;k<int(i.size());k++){
    ODO.push_back(make_pair(i[k],j[k]));
  }
  for(int k=0;k<i.size();k++){
    int W=1;
    if(find(PCM.begin(),PCM.end(),ODO[k])!=PCM.end()){
      W=W-2;
    }
    if(ODO[k].first+1!=ODO[k].second){
      if(abs(ODO[k+1].first-ODO[k].first)<2 && abs(ODO[k+1].second-ODO[k].second)<2 || abs(ODO[k].first-ODO[k-1].first)<2 && abs(ODO[k].second-ODO[k-1].second)<2){
        W=W-2;
      }
      else{
	W=W+2;
      }
      graph.insertEdge(ODO[k].first,ODO[k].second);
      weights.push_back(W);
    }
  }
  vector<double>edge_labels(graph.numberOfEdges());
  andres::graph::multicut::greedyAdditiveEdgeContraction(graph, weights, edge_labels);

  ofstream ofs("/home/chinourobot/results/graphcut.txt");
  for(int k=0;k<int(i.size());k++){
    if(edge_labels[k]==1){
      ofs<<"EDGE2 "<<i[k]<<" "<<j[k]<<" "<<x[k]<<" "<<y[k]<<" "<<t[k]<<" "<<"10 0 0 10 0 10"<<endl;
    }
  }
  
  return 0;
}
