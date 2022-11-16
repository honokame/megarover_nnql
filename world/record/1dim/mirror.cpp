#include<vector>
#include<sstream>
#include<iostream>
#include<string>
#include<fstream>
#include<numeric> //accumulate
#include<math.h>
using namespace std;

vector<string> split(string& input, char delimiter){
  istringstream stream(input);
  string field;
  vector<string> result;
  while(getline(stream, field, delimiter)){
    result.push_back(field);
  }
  return result;
}

int main(int argc, char* argv[]){
  string line;
  vector<long int> n1,n2,n3,n4,n5,n6,n7,n8,n9,label; //ノード値
  
  //odoデータの処理
  ifstream mirror("mirror.txt");//元データ
  ofstream ofs("mirror2.txt");
  while(getline(mirror,line)){
      vector<string>vec=split(line,' ');
      n1.push_back(stol(vec[0]));
      n2.push_back(stol(vec[1]));
      n3.push_back(stol(vec[2]));
      n4.push_back(stol(vec[3]));
      n5.push_back(stol(vec[4]));
      n6.push_back(stol(vec[5]));
      n7.push_back(stol(vec[6]));
      n8.push_back(stol(vec[7]));
      n9.push_back(stol(vec[8]));
      label.push_back(stol(vec[9]));
  }
  for(int k = 0;k < n1.size();k++){
    ofs<<n9[k]<<" "<<n8[k]<<" "<<n7[k]<<" "<<n6[k]<<" "<<n5[k]<<" "<<n4[k]<<" "<<n3[k]<<" "<<n2[k]<<" "<<n1[k]<<" "<<label[k]<<endl;
  }
}

     
