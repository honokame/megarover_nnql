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
  string file =  argv[1]; //$B%U%!%$%kHV9f(B
  string out = argv[2];
  int pos = stoi(argv[2]); //$B0LCV%/%i%9(B
  bool top_s = 1,top_o = 1; //$B0l9TL\=hM}(B
  string line_s,line_o; //csv$BFI$_9~$_(B
  vector<string> laser_s; //$B%l!<%6!<%G!<%?$N%3%T!<(B
  vector<long> time_s,time_o; //$B;~4V(B
  long double n,temp = 0.0; //$B%N!<%ICM!"8=:_$N3QEY!J7W;;2aDx!K(B
  vector<long int> n1,n2,n3,n4,n5,n6,n7,n8,n9; //$B%N!<%ICM(B
  vector<long double> laser,ang;
  int a;
  vector<int> place,place_30; //$B>l=j%/%i%9(B

  //odo$B%G!<%?$N=hM}(B
  ifstream odo(file + "_odo.csv");//(argv[2]); //$B85%G!<%?(B
  while(getline(odo,line_o)){
    if(top_o == 1) {
      top_o = 0;
    }
    else{
      vector<string>vec=split(line_o,',');
      time_o.push_back(stol(vec[0]));
      ang.push_back((stold(vec[6]))*180/M_PI); //$B%i%8%"%s$+$iEY$KJQ49(B
    }
  }
  //ofstream ofs_o(file + "_o.csv");//(argv[5]);
  for(int k = 0;k < time_o.size();k++){
    temp += ang[k]; //$B@{2sNL$+$i8=:_$N3QEY$r5a$a$k(B
    
    //$B3QEY$,(B0-360$B$K$J$k$h$&$KD4@0(B
    if(temp > 360){ 
      temp = temp - 360;
    }else if(temp < 0){
      temp = 360 + temp;
    }
    a = round(temp); 

   //$B>l=j%/%i%9(B 60
   if(a == 360 || a <= 45){
     place.push_back(pos*8-7);
   }else if(a <= 90){
     place.push_back(pos*8-6);
   }else if(a <= 135){
     place.push_back(pos*8-5);
   }else if(a <= 180){
     place.push_back(pos*8-4);
   }else if(a <= 225){
     place.push_back(pos*8-3);
   }else if(a <= 270){
     place.push_back(pos*8-2);
   }else if(a <= 315){
     place.push_back(pos*8-1);
   }else{
     place.push_back(pos*8);
   }
   //$B>l=j%/%i%9(B 30
   if(a == 360 || a <= 30){
     place_30.push_back(pos*12-11);
   }else if(a <= 60){
     place_30.push_back(pos*12-10);
   }else if(a <= 90){
     place_30.push_back(pos*12-9);
   }else if(a <= 120){
     place_30.push_back(pos*12-8);
   }else if(a <= 150){
     place_30.push_back(pos*12-7);
   }else if(a <= 180){
     place_30.push_back(pos*12-6);
   }else if(a <= 210){
     place_30.push_back(pos*12-5); 
   }else if(a <= 240){
     place_30.push_back(pos*12-4);
   }else if(a <= 270){
     place_30.push_back(pos*12-3);
   }else if(a <= 300){
     place_30.push_back(pos*12-2);
   }else if(a <= 330){
     place_30.push_back(pos*12-1);
   }else{
     place_30.push_back(pos*12);
   }

   //ofs_o<<time_o[k]<<","<<place[k]<<endl;
  }
     
  //scan$B%G!<%?$N=hM}(B 
  ifstream scan(file + "_scan.csv");//(argv[1]); //$B85%G!<%?(B
  while(getline(scan,line_s)){
    if(top_s == 1) { //1$B9TL\$ONsL>$J$N$GHt$P$9(B
      top_s = 0;
    }
    else{ //$B?tCM%G!<%?(B
      vector<string>strvec=split(line_s,','); //$BJ8;zNs$H$7$F<u$1<h$k(B
      time_s.push_back(stol(strvec[0])); //$B;~4V(B
      laser_s.resize(1010); //core damp$BM=KI(B
      copy(strvec.begin()+14, strvec.begin()+1022, laser_s.begin()); //$B%l!<%6!<%G!<%?$N$_(B
      //cout<<laser_s[1007]<<endl;:
      //$B%N!<%ICM$N7W;;!"(B1008$B8D$N%G!<%?$r(B9$B8D$GI=8=(B
      for(int i = 0;i < 1008;i++){
        laser.push_back(stold(laser_s[i])); //$B%l!<%6!<%G!<%?$rJ8;zNs$+$i>.?t$KJQ49(B
        switch(i){
          case 111:
            n = accumulate(laser.begin(), laser.begin()+112,0.0); //20$BEY$4$H$K%G!<%?$r9g7W$7$FJ?6Q$9$k(B
            n1.push_back(n/112*1000); 
            break;
          case 223:
            n = accumulate(laser.begin()+112, laser.begin()+224,0.0);
            n2.push_back(n/112*1000);
            break;
          case 335:
            n = accumulate(laser.begin()+224, laser.begin()+336,0.0); 
            n3.push_back(n/112*1000);
            break;
          case 447:
            n = accumulate(laser.begin()+336, laser.begin()+448,0.0);
            n4.push_back(n/112*1000);
            break;
          case 559:
            n = accumulate(laser.begin()+448, laser.begin()+560,0.0); 
            n5.push_back(n/112*1000);
            break;
          case 671:
            n = accumulate(laser.begin()+560, laser.begin()+672,0.0);
            n6.push_back(n/112*1000);
            break;
          case 783:
            n = accumulate(laser.begin()+672, laser.begin()+784,0.0); 
            n7.push_back(n/112*1000);
            break;
          case 895:
            n = accumulate(laser.begin()+784, laser.begin()+896,0.0);
            n8.push_back(n/112*1000);
            break;
          case 1007:
            n = accumulate(laser.begin()+896, laser.begin()+1008,0.0);
            n9.push_back(n/112*1000);
            break;
        }
      }
      laser.clear(); 
     //else
   //while
  
  ofstream ofs_s(file + "_s.csv");//(argv[4]);
  for(int j = 0;j < laser_s.size();j++){
    //ofs_s<<time_s[j]<<","<<endl;
    ofs_s<<laser_s[j]<<",";
  }
  }
  }
  ofstream ofs(out + ".csv");//(argv[3]); //3
  for(int i = 0;i < time_s.size();i++){
    for(int j = 0;j < time_o.size();j++){
      if(abs(time_s[i]-time_o[j]) < abs(time_s[i]-time_o[j+1]) && abs(time_s[i]-time_o[j]) < abs(time_s[i]-time_o[j-1])){
        ofs<<n1[i]<<","<<n2[i]<<","<<n3[i]<<","<<n4[i]<<","<<n5[i]<<","<<n6[i]<<","<<n7[i]<<","<<n8[i]<<","<<n9[i]<<","<<pos<<","<<place[j]<<","<<place_30[j]<<endl;
      }
    }
  }
  //return 0;
} //main

