#include<ros/ros.h>
#include<ros/package.h>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>

int main(int argc, char** argv){
  std::string file_dir = ros::package::getPath("map_tools")+"/map/";
  std::string map = file_dir + "map.pgm";
  cv::Mat source_image = cv::imread(map, cv::IMREAD_GRAYSCALE);
  //cv::imshow("image", source_image);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size (3,3));
  cv::Mat dst;
  cv::erode(source_image, dst, kernel);
  cv::imshow("dst", dst);
  cv::waitKey();
  
  std::string output = file_dir + "dst.pgm";
  cv::imwrite(output, dst);
  return 0;
}
