#include <opencv2/aruco.hpp> 
#include <opencv2/highgui.hpp>
#include <iostream>
int main(int argc, char** argv)
{
  // generate marker yeee
  std::cout << "hai";
  cv::Mat markerImage; 
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(10);
  
  cv::aruco::drawMarker(dictionary, 19, 200, markerImage, 1);
  
  // store marker
  std::string out_file_name = "/home/clp/catkin_ws/src/auto_landing/test/marker2.png";
  cv::imwrite(out_file_name, markerImage);
  return 0;
}