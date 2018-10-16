#include <iostream>
#include <fstream>
#include <cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <png++/png.hpp>
using namespace std;
using namespace cv;

int main()
{

  cv::Mat depth = cv::imread("1_depth.png",IMREAD_ANYDEPTH);
  cv::Mat color = cv::imread("1.jpg");
  cv::cvtColor(color,color,CV_BGR2RGB);
//  cv::imwrite()
  // prepare show depth
  double min, max;
  cv::minMaxIdx(depth,&min,&max);
  cv::Mat show_dp;
  depth.convertTo(show_dp,CV_8U,255/(max-min),-min);
  std::cout<<"rows: "<<show_dp.rows<<" cols: "<<show_dp.cols<<std::endl;
  cout<<"min: "<<min<<endl;
  cout<<"max: "<<max<<endl;
  imshow("depth",show_dp);
  imshow("color",color);
  waitKey(0);
  return 0;
}
