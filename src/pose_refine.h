#ifndef CXXLINEMOD_H
#define CXXLINEMOD_H
#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class poseRefine{
public:
    poseRefine(): fitness(-1.0),inlier_rmse(-1.0){}
    void process(cv::Mat& sceneDepth, cv::Mat& modelDepth, cv::Mat& sceneK, cv::Mat& modelK,
                  cv::Mat& modelRT, int detectX, int detectY);
    cv::Mat result_refined;
    double fitness, inlier_rmse;
};

#endif
