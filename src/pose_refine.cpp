// originally running linemod fails under py,
// this file is designed for running in c++ then binding to py
// however after some effort I find out that depth type matters
// now linemod works fine under py

#include "pose_refine.h"
#include <memory>
#include <iostream>
#include <opencv2/surface_matching.hpp>
#include "linemod_icp.h"
#include <assert.h>
#include <utils.h>

#include "Open3D/Core/Registration/Registration.h"
#include "Open3D/Core/Geometry/Image.h"
#include "Open3D/Core/Camera/PinholeCameraIntrinsic.h"
#include "Open3D/Core/Geometry/PointCloud.h"
#include "Open3D/Visualization/Visualization.h"

using namespace std;
using namespace cv;

// for test
// string type2str(int type) {
//   string r;

//   uchar depth = type & CV_MAT_DEPTH_MASK;
//   uchar chans = 1 + (type >> CV_CN_SHIFT);

//   switch ( depth ) {
//     case CV_8U:  r = "8U"; break;
//     case CV_8S:  r = "8S"; break;
//     case CV_16U: r = "16U"; break;
//     case CV_16S: r = "16S"; break;
//     case CV_32S: r = "32S"; break;
//     case CV_32F: r = "32F"; break;
//     case CV_64F: r = "64F"; break;
//     default:     r = "User"; break;
//   }

//   r += "C";
//   r += (chans+'0');

//   return r;
// }

// static std::string prefix = "/home/meiqua/6DPose/cxxlinemod/test/case1/";
// // use rgb to check ROI
// static Mat rgb = cv::imread(prefix+"rgb.png");
// static Mat rgb_ren = cv::imread(prefix+"rgb_ren.png");
// int main(){
//     // test case1
//     /*
// - cam_R_m2c: [0.99710798, -0.07587780, -0.00426525,
//              -0.06000210, -0.75155902, -0.65693200,
//               0.04664090, 0.65528798, -0.75393802]
//   cam_t_m2c: [100.61034025, 180.32773127, 1024.07664363]
//   obj_bb: [352, 324, 57, 48]
//   */

//     Mat depth = cv::imread(prefix+"0003.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
//     Mat depth_ren = cv::imread(prefix+"depth_ren.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
// //    cout << "depth: " << type2str(depth.type())  << endl;

//     Mat K = (Mat_<float>(3,3) << 572.4114, 0.0, 325.2611, 0.0, 573.57043, 242.04899, 0.0, 0.0, 1.0);
//     Mat R = (Mat_<float>(3,3) << 1.00000000, 0.00000000, 0.00000000,
//              0.00000000, -0.90727223, -0.42054381, 0.00000000, 0.42054381, -0.90727223);
//     Mat t = (Mat_<float>(3,1) << 0.0, 0.0, 1000.0);
//     auto pr = std::make_unique<poseRefine>();
//     pr->process(depth, depth_ren, K, K, R, t, 352, 327);

//     cout << pr->getR() << endl;
//     cout << pr->getT() << endl;
//     cout << pr->getResidual() << endl;

//     cout << "break point line" << endl;
//     return 0;
// }

////adapted from ros ork
//void poseRefine::process(Mat &sceneDepth, Mat &modelDepth, Mat &sceneK, Mat &modelK,
//                        Mat &modelRT, int detectX, int detectY)
//{
////    sceneDepth.convertTo(sceneDepth, CV_16U);
////    modelDepth.convertTo(modelDepth, CV_16U);
//    assert(sceneDepth.type() == CV_16U);
//    assert(sceneK.type() == CV_32F);

//    cv::Mat modelMask = modelDepth > 0;
//    Mat non0p;
//    findNonZero(modelMask,non0p);
//    Rect bbox=boundingRect(non0p);

//    //crop scene
//    int enhancedX = bbox.width/8*0;
//    int enhancedY = bbox.height/8*0; // no padding
//    //boundary check
//    int bboxX1 = detectX - enhancedX;
//    if(bboxX1 < 0) bboxX1 = 0;
//    int bboxX2 = detectX + bbox.width + enhancedX;
//    if(bboxX2 > sceneDepth.cols) bboxX2 = sceneDepth.cols;
//    int bboxY1 = detectY - enhancedY;
//    if(bboxY1 < 0) bboxY1 = 0;
//    int bboxY2 = detectY + bbox.height + enhancedY;
//    if(bboxY2 > sceneDepth.rows) bboxY2 = sceneDepth.rows;
//    cv::Rect ROI_sceneDepth(bboxX1, bboxY1, bboxX2-bboxX1, bboxY2-bboxY1);
//    //get a scene mask
//    cv::Mat sceneMask = cv::Mat::zeros(sceneDepth.size(),CV_8U);
//    sceneMask(ROI_sceneDepth)=255;
//    cv::Mat modelCloud;
//    cv::rgbd::depthTo3d(modelDepth, modelK, modelCloud, modelMask);
//    /*@TODO is there boolean operation in opencv? then use the whole scene, and select latter*/
//    cv::Mat sceneCloud;
//    cv::rgbd::depthTo3d(sceneDepth, sceneK, sceneCloud);

//    //info
////    std::cout<<"rows:"<<sceneCloud.rows<<std::endl;//640
////    std::cout<<"cols:"<<sceneCloud.cols<<std::endl;//480
////    imshow("rgb_ren_cropped", rgb_ren(ROI_modelDepth));
////    imshow("rgb_cropped", rgb(ROI_sceneDepth));
////    waitKey(1000000);
////    imshow("model_mask",modelMask);
////    imshow("scenemask",sceneMask);
////    waitKey(0);
//    // get x,y coordinate of obj in scene
//    int smoothSize = 7;
//    //boundary check
//    int offsetX = ROI_sceneDepth.x + ROI_sceneDepth.width/2;
//    int offsetY = ROI_sceneDepth.y + ROI_sceneDepth.height/2;
////    Vec3f tmp = sceneCloud.at<Vec3f>(offsetY,offsetX);
//    int startoffsetX1 = offsetX - smoothSize/2;
//    if(startoffsetX1 < 0) startoffsetX1 = 0;
//    int startoffsetX2 = offsetX + smoothSize/2;
//    if(startoffsetX2 > sceneCloud.cols) startoffsetX2 = sceneCloud.cols;
//    int startoffsetY1 = offsetY - smoothSize/2;
//    if(startoffsetY1 < 0) startoffsetY1 = 0;
//    int startoffsetY2 = offsetY + smoothSize/2;
//    if(startoffsetY2 > sceneCloud.rows) startoffsetY2 = sceneCloud.rows;

//    cv::Vec3f avePoint; int count=0;
//    for(auto i=startoffsetX1; i<startoffsetX2; i++){
//        for(auto j=startoffsetY1; j<startoffsetY2; j++){
//            auto p = sceneCloud.at<cv::Vec3f>(j, i);
//            if(checkRange(p)){
//                avePoint += p;
//                count++;
//            }
//        }
//    }
//    avePoint /= count;
////    modelRT.at<float>(0, 3) = 0.0; // scene cloud unit is meter
////    modelRT.at<float>(1, 3) = 0.0;
//    modelRT.at<float>(2,3)/=1000.0; //change to meter, for the cloud is all in meter
//    // well, it looks stupid
////    auto R_real_icp = cv::Matx33f(modelRT.at<float>(0, 0), modelRT.at<float>(0, 1), modelRT.at<float>(0, 2),
////                       modelRT.at<float>(1, 0), modelRT.at<float>(1, 1), modelRT.at<float>(1, 2),
////                       modelRT.at<float>(2, 0), modelRT.at<float>(2, 1), modelRT.at<float>(2, 2));
////    auto T_real_icp = cv::Vec3f(modelRT.at<float>(0, 3), modelRT.at<float>(1, 3), modelRT.at<float>(2, 3));

//    auto R_real_icp = cv::Matx33f::eye();
//    auto T_real_icp = cv::Vec3f(avePoint[0],avePoint[1],0.0);

//    std::vector<cv::Vec3f> pts_real_model_temp;
//    std::vector<cv::Vec3f> pts_real_ref_temp;
//    matToVec(modelCloud,pts_real_model_temp,cv::Mat::ones(modelCloud.size(),CV_8U));
//    matToVec(sceneCloud, pts_real_ref_temp, sceneMask);
////    float px_ratio_missing = matToVec(sceneCloud_cropped, modelCloud_cropped, pts_real_ref_temp, pts_real_model_temp);
////    saveCloud(pts_real_model_temp,"pts_real_model_temp.txt",1000.0);
////    saveCloud(pts_real_ref_temp,"pts_real_ref_temp.txt",1000.0);
////    saveMat(modelRT,"modelRT.txt");
//    //@TODO  the below icp result is not correct
//    float px_ratio_match_inliers = 0.0f;
//    float icp_dist = icpCloudToCloud(pts_real_ref_temp, pts_real_model_temp, R_real_icp,
//                                     T_real_icp, px_ratio_match_inliers, 1);

//    icp_dist = icpCloudToCloud(pts_real_ref_temp, pts_real_model_temp, R_real_icp,
//                               T_real_icp, px_ratio_match_inliers, 2);

//    icp_dist = icpCloudToCloud(pts_real_ref_temp, pts_real_model_temp, R_real_icp,
//                               T_real_icp, px_ratio_match_inliers, 0);

//    R_refined = Mat(R_real_icp)*modelRT(Range(0,3),Range(0,3));
//    t_refiend = Mat(R_real_icp)*modelRT(Range(0,3),Range(3,4))+Mat(T_real_icp); // if the last Mat() is eliminated, the answer will be wrong
////    cv::Mat final_rt = getRT();
////    R_refined=Mat(R_real_icp);
////    t_refiend=Mat(T_real_icp);
////    cv::Mat final_rt = getRT()*modelRT;
////    cout<<"check"<<final_rt<<endl;
////    R_refined = final_rt(Range(0,3),Range(0,3)).clone();
////    t_refiend = final_rt(Range(0,3),Range(3,4)).clone();
//    residual = icp_dist;
//}

template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst)
{
    if (!(src.Flags & Eigen::RowMajorBit))
    {
        cv::Mat _src(src.cols(), src.rows(), cv::DataType<_Tp>::type,
                     (void*)src.data(), src.stride() * sizeof(_Tp));
        cv::transpose(_src, dst);
    }
    else
    {
        cv::Mat _src(src.rows(), src.cols(), cv::DataType<_Tp>::type,
                     (void*)src.data(), src.stride() * sizeof(_Tp));
        _src.copyTo(dst);
    }
}

void poseRefine::process(Mat &sceneDepth, Mat &modelDepth, Mat &sceneK, Mat &modelK, Mat &modelRT, int detectX, int detectY)
{
  assert(sceneDepth.type() == CV_16U);
  assert(sceneK.type() == CV_32F);
  fitness = -1;
  inlier_rmse = -1;

  cv::Mat init_base_cv = modelRT.t();// row to col major
  Eigen::Matrix4f init_base = Eigen::Map<Eigen::Matrix4f>(reinterpret_cast<float*>(init_base_cv.data));

  cv::Mat modelMask = modelDepth > 0;
  cv::Mat non0p;
  cv::findNonZero(modelMask, non0p);
  cv::Rect bbox = boundingRect(non0p);

  cv::Rect roi = cv::Rect(detectX, detectY, bbox.width, bbox.height);
  //@todo is the following necessary?
//  if((detectX + bbox.width >= sceneDepth.cols) || (detectY + bbox.height >= sceneDepth.rows)) return;
  if(detectX+roi.width>=sceneDepth.cols) roi.width = sceneDepth.cols-detectX;
  if(detectY+roi.height>=sceneDepth.rows) roi.height = sceneDepth.rows-detectY;
  open3d::Image scene_depth_open3d, model_depth_open3d;
  model_depth_open3d.PrepareImage(modelDepth.cols, modelDepth.rows, 1, 2);

  std::copy_n(modelDepth.data, model_depth_open3d.data_.size(),
              model_depth_open3d.data_.begin());

  open3d::PinholeCameraIntrinsic K_scene_open3d(sceneDepth.cols, sceneDepth.rows,
                                                    double(sceneK.at<float>(0, 0)), double(sceneK.at<float>(1, 1)),
                                                    double(sceneK.at<float>(0, 2)), double(sceneK.at<float>(1, 2)));

  open3d::PinholeCameraIntrinsic K_model_open3d(modelDepth.cols, modelDepth.rows,
                                                    double(modelK.at<float>(0, 0)), double(modelK.at<float>(1, 1)),
                                                    double(modelK.at<float>(0, 2)), double(modelK.at<float>(1, 2)));
  auto model_pcd = open3d::CreatePointCloudFromDepthImage(model_depth_open3d, K_model_open3d);
  Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity(4, 4);
  cv::Mat scene_depth_model_cover = cv::Mat::zeros(sceneDepth.rows, sceneDepth.cols, sceneDepth.type());
  //@todo necessary to add a mask?
  sceneDepth(roi).copyTo(scene_depth_model_cover(roi), modelMask(bbox));
  cv::medianBlur(scene_depth_model_cover, scene_depth_model_cover, 5);

  open3d::Image scene_depth_for_center_estimation;
  scene_depth_for_center_estimation.PrepareImage(scene_depth_model_cover.cols, scene_depth_model_cover.rows, 1, 2);
  std::copy_n(scene_depth_model_cover.data, scene_depth_for_center_estimation.data_.size(),
              scene_depth_for_center_estimation.data_.begin());
  auto scene_pcd_for_center = open3d::CreatePointCloudFromDepthImage(scene_depth_for_center_estimation, K_scene_open3d);

  double voxel_size = 0.0025;
  auto model_pcd_down = open3d::VoxelDownSample(*model_pcd, voxel_size);
  auto scene_pcd_down = open3d::VoxelDownSample(*scene_pcd_for_center, voxel_size);

  Eigen::Vector3d center_scene = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_model = Eigen::Vector3d::Zero();

  for(auto& p: scene_pcd_down->points_){
      center_scene += p;
  }
  center_scene /= scene_pcd_down->points_.size();

  for(auto& p: model_pcd_down->points_){
      center_model += p;
  }
  center_model /= model_pcd_down->points_.size();

  init_guess.block(0, 3, 3, 1) = center_scene - center_model;

  double threshold = 0.005;

  open3d::EstimateNormals(*model_pcd_down);
  open3d::EstimateNormals(*scene_pcd_down);
  auto final_result = open3d::RegistrationICP(*model_pcd_down, *scene_pcd_down, threshold,
                                              init_guess,
                                              open3d::TransformationEstimationPointToPlane());
  Eigen::Matrix4d result = final_result.transformation_*init_base.cast<double>();

  fitness = final_result.fitness_;
  inlier_rmse = final_result.inlier_rmse_;
  eigen2cv(result, result_refined);
}


