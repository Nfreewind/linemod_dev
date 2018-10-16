#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <fstream>
//renderer headers
#include <object_renderer/utils.h>
#include <object_renderer/renderer3d.h>

#include <pose_refine.h>
using namespace std;
// Function prototypes
void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f);

std::vector<CvPoint> maskFromTemplate(const std::vector<cv::linemod::Template>& templates,
                                      int num_modalities, cv::Point offset, cv::Size size,
                                      cv::Mat& mask, cv::Mat& dst);

void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
                        int num_modalities, cv::Point offset, cv::Size size,
                        cv::Mat& dst);

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T);

cv::Mat displayQuantized(const cv::Mat& quantized);

// Copy of cv_mouse from cv_utilities
class Mouse
{
public:
  static void start(const std::string& a_img_name)
  {
      cv::setMouseCallback(a_img_name.c_str(), Mouse::cv_on_mouse, 0);
  }
  static int event(void)
  {
    int l_event = m_event;
    m_event = -1;
    return l_event;
  }
  static int x(void)
  {
    return m_x;
  }
  static int y(void)
  {
    return m_y;
  }

private:
  static void cv_on_mouse(int a_event, int a_x, int a_y, int, void *)
  {
    m_event = a_event;
    m_x = a_x;
    m_y = a_y;
  }

  static int m_event;
  static int m_x;
  static int m_y;
};
int Mouse::m_event;
int Mouse::m_x;
int Mouse::m_y;

//static void help()
//{
//  printf("Usage: openni_demo [templates.yml]\n\n"
//         "Place your object on a planar, featureless surface. With the mouse,\n"
//         "frame it in the 'color' window and right click to learn a first template.\n"
//         "Then press 'l' to enter online learning mode, and move the camera around.\n"
//         "When the match score falls between 90-95%% the demo will add a new template.\n\n"
//         "Keys:\n"
//         "\t h   -- This help page\n"
//         "\t l   -- Toggle online learning\n"
//         "\t m   -- Toggle printing match result\n"
//         "\t t   -- Toggle printing timings\n"
//         "\t w   -- Write learned templates to disk\n"
//         "\t [ ] -- Adjust matching threshold: '[' down,  ']' up\n"
//         "\t q   -- Quit\n\n");
//}

// Adapted from cv_timer in cv_utilities
class Timer
{
public:
  Timer() : start_(0), time_(0) {}

  void start()
  {
    start_ = cv::getTickCount();
  }

  void stop()
  {
    CV_Assert(start_ != 0);
    int64 end = cv::getTickCount();
    time_ += end - start_;
    start_ = 0;
  }

  double time()
  {
    double ret = time_ / cv::getTickFrequency();
    time_ = 0;
    return ret;
  }

private:
  int64 start_, time_;
};

// Functions to store detector and templates in single XML/YAML file
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = cv::makePtr<cv::linemod::Detector>();
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector->write(fs);

  std::vector<cv::String> ids = detector->classIds();
  fs << "classes" << "[";
  for (int i = 0; i < (int)ids.size(); ++i)
  {
    fs << "{";
    detector->writeClass(ids[i], fs);
    fs << "}"; // current class
  }
  fs << "]"; // classes
}

static void readInfo(cv::Mat& K, std::vector<cv::Mat>& rt_, const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  fs["K"]>>K;
  int num=0;
  fs["num"]>>num;
  cv::FileNode fn=fs["rt_"];
  for(int i=0;i<num;++i)
  {
    cv::Mat rt;
    fn[i]>>rt;
    rt_.push_back(rt);
  }
}
static void writeInfo(const cv::Mat& K, std::vector<cv::Mat>& rt_, const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs<<"K"<<K;
  fs<<"num"<<static_cast<int>(rt_.size());
  fs<<"rt_"<<"[";
  for(int i=0;i<rt_.size();++i)
  {
    fs<<rt_[i];
  }
  fs<<"]";
}


std::string replace_string(std::string str, std::string patten_org, std::string patten_new)
{
  int pos = str.find(patten_org);
  if(pos==-1)
    return str;
  return str.replace(pos,patten_org.length(),patten_new);
}
IplImage * loadDepth(std::string a_name)
{
    std::ifstream l_file(a_name.c_str(), std::ofstream::in | std::ofstream::binary);

    if (l_file.fail() == true)
    {
        printf("cv_load_depth: could not open file for writing!\n");
        return NULL;
    }
    int l_row;
    int l_col;

    l_file.read((char*)&l_row, sizeof(l_row));
    l_file.read((char*)&l_col, sizeof(l_col));

    IplImage * lp_image = cvCreateImage(cvSize(l_col, l_row), IPL_DEPTH_16U, 1);

    for (int l_r = 0; l_r < l_row; ++l_r)
    {
        for (int l_c = 0; l_c < l_col; ++l_c)
        {
            l_file.read((char*)&CV_IMAGE_ELEM(lp_image, unsigned short, l_r, l_c), sizeof(unsigned short));
        }
    }
    l_file.close();

    return lp_image;
}

float intersectRect(const cv::Rect& rectA, const cv::Rect& rectB){
  if (rectA.x > rectB.x + rectB.width) { return 0.; }
  if (rectA.y > rectB.y + rectB.height) { return 0.; }
  if ((rectA.x + rectA.width) < rectB.x) { return 0.; }
  if ((rectA.y + rectA.height) < rectB.y) { return 0.; }
  cv::Rect intersectRect;
  float colInt = min(rectA.x + rectA.width, rectB.x + rectB.width) - max(rectA.x, rectB.x);
  float rowInt = min(rectA.y + rectA.height, rectB.y + rectB.height) - max(rectA.y, rectB.y);
  float intersection = colInt * rowInt;
  float areaA = rectA.width * rectA.height;
  float areaB = rectB.width * rectB.height;
  float intersectionPercent = intersection / (areaA + areaB - intersection);

  intersectRect.x = max(rectA.x, rectB.x);
  intersectRect.y = max(rectA.y, rectB.y);
  intersectRect.width = min(rectA.x + rectA.width, rectB.x + rectB.width) - intersectRect.x;
  intersectRect.height = min(rectA.y + rectA.height, rectB.y + rectB.height) - intersectRect.y;
  return intersectionPercent;
}


int main(int argc, char * argv[])
{
  // Various settings and flags
  bool show_match_result = true;
  bool show_timings = true;
  std::string model_filename;
//  bool learn_online = false;
  int num_classes = 0;
  int matching_threshold = 65;
  /// @todo Keys for changing these?
//  cv::Size roi_size(200, 200);
//  int learning_lower_bound = 90;
//  int learning_upper_bound = 95;

  // Timers
  Timer extract_timer;
  Timer match_timer;

  // Initialize HighGUI
  //help();
  cv::namedWindow("color");
  cv::namedWindow("normals");
  Mouse::start("color");

  // Initialize LINEMOD data structures
  cv::Ptr<cv::linemod::Detector> detector;
  cv::Mat K_model;
  std::vector<cv::Mat> rt_;
//  std::string filename;
  if (argc <= 1)
  {
    std::cout<<"error!"<<std::endl;
    exit(-1);
  }
  else
  {
    model_filename = argv[1];
    detector = readLinemod(model_filename+"_templates.yaml");
    std::vector<cv::String> ids = detector->classIds();
    num_classes = detector->numClasses();
    printf("Loaded %s with %d classes and %d templates\n",
           argv[1], num_classes, detector->numTemplates());
    //read info
    std::string info_filename = model_filename+"_info.yaml";
    cout<<"info file name: "<<info_filename<<std::endl;
    readInfo(K_model,rt_,info_filename);
    std::cout<<"K: \n"<<K_model<<std::endl;
    if (!ids.empty())
    {
      printf("Class ids:\n");
      std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    }
  }
  int num_modalities = (int)detector->getModalities().size();

  // load color and depth, depth im milimeter
//  IplImage* lp=loadDepth("depth0.dpt");
//  cv::Mat depth = cv::cvarrToMat(lp);
//  cv::Mat color = cv::imread("color0.jpg");
  cv::Mat depth = cv::imread("1_depth.png",cv::IMREAD_ANYDEPTH);
  depth.convertTo(depth,CV_16UC1);
  cv::Mat color = cv::imread("1.jpg");
  cv::cvtColor(color,color,CV_BGR2RGB);

  //initialize a renderer for test
  Renderer3d renderer = Renderer3d(model_filename);
  renderer.set_parameters(640,480,572.41140,573.57043,325.26110,242.04899);
  cv::Mat K_scene = renderer.getIntrinsic();
  std::cout<<K_scene<<std::endl;
  poseRefine pr;
  // prepare show depth
  double min, max;
  cv::minMaxIdx(depth,&min,&max);
  cv::Mat show_dp;
  depth.convertTo(show_dp,CV_8U,255/(max-min),-min);
//  cv::imshow("tst",color);
//  cv::waitKey(0);
//  cv::imshow("tst",show_dp);
//  cv::waitKey(0);

  std::vector<cv::Mat> sources;
  sources.push_back(color);
  sources.push_back(depth);


  // Perform matching
  std::vector<cv::linemod::Match> matches;
  std::vector<cv::String> class_ids;
  std::vector<cv::Mat> quantized_images;
  match_timer.start();
  detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);
  match_timer.stop();

  if (show_match_result && matches.empty()){
    printf("No matches found...\n");
    return -1;
  }
  printf("found %d matches...\n",static_cast<int>(matches.size()));
  if (show_timings)
  {
    printf("Training: %.2fs\n", extract_timer.time());
    printf("Matching: %.2fs\n", match_timer.time());
  }
  if (show_match_result || show_timings)
    printf("------------------------------------------------------------\n");



  //
  // icp check
  //
  int top_k = 100;
  std::vector<cv::Mat> icp_poses;
  std::vector<float> icp_scores;
  std::vector<cv::linemod::Match> icp_matches;
  for(int i=0; (i < (int)matches.size() && i<top_k);++i)
  {
    cv::linemod::Match m = matches[i];
    cv::Mat depth2;
    cv::Mat tmp_rt = rt_[m.template_id].clone();
    renderer.setModelRt(tmp_rt);
    renderer.renderDepthOnly(depth2);
    tmp_rt(cv::Range(0,3),cv::Range(3,4))/=1000.0;//scale to meter
    pr.process(depth,depth2,K_scene,K_model,tmp_rt,m.x,m.y);
    if(pr.fitness < 0.5 || pr.inlier_rmse>0.01)
      continue;
    icp_scores.push_back(1.0/(pr.inlier_rmse+0.01));
    icp_poses.push_back(pr.result_refined.clone());
    icp_matches.push_back(m);
  }

  //
  // nms
  //

  std::vector<int> sel_index;
  std::vector<cv::Rect> sel_rect;//for_debug
  for(int i=0;i<icp_scores.size();++i)
  {
    cv::linemod::Match& cm = icp_matches[i];
    const std::vector<cv::linemod::Template>& ctemplates = detector->getTemplates(cm.class_id, cm.template_id);
    cv::Rect cr(cm.x,cm.y,ctemplates[0].width*(ctemplates[0].pyramid_level+1),ctemplates[0].height*(ctemplates[0].pyramid_level+1));
    if(sel_index.empty()){
      sel_index.push_back(i);
      sel_rect.push_back(cr);
      continue;
    }
    bool found_replace=false;
    for(int j=0;j<sel_index.size();++j)
    {
      //check replace condition
      cv::linemod::Match& tm = icp_matches[sel_index[j]];
      const std::vector<cv::linemod::Template>& ttemplates = detector->getTemplates(tm.class_id, tm.template_id);
      cv::Rect tr(tm.x,tm.y,ttemplates[0].width*(ttemplates[0].pyramid_level+1),ttemplates[0].height*(ttemplates[0].pyramid_level+1));
      float iou_val = intersectRect(tr,cr);
      if(iou_val>0.5){
        found_replace=true;
        if(icp_scores[i]>icp_scores[j])
        {
          sel_index[j]=i;
          sel_rect[j]=cr;
        }
        break;
      }
    }
    if(!found_replace){
      sel_index.push_back(i);
      sel_rect.push_back(cr);
    }
  }
  std::cout<<"final match select: "<< sel_index.size()<<std::endl;
  std::vector<cv::Mat> final_poses;
  std::vector<float> final_scores;
  std::vector<cv::linemod::Match> final_matches;
  for(int i=0;i<sel_index.size();++i)
  {
    final_poses.push_back(icp_poses[sel_index[i]]);
    final_scores.push_back(icp_scores[sel_index[i]]);
    final_matches.push_back(icp_matches[sel_index[i]]);
  }

  //
  // draw rect
  //
  std::cout<<"iou test: "<<intersectRect(sel_rect[0],sel_rect[3])<<std::endl;
  cv::Mat display0=color.clone();
  for(int i=0;i<sel_rect.size();++i)
  {
    cv::rectangle(display0,sel_rect[i], cvScalar(0, 0, 255),1);
  }
  cv::imshow("rect: ", display0);
  cv::waitKey(0);

  //
  // view final result
  //
  for(int i=0;i<final_poses.size();++i)
  {
    printf("ICP Score: %5.1f\n", final_scores[i]);
    cv::Mat tmp_rt=final_poses[i].clone();
    tmp_rt(cv::Range(0,3),cv::Range(3,4))*=1000.0;
    renderer.setModelRt(tmp_rt);
    cv::Mat reproject;
    renderer.renderImageOnly(reproject);
    cv::linemod::Match& cm = final_matches[i];
    const std::vector<cv::linemod::Template>& templates = detector->getTemplates(cm.class_id, cm.template_id);
    cv::Mat display = color.clone();
    drawResponse(templates, num_modalities, display, cv::Point(cm.x, cm.y), detector->getT(0));
    cv::imshow("original color",display);
    cv::imshow("reproject", reproject);
    cv::waitKey(0);
  }

// //
// // test
// //
//    int classes_visited = 0;
//    std::set<std::string> visited;
//  for (int i = 0; (i < (int)matches.size()) /*&& (classes_visited < num_classes)*/; ++i)
//  {
//    cv::linemod::Match m = matches[i];

//    if (visited.insert(m.class_id).second)
//    {
//      ++classes_visited;
//    }
//    if (show_match_result)
//    {
//      printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
//             m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
//    }

//    // Draw matching template
//    const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
//    cv::Mat display = color.clone();
//    drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y), detector->getT(0));
//    // test read_info, reproject the rt to image
//    renderer.setModelRt(rt_[m.template_id]);
//    cv::Mat depth2;
//    //@TODO check depth2 and depth scale
//    renderer.renderDepthOnly(depth2);
//    cv::Mat tmp_rt = rt_[m.template_id].clone();
//    tmp_rt(cv::Range(0,3),cv::Range(3,4))/=1000.0;//scale to meter
//    pr.process(depth,depth2,K_scene,K_model,tmp_rt,m.x,m.y);
//    std::cout<<"fitness:" << pr.fitness<<std::endl;
//    std::cout<<"inlier_rmse"<<pr.inlier_rmse<<std::endl;
//    std::cout<<"org: "<<rt_[m.template_id]<<std::endl;
//    std::cout<<"tmpRT: "<<tmp_rt<<std::endl;
//    std::cout<<"nowRT: "<< pr.result_refined<<std::endl;
//    // reproject use tmp_rt(which add an approximate translation
//    cv::Mat refined_result_mili = pr.result_refined.clone();
//    refined_result_mili(cv::Range(0,3),cv::Range(3,4))*=1000.0;
//    renderer.setModelRt(refined_result_mili);
////    std::cout<<"tmpRT: "<<tmp_rt<<std::endl;
//    cv::Mat disp3;
//    renderer.renderImageOnly(disp3);
////    cv::flip(disp3,disp3,0);
//    cv::imshow("color", display);
//    cv::imshow("normals", quantized_images[1]);
//    cv::imshow("reproject",disp3);
//    cv::waitKey(0);
//  }



//  cv::FileStorage fs;
//  char key = (char)cv::waitKey(10);
//  if( key == 'q' )
//      break;

//  switch (key)
//  {
//    case 'h':
//      //help();
//      break;
//    case 'm':
//      // toggle printing match result
//      show_match_result = !show_match_result;
//      printf("Show match result %s\n", show_match_result ? "ON" : "OFF");
//      break;
//    case 't':
//      // toggle printing timings
//      show_timings = !show_timings;
//      printf("Show timings %s\n", show_timings ? "ON" : "OFF");
//      break;
//    case 'l':
//      // toggle online learning
//      learn_online = !learn_online;
//      printf("Online learning %s\n", learn_online ? "ON" : "OFF");
//      break;
//    case '[':
//      // decrement threshold
//      matching_threshold = std::max(matching_threshold - 1, -100);
//      printf("New threshold: %d\n", matching_threshold);
//      break;
//    case ']':
//      // increment threshold
//      matching_threshold = std::min(matching_threshold + 1, +100);
//      printf("New threshold: %d\n", matching_threshold);
//      break;
//    case 'w':
//      // write model to disk
//      writeLinemod(detector, filename);
//      printf("Wrote detector and templates to %s\n", filename.c_str());
//      break;
//    default:
//      ;
//  }
  return 0;
}

static void reprojectPoints(const std::vector<cv::Point3d>& proj, std::vector<cv::Point3d>& real, double f)
{
  real.resize(proj.size());
  double f_inv = 1.0 / f;

  for (int i = 0; i < (int)proj.size(); ++i)
  {
    double Z = proj[i].z;
    real[i].x = (proj[i].x - 320.) * (f_inv * Z);
    real[i].y = (proj[i].y - 240.) * (f_inv * Z);
    real[i].z = Z;
  }
}

static void filterPlane(IplImage * ap_depth, std::vector<IplImage *> & a_masks, std::vector<CvPoint> & a_chain, double f)
{
  const int l_num_cost_pts = 200;

  float l_thres = 4;

  IplImage * lp_mask = cvCreateImage(cvGetSize(ap_depth), IPL_DEPTH_8U, 1);
  cvSet(lp_mask, cvRealScalar(0));

  std::vector<CvPoint> l_chain_vector;

  float l_chain_length = 0;
  float * lp_seg_length = new float[a_chain.size()];

  for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
  {
    float x_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x);
    float y_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y);
    lp_seg_length[l_i] = sqrt(x_diff*x_diff + y_diff*y_diff);
    l_chain_length += lp_seg_length[l_i];
  }
  for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
  {
    if (lp_seg_length[l_i] > 0)
    {
      int l_cur_num = cvRound(l_num_cost_pts * lp_seg_length[l_i] / l_chain_length);
      float l_cur_len = lp_seg_length[l_i] / l_cur_num;

      for (int l_j = 0; l_j < l_cur_num; ++l_j)
      {
        float l_ratio = (l_cur_len * l_j / lp_seg_length[l_i]);

        CvPoint l_pts;

        l_pts.x = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x) + a_chain[l_i].x);
        l_pts.y = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y) + a_chain[l_i].y);

        l_chain_vector.push_back(l_pts);
      }
    }
  }
  std::vector<cv::Point3d> lp_src_3Dpts(l_chain_vector.size());

  for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
  {
    lp_src_3Dpts[l_i].x = l_chain_vector[l_i].x;
    lp_src_3Dpts[l_i].y = l_chain_vector[l_i].y;
    lp_src_3Dpts[l_i].z = CV_IMAGE_ELEM(ap_depth, unsigned short, cvRound(lp_src_3Dpts[l_i].y), cvRound(lp_src_3Dpts[l_i].x));
    //CV_IMAGE_ELEM(lp_mask,unsigned char,(int)lp_src_3Dpts[l_i].Y,(int)lp_src_3Dpts[l_i].X)=255;
  }
  //cv_show_image(lp_mask,"hallo2");

  reprojectPoints(lp_src_3Dpts, lp_src_3Dpts, f);

  CvMat * lp_pts = cvCreateMat((int)l_chain_vector.size(), 4, CV_32F);
  CvMat * lp_v = cvCreateMat(4, 4, CV_32F);
  CvMat * lp_w = cvCreateMat(4, 1, CV_32F);

  for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
  {
    CV_MAT_ELEM(*lp_pts, float, l_i, 0) = (float)lp_src_3Dpts[l_i].x;
    CV_MAT_ELEM(*lp_pts, float, l_i, 1) = (float)lp_src_3Dpts[l_i].y;
    CV_MAT_ELEM(*lp_pts, float, l_i, 2) = (float)lp_src_3Dpts[l_i].z;
    CV_MAT_ELEM(*lp_pts, float, l_i, 3) = 1.0f;
  }
  cvSVD(lp_pts, lp_w, 0, lp_v);

  float l_n[4] = {CV_MAT_ELEM(*lp_v, float, 0, 3),
                  CV_MAT_ELEM(*lp_v, float, 1, 3),
                  CV_MAT_ELEM(*lp_v, float, 2, 3),
                  CV_MAT_ELEM(*lp_v, float, 3, 3)};

  float l_norm = sqrt(l_n[0] * l_n[0] + l_n[1] * l_n[1] + l_n[2] * l_n[2]);

  l_n[0] /= l_norm;
  l_n[1] /= l_norm;
  l_n[2] /= l_norm;
  l_n[3] /= l_norm;

  float l_max_dist = 0;

  for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
  {
    float l_dist =  l_n[0] * CV_MAT_ELEM(*lp_pts, float, l_i, 0) +
                    l_n[1] * CV_MAT_ELEM(*lp_pts, float, l_i, 1) +
                    l_n[2] * CV_MAT_ELEM(*lp_pts, float, l_i, 2) +
                    l_n[3] * CV_MAT_ELEM(*lp_pts, float, l_i, 3);

    if (fabs(l_dist) > l_max_dist)
      l_max_dist = l_dist;
  }
  //std::cerr << "plane: " << l_n[0] << ";" << l_n[1] << ";" << l_n[2] << ";" << l_n[3] << " maxdist: " << l_max_dist << " end" << std::endl;
  int l_minx = ap_depth->width;
  int l_miny = ap_depth->height;
  int l_maxx = 0;
  int l_maxy = 0;

  for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
  {
    l_minx = std::min(l_minx, a_chain[l_i].x);
    l_miny = std::min(l_miny, a_chain[l_i].y);
    l_maxx = std::max(l_maxx, a_chain[l_i].x);
    l_maxy = std::max(l_maxy, a_chain[l_i].y);
  }
  int l_w = l_maxx - l_minx + 1;
  int l_h = l_maxy - l_miny + 1;
  int l_nn = (int)a_chain.size();

  CvPoint * lp_chain = new CvPoint[l_nn];

  for (int l_i = 0; l_i < l_nn; ++l_i)
    lp_chain[l_i] = a_chain[l_i];

  cvFillPoly(lp_mask, &lp_chain, &l_nn, 1, cvScalar(255, 255, 255));

  delete[] lp_chain;

  //cv_show_image(lp_mask,"hallo1");

  std::vector<cv::Point3d> lp_dst_3Dpts(l_h * l_w);

  int l_ind = 0;

  for (int l_r = 0; l_r < l_h; ++l_r)
  {
    for (int l_c = 0; l_c < l_w; ++l_c)
    {
      lp_dst_3Dpts[l_ind].x = l_c + l_minx;
      lp_dst_3Dpts[l_ind].y = l_r + l_miny;
      lp_dst_3Dpts[l_ind].z = CV_IMAGE_ELEM(ap_depth, unsigned short, l_r + l_miny, l_c + l_minx);
      ++l_ind;
    }
  }
  reprojectPoints(lp_dst_3Dpts, lp_dst_3Dpts, f);

  l_ind = 0;

  for (int l_r = 0; l_r < l_h; ++l_r)
  {
    for (int l_c = 0; l_c < l_w; ++l_c)
    {
      float l_dist = (float)(l_n[0] * lp_dst_3Dpts[l_ind].x + l_n[1] * lp_dst_3Dpts[l_ind].y + lp_dst_3Dpts[l_ind].z * l_n[2] + l_n[3]);

      ++l_ind;

      if (CV_IMAGE_ELEM(lp_mask, unsigned char, l_r + l_miny, l_c + l_minx) != 0)
      {
        if (fabs(l_dist) < std::max(l_thres, (l_max_dist * 2.0f)))
        {
          for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p)
          {
            int l_col = cvRound((l_c + l_minx) / (l_p + 1.0));
            int l_row = cvRound((l_r + l_miny) / (l_p + 1.0));

            CV_IMAGE_ELEM(a_masks[l_p], unsigned char, l_row, l_col) = 0;
          }
        }
        else
        {
          for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p)
          {
            int l_col = cvRound((l_c + l_minx) / (l_p + 1.0));
            int l_row = cvRound((l_r + l_miny) / (l_p + 1.0));

            CV_IMAGE_ELEM(a_masks[l_p], unsigned char, l_row, l_col) = 255;
          }
        }
      }
    }
  }
  cvReleaseImage(&lp_mask);
  cvReleaseMat(&lp_pts);
  cvReleaseMat(&lp_w);
  cvReleaseMat(&lp_v);
}

void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f)
{
  mask = cv::Mat::zeros(depth.size(), CV_8U);
  std::vector<IplImage*> tmp;
  IplImage mask_ipl = mask;
  tmp.push_back(&mask_ipl);
  IplImage depth_ipl = depth;
  filterPlane(&depth_ipl, tmp, chain, f);
}

std::vector<CvPoint> maskFromTemplate(const std::vector<cv::linemod::Template>& templates,
                                      int num_modalities, cv::Point offset, cv::Size size,
                                      cv::Mat& mask, cv::Mat& dst)
{
  templateConvexHull(templates, num_modalities, offset, size, mask);

  const int OFFSET = 30;
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), OFFSET);

  CvMemStorage * lp_storage = cvCreateMemStorage(0);
  CvTreeNodeIterator l_iterator;
  CvSeqReader l_reader;
  CvSeq * lp_contour = 0;

  cv::Mat mask_copy = mask.clone();
  IplImage mask_copy_ipl = mask_copy;
  cvFindContours(&mask_copy_ipl, lp_storage, &lp_contour, sizeof(CvContour),
                 CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  std::vector<CvPoint> l_pts1; // to use as input to cv_primesensor::filter_plane

  cvInitTreeNodeIterator(&l_iterator, lp_contour, 1);
  while ((lp_contour = (CvSeq *)cvNextTreeNode(&l_iterator)) != 0)
  {
    CvPoint l_pt0;
    cvStartReadSeq(lp_contour, &l_reader, 0);
    CV_READ_SEQ_ELEM(l_pt0, l_reader);
    l_pts1.push_back(l_pt0);

    for (int i = 0; i < lp_contour->total; ++i)
    {
      CvPoint l_pt1;
      CV_READ_SEQ_ELEM(l_pt1, l_reader);
      /// @todo Really need dst at all? Can just as well do this outside
      cv::line(dst, l_pt0, l_pt1, CV_RGB(0, 255, 0), 2);

      l_pt0 = l_pt1;
      l_pts1.push_back(l_pt0);
    }
  }
  cvReleaseMemStorage(&lp_storage);

  return l_pts1;
}

// Adapted from cv_show_angles
cv::Mat displayQuantized(const cv::Mat& quantized)
{
  cv::Mat color(quantized.size(), CV_8UC3);
  for (int r = 0; r < quantized.rows; ++r)
  {
    const uchar* quant_r = quantized.ptr(r);
    cv::Vec3b* color_r = color.ptr<cv::Vec3b>(r);

    for (int c = 0; c < quantized.cols; ++c)
    {
      cv::Vec3b& bgr = color_r[c];
      switch (quant_r[c])
      {
        case 0:   bgr[0]=  0; bgr[1]=  0; bgr[2]=  0;    break;
        case 1:   bgr[0]= 55; bgr[1]= 55; bgr[2]= 55;    break;
        case 2:   bgr[0]= 80; bgr[1]= 80; bgr[2]= 80;    break;
        case 4:   bgr[0]=105; bgr[1]=105; bgr[2]=105;    break;
        case 8:   bgr[0]=130; bgr[1]=130; bgr[2]=130;    break;
        case 16:  bgr[0]=155; bgr[1]=155; bgr[2]=155;    break;
        case 32:  bgr[0]=180; bgr[1]=180; bgr[2]=180;    break;
        case 64:  bgr[0]=205; bgr[1]=205; bgr[2]=205;    break;
        case 128: bgr[0]=230; bgr[1]=230; bgr[2]=230;    break;
        case 255: bgr[0]=  0; bgr[1]=  0; bgr[2]=255;    break;
        default:  bgr[0]=  0; bgr[1]=255; bgr[2]=  0;    break;
      }
    }
  }

  return color;
}

// Adapted from cv_line_template::convex_hull
void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
                        int num_modalities, cv::Point offset, cv::Size size,
                        cv::Mat& dst)
{
  std::vector<cv::Point> points;
  for (int m = 0; m < num_modalities; ++m)
  {
    for (int i = 0; i < (int)templates[m].features.size(); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      points.push_back(cv::Point(f.x, f.y) + offset);
    }
  }

  std::vector<cv::Point> hull;
  cv::convexHull(points, hull);

  dst = cv::Mat::zeros(size, CV_8U);
  const int hull_count = (int)hull.size();
  const cv::Point* hull_pts = &hull[0];
  cv::fillPoly(dst, &hull_pts, &hull_count, 1, cv::Scalar(255));
}

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                        CV_RGB(0, 255, 0),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };

  for (int m = 0; m < num_modalities; ++m)
  {
    // NOTE: Original demo recalculated max response for each feature in the TxT
    // box around it and chose the display color based on that response. Here
    // the display color just depends on the modality.
    cv::Scalar color = COLORS[m];

    for (int i = 0; i < (int)templates[m].features.size(); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T / 2, color);
    }
  }
}
