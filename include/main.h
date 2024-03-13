#ifndef TKDNN_YOLO_ROS_H
#define TKDNN_YOLO_ROS_H

///// common headers
#include <ctime>
#include <ros/ros.h>
#include <ros/package.h>

///// image processing
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

///// tkDNN
#include "Yolo3Detection.h"
#include <tkdnn_ros/bboxes.h>


////////////////////////////////////////////////////////////////////////////////////////////////////
class TkdnnYoloRos
{
private:
    ///// tkDNN
    std::string m_package_path;
    int m_downsampling_inference, m_counter=0;
    tk::dnn::Yolo3Detection m_yolo;
    tk::dnn::DetectionNN *m_detNN; 
    ///// rectification, optional
    bool m_image_compressed, m_save_image, m_image_rectification_enabled;
    cv::Mat m_img_map1, m_img_map2;
    ///// ros and tf
    ros::NodeHandle m_nh;
    ros::Subscriber m_img_sub;
    ros::Publisher m_detected_img_pub, m_bounding_box_pub;
    ///// Functions
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void compressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void processImage(cv::Mat& img_in, const double& time, const bool& is_compressed);
public:
    TkdnnYoloRos(const ros::NodeHandle& n); // constructor
    ~TkdnnYoloRos(){}; // destructor
};

TkdnnYoloRos::TkdnnYoloRos(const ros::NodeHandle& n) : m_nh(n)
{
  // temporal variables
  bool fisheye_image_;
  std::vector<double> intrinsic_, distortion_, resolution_;
  std::string image_topic_, rt_file_path_;
  int class_number_;
  float confidence_thresh_;
  ///// params
  m_nh.param<bool>("/tkdnn_ros/image_compressed", m_image_compressed, true);
  m_nh.param<std::string>("/tkdnn_ros/image_topic", image_topic_, "/image_raw");
  m_nh.param<std::string>("/tkdnn_ros/rt_file", rt_file_path_, "/rt_file/yolo4tiny_fp32.rt");
  m_nh.param<int>("/tkdnn_ros/class_number", class_number_, 1);
  m_nh.param<float>("/tkdnn_ros/confidence_thresh", confidence_thresh_, 0.9);
  m_nh.param<int>("/tkdnn_ros/downsampling_inference", m_downsampling_inference, 1);
  m_nh.param<bool>("/tkdnn_ros/save_image", m_save_image, false);
  m_nh.param<bool>("/tkdnn_ros/image_rectification/enable", m_image_rectification_enabled, false);
  m_nh.param<bool>("/tkdnn_ros/image_rectification/fisheye_image", fisheye_image_, false);
  m_nh.param<std::vector<double>>("/tkdnn_ros/image_rectification/intrinsic", intrinsic_, std::vector<double>(9, 0));
  m_nh.param<std::vector<double>>("/tkdnn_ros/image_rectification/distortion", distortion_, std::vector<double>(4, 0));
  m_nh.param<std::vector<double>>("/tkdnn_ros/image_rectification/resolution", resolution_, std::vector<double>(2, 0));
  ///// Variables
  // rectification
  if (m_image_rectification_enabled)
  {
    cv::Size img_size_ = {resolution_[0], resolution_[1]};
    double k_mat_[] = {intrinsic_[0], intrinsic_[1], intrinsic_[2], intrinsic_[3], intrinsic_[4], intrinsic_[5], intrinsic_[6], intrinsic_[7], intrinsic_[8]};
    double d_mat_[] = {distortion_[0], distortion_[1], distortion_[2], distortion_[3]};
    cv::Mat K_intrinsic_ = cv::Mat(3, 3, CV_64FC1, (void*)k_mat_);
    cv::Mat D_distortion_ = cv::Mat(1, 4, CV_64FC1, (void*)d_mat_);
    if (fisheye_image_)
    {
      cv::fisheye::initUndistortRectifyMap(K_intrinsic_, D_distortion_, cv::Mat(), K_intrinsic_, img_size_, CV_32FC1, m_img_map1, m_img_map2);
    }
    else
    {
      cv::initUndistortRectifyMap(K_intrinsic_, D_distortion_, cv::Mat(), K_intrinsic_, img_size_, CV_32FC1, m_img_map1, m_img_map2);
    }
  }
  // tkDNN
  m_package_path = ros::package::getPath("tkdnn_ros");
  m_detNN = &m_yolo;
  m_detNN->init(m_package_path + rt_file_path_, class_number_, 1, confidence_thresh_);
  ///// sub pub
  if (m_image_compressed)
  {
    m_img_sub = m_nh.subscribe<sensor_msgs::CompressedImage>(image_topic_, 10, &TkdnnYoloRos::compressedImageCallback, this);
    m_detected_img_pub = m_nh.advertise<sensor_msgs::CompressedImage>("/tkdnn_ros/detected_output" + image_topic_, 10);
  }
  else
  {
    m_img_sub = m_nh.subscribe<sensor_msgs::Image>(image_topic_, 10, &TkdnnYoloRos::imageCallback, this);
    m_detected_img_pub = m_nh.advertise<sensor_msgs::Image>("/tkdnn_ros/detected_output" + image_topic_, 10); 
  }
  m_bounding_box_pub = m_nh.advertise<tkdnn_ros::bboxes>("/tkdnn_ros/detected_bounding_boxes", 10);

  ROS_WARN("class heritated, starting node...");
}

void TkdnnYoloRos::processImage(cv::Mat& img_in, const double& time, const bool& is_compressed)
{
  // prepare image
  if (m_image_rectification_enabled)
  {
    cv::remap(img_in, img_in, m_img_map1, m_img_map2, cv::INTER_LINEAR);
  }
  std::vector<cv::Mat> batch_frame_;
  std::vector<cv::Mat> batch_dnn_input_;
  batch_frame_.push_back(img_in);
  batch_dnn_input_.push_back(img_in.clone()); // note image will be resized to the net format

  // infer and draw
  std::chrono::high_resolution_clock::time_point start_time_ = std::chrono::high_resolution_clock::now();
  m_detNN->update(batch_dnn_input_, 1); // note batch_size = 1
  m_detNN->draw(batch_frame_);
  std::chrono::high_resolution_clock::time_point end_time_ = std::chrono::high_resolution_clock::now();

  // handle output
  cv::Mat out_image_ = batch_frame_.back();
  tkdnn_ros::bboxes out_boxes_;
  out_boxes_.header.stamp = ros::Time().fromSec(time);
  for (size_t i = 0; i < m_detNN->batchDetected[0].size(); ++i)
  {
    tkdnn_ros::bbox out_box_;
    tk::dnn::box b = m_detNN->batchDetected[0][i];
    out_box_.score = b.prob;
    out_box_.x = b.x;
    out_box_.y = b.y;
    out_box_.width = b.w;
    out_box_.height = b.h;
    out_box_.id = b.cl;
    out_box_.Class = m_detNN->classesNames[b.cl];
    out_boxes_.bboxes.push_back(out_box_);
  }
  
  // publish
  if (out_boxes_.bboxes.size()>0)
  {
    m_bounding_box_pub.publish(out_boxes_);
  }

  // draw and publish
  char fps_[40], date_time_[40];
  std::sprintf(fps_, "%.2f ms infer + draw", std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count()/1e3);
  cv::putText(out_image_, std::string(fps_), cv::Point(5, 18), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 200), 2);

  std::time_t timer_ = std::time(NULL);
  struct std::tm* t_;
  t_ = std::localtime(&timer_);
  if (t_) // not NULL
  {
    std::sprintf(date_time_, "%d-%d-%d_%d:%d:%d__%d", t_->tm_year+1900, t_->tm_mon+1, t_->tm_mday, t_->tm_hour, t_->tm_min, t_->tm_sec, m_counter);
    cv::putText(out_image_, std::string(date_time_), cv::Point(5, 36), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 50, 50), 2);
  }
  cv_bridge::CvImage bridge_img = cv_bridge::CvImage(out_boxes_.header, sensor_msgs::image_encodings::BGR8, out_image_);
  if (is_compressed)
  {
    sensor_msgs::CompressedImage comp_img_msg_;
    bridge_img.toCompressedImageMsg(comp_img_msg_);
    m_detected_img_pub.publish(comp_img_msg_);
  }
  else
  {
    sensor_msgs::Image img_msg_;
    bridge_img.toImageMsg(img_msg_);
    m_detected_img_pub.publish(img_msg_);
  }

  // save image
  if (m_save_image && t_)
  {
    cv::imwrite(m_package_path + "/image/" + date_time_ + ".jpg", out_image_);
  }

  return;
}
void TkdnnYoloRos::compressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
  m_counter++;
  if (m_counter % m_downsampling_inference==0)
  {
    cv::Mat img_in_ = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
    processImage(img_in_, msg->header.stamp.toSec(), true);
  }
}
void TkdnnYoloRos::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  m_counter++;
  if (m_counter % m_downsampling_inference==0)
  {
    cv::Mat img_in_ = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
    processImage(img_in_, msg->header.stamp.toSec(), false);
  }
}


#endif
