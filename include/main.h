#ifndef TKDNNROS_H
#define TKDNNROS_H

///// common headers
#include <time.h>
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


///// utils
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////


class tkdnn_ros_class{
  public:

    ///// tkDNN
    std_msgs::Header header;
    sensor_msgs::CompressedImage comp_img_msg;
    sensor_msgs::Image img_msg;
    string path, rt_file, image_topic;
    int class_number, downsampling_inference, counter=0;
    float confidence_thresh;
    std::vector<cv::Mat> batch_frame;
    std::vector<cv::Mat> batch_dnn_input;

    tk::dnn::Yolo3Detection yolo;
    tk::dnn::DetectionNN *detNN; 

    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber img_sub;
    ros::Publisher detected_img_pub, bounding_box_pub;

    ///// rectification, optional
    bool image_compressed, save_image, need_image_recitifaction, fisheye_image;
    vector<double> intrinsic, distortion, resolution;
    cv::Mat map1, map2;

    void img_callback(const sensor_msgs::Image::ConstPtr& msg);
    void comp_img_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    tkdnn_ros_class(ros::NodeHandle& n) : nh(n){
      ///// params
      nh.param<bool>("/image_compressed", image_compressed, true);
      nh.param<std::string>("/rt_file", rt_file, "/rt_file/yolo4tiny_fp32.rt");
      nh.param<std::string>("/image_topic", image_topic, "/image_raw");
      nh.param("/class_number", class_number, 1);
      nh.param<float>("/confidence_thresh", confidence_thresh, 0.9);

      nh.param<bool>("/save_image", save_image, false);
      nh.param("/downsampling_inference", downsampling_inference, 1);
      nh.param<bool>("/need_image_recitifaction", need_image_recitifaction, false);
      nh.param<bool>("/fisheye_image", fisheye_image, false);
      nh.getParam("/intrinsic", intrinsic);
      nh.getParam("/distortion", distortion);
      nh.getParam("/resolution", resolution);

      if (need_image_recitifaction){
        cv::Size img_size = {resolution[0], resolution[1]};
        double cm[] = {intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5], intrinsic[6], intrinsic[7], intrinsic[8]};
        double dm[] = {distortion[0], distortion[1], distortion[2], distortion[3]};
        cv::Mat K_intrinsic = cv::Mat(3,3,CV_64FC1,(void*)cm);
        cv::Mat D_distortion = cv::Mat(1,4,CV_64FC1,(void*)dm);
        if (fisheye_image)
          cv::fisheye::initUndistortRectifyMap(K_intrinsic, D_distortion, cv::Mat(), K_intrinsic, img_size, CV_32FC1, map1, map2);
        else
          cv::initUndistortRectifyMap(K_intrinsic, D_distortion, cv::Mat(), K_intrinsic, img_size, CV_32FC1, map1, map2);
      }

      path = ros::package::getPath("tkdnn_ros");
      detNN = &yolo;
      detNN->init(path+rt_file, class_number, 1, confidence_thresh);
      //detNN->init(rt_file, class_number, batch size, confidence_thresh);

      ///// sub pub
      if (image_compressed){
        img_sub = nh.subscribe<sensor_msgs::CompressedImage>(image_topic, 10, &tkdnn_ros_class::comp_img_callback, this);
        detected_img_pub = nh.advertise<sensor_msgs::CompressedImage>("/detected_output"+image_topic, 10);
      }
      else{
        img_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 10, &tkdnn_ros_class::img_callback, this);
        detected_img_pub = nh.advertise<sensor_msgs::Image>("/detected_output"+image_topic, 10); 
      }
      bounding_box_pub = nh.advertise<tkdnn_ros::bboxes>("/detected_bounding_boxes", 10);


      ROS_WARN("class heritated, starting node...");
    }
};


void tkdnn_ros_class::comp_img_callback(const sensor_msgs::CompressedImage::ConstPtr& msg){
  counter++;
  if (counter%downsampling_inference==0){
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat img_in = img_ptr->image;
    if (need_image_recitifaction)
      cv::remap(img_in, img_in, map1, map2, cv::INTER_LINEAR);

    batch_dnn_input.clear();
    batch_frame.clear();

    batch_frame.push_back(img_in);
    // this will be resized to the net format
    batch_dnn_input.push_back(img_in.clone());

    detNN->update(batch_dnn_input, 1); //batch_size
    detNN->draw(batch_frame);
    
    cv::Mat out_image = batch_frame.back();

    char fps[40], date_time[40];
    sprintf(fps, "%.3f ms spent for inference", detNN->stats.back());
    cv::putText(out_image, string(fps), cv::Point(5, 25), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 0, 200), 2);
    time_t timer = time(NULL); struct tm* t; t = localtime(&timer);
    if (t){ // not NULL
      sprintf(date_time, "%d-%d-%d_%d:%d:%d__%d", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, counter);
      cv::putText(out_image, string(date_time), cv::Point(5, 50), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 50, 50), 2);
    }

    // header.stamp = ros::Time::now();
    header.stamp = msg->header.stamp;
    cv_bridge::CvImage bridge_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, out_image);
    bridge_img.toCompressedImageMsg(comp_img_msg);
    detected_img_pub.publish(comp_img_msg);

    tkdnn_ros::bboxes out_boxes;
    for (int i = 0; i < detNN->batchDetected[0].size(); ++i){
      tkdnn_ros::bbox out_box;
      tk::dnn::box b = detNN->batchDetected[0][i];

      out_box.score = b.prob;
      out_box.x = b.x;
      out_box.y = b.y;
      out_box.width = b.w;
      out_box.height = b.h;
      out_box.id = b.cl;
      out_box.Class = detNN->classesNames[b.cl];
      out_boxes.bboxes.push_back(out_box);
    }
    if (out_boxes.bboxes.size()>0){
      bounding_box_pub.publish(out_boxes);
      if (save_image && t){
        cv::imwrite(path+"/image/"+date_time+".jpg", out_image);
      }
    }
  }
}


void tkdnn_ros_class::img_callback(const sensor_msgs::Image::ConstPtr& msg){
  counter++;
  if (counter%downsampling_inference==0){
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat img_in = img_ptr->image;
    if (need_image_recitifaction)
      cv::remap(img_in, img_in, map1, map2, cv::INTER_LINEAR);

    batch_dnn_input.clear();
    batch_frame.clear();

    batch_frame.push_back(img_in);
    // this will be resized to the net format
    batch_dnn_input.push_back(img_in.clone());

    detNN->update(batch_dnn_input, 1); //batch_size
    detNN->draw(batch_frame);
    
    cv::Mat out_image = batch_frame.back();

    char fps[40], date_time[40];
    sprintf(fps, "%.3f ms spent for inference", detNN->stats.back());
    cv::putText(out_image, string(fps), cv::Point(5, 25), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 0, 200), 2);
    time_t timer = time(NULL); struct tm* t; t = localtime(&timer);
    if (t){ // not NULL
      sprintf(date_time, "%d-%d-%d_%d:%d:%d__%d", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, counter);
      cv::putText(out_image, string(date_time), cv::Point(5, 50), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 50, 50), 2);
    }

    // header.stamp = ros::Time::now();
    header.stamp = msg->header.stamp;
    cv_bridge::CvImage bridge_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, out_image);
    bridge_img.toImageMsg(img_msg);
    detected_img_pub.publish(img_msg);

    tkdnn_ros::bboxes out_boxes;
    for (int i = 0; i < detNN->batchDetected[0].size(); ++i){
      tkdnn_ros::bbox out_box;
      tk::dnn::box b = detNN->batchDetected[0][i];

      out_box.score = b.prob;
      out_box.x = b.x;
      out_box.y = b.y;
      out_box.width = b.w;
      out_box.height = b.h;
      out_box.id = b.cl;
      out_box.Class = detNN->classesNames[b.cl];
      out_boxes.bboxes.push_back(out_box);
    }
    if (out_boxes.bboxes.size()>0){
      bounding_box_pub.publish(out_boxes);
      if (save_image && t){
        cv::imwrite(path+"/image/"+date_time+".jpg", out_image);
      }
    }
  }
}



#endif