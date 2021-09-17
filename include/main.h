#ifndef TKDNNROS_H
#define TKDNNROS_H

///// common headers
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
    sensor_msgs::CompressedImage img_msg;
    string path, rt_file, image_topic;
    int class_number;
    float confidence_thresh;
    std::vector<cv::Mat> batch_frame;
    std::vector<cv::Mat> batch_dnn_input;

    tk::dnn::Yolo3Detection yolo;
    tk::dnn::DetectionNN *detNN; 

    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber img_sub;
    ros::Publisher detected_img_pub, bounding_box_pub;

    void img_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    tkdnn_ros_class(ros::NodeHandle& n) : nh(n){
      ///// params
      nh.param<std::string>("/rt_file", rt_file, "yolo4tiny_fp32.rt");
      nh.param<std::string>("/image_topic", image_topic, "/image_raw");
      nh.param("/class_number", class_number, 1);
      nh.param<float>("/confidence_thresh", confidence_thresh, 0.9);

      ///// sub pub
      img_sub = nh.subscribe<sensor_msgs::CompressedImage>(image_topic, 10, &tkdnn_ros_class::img_callback, this);
      detected_img_pub = nh.advertise<sensor_msgs::CompressedImage>("/detected_output"+image_topic, 10);
      bounding_box_pub = nh.advertise<tkdnn_ros::bboxes>("/detected_bounding_boxes", 10);

      path = ros::package::getPath("tkdnn_ros");
      detNN = &yolo;
      detNN->init(path+"/rt_file/"+rt_file, class_number, 1, confidence_thresh);
      //detNN->init(rt_file, class_number, batch size, confidence_thresh);

      ROS_WARN("class heritated, starting node...");
    }
};



void tkdnn_ros_class::img_callback(const sensor_msgs::CompressedImage::ConstPtr& msg){
  cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(*msg);

  batch_dnn_input.clear();
  batch_frame.clear();

  batch_frame.push_back(img_ptr->image);
  // this will be resized to the net format
  batch_dnn_input.push_back(img_ptr->image.clone());

  detNN->update(batch_dnn_input, 1); //batch_size
  detNN->draw(batch_frame);
  
  cv::Mat out_image = batch_frame.back();

  char fps[40];
  sprintf(fps, "%.3f ms spent for inference", detNN->stats.back());
  cv::putText(out_image, string(fps), cv::Point(0, 25), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 0, 120), 2);

  header.stamp = ros::Time::now();
  cv_bridge::CvImage bridge_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, out_image);;
  bridge_img.toCompressedImageMsg(img_msg);
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
  bounding_box_pub.publish(out_boxes);
}



#endif