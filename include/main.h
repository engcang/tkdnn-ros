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

///// utils
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////


class tkdnn_ros{
  public:
    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber laser_sub;
    ros::Publisher frontier_sensor_end_pub;
    ros::Timer octo_update_and_publisher;

    // void state_callback(const mavros_msgs::State::ConstPtr& msg);

    tkdnn_ros(ros::NodeHandle& n) : nh(n){
      ///// params
      // nh.param<std::string>("/octomap_topic", octomap_topic, "/octomap");
      // nh.param("/frontier_visualize", frontier_visualize, true);

      // path = ros::package::getPath("oakd-ros");
      ///// sub pub
      // laser_sub = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 10, &active_slam::laser_callback, this);
      // global_nodes_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_nodes", 10);
      // octo_update_and_publisher = nh.createTimer(ros::Duration(1/octomap_hz), &active_slam::octomap_Timer, this); // every hz

      ROS_WARN("class heritated, starting node...");
    }
};



// void tkdnn_ros::state_callback(const mavros_msgs::State::ConstPtr& msg){
  // curr_state=*msg;
// }



#endif