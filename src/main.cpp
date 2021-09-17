#include "main.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "oakd_ros_node");
  ros::NodeHandle n("~");

  tkdnn_ros_class tkdnn(n);

  signal(SIGINT, signal_handler); // to exit program when ctrl+c

  ros::AsyncSpinner spinner(2); // Use 2 threads -> 1 callbacks + 2 publisher
  spinner.start();
  ros::waitForShutdown();

  return 0;
}