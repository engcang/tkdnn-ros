#include "main.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "oakd_ros_node");
  ros::NodeHandle n("~");

  tkdnn_ros tkdnn(n);

  signal(SIGINT, signal_handler); // to exit program when ctrl+c

  ros::AsyncSpinner spinner(7); // Use 7 threads -> 3 callbacks + 4 Timer callbacks
  spinner.start();
  ros::waitForShutdown();

  return 0;
}