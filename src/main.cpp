#include "main.h"
#include <signal.h>
void signal_handler(sig_atomic_t s)
{
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tkdnn_ros_node");
  ros::NodeHandle n("~");

  TkdnnYoloRos tkdnnyoloros_(n);

  signal(SIGINT, signal_handler); // to exit program when ctrl+c

  ros::AsyncSpinner spinner(2); // Use 2 threads -> 1 callbacks + 2 publisher
  spinner.start();
  ros::waitForShutdown();

  return 0;
}