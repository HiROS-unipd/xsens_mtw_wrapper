// ROS
#include "ros/ros.h"

// Internal dependencies
#include "xsens_mtw/XsensMtw.h"

int main(int argc, char* argv[])
{
  std::string node_name = "xsens_mtw_driver";
  ros::init(argc, argv, node_name);

  xsens::mtw::XsensMtw xsens_mtw;
  xsens_mtw.start();
  xsens_mtw.run();

  return 0;
}
