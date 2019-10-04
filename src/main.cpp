// ROS dependencies
#include "ros/ros.h"

// Internal dependencies
#include "xsens_mtw/Wrapper.h"

int main(int argc, char* argv[])
{
  std::string node_name = "hiros_xsens_mtw_wrapper";
  ros::init(argc, argv, node_name);

  hiros::xsens_mtw::Wrapper wrapper;
  wrapper.start();
  wrapper.run();

  return 0;
}
