#include "al5d_pckg/hld.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hld");

  hld::hld hld("AL5D", "/dev/ttyUSB0");

  ros::spin();
  //exit
  return 0;
}