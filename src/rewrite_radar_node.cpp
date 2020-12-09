#include "rewrite_radar.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "rewrite_radar");
  Rewrite_Radar rewrite_radar;
  rewrite_radar.initialize_variables();
  rewrite_radar.initialize_sub_pub();
  ros::spin();
}
