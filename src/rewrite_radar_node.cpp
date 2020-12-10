#include "rewrite_radar.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "rewrite_radar_node");
  fmat A_0(1,3, fill::zeros);
  fmat B_0(1,1, fill::zeros);
  ros::NodeHandle nh;
  Rewrite_Radar rewrite_radar(A_0, B_0, nh);

  rewrite_radar.initialize_subscriber(rewrite_radar);

  return 0;
}
