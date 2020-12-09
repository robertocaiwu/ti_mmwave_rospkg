#ifndef _REWRITE_RADAR_CLASS_
#define _REWRITE_RADAR_CLASS_


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/bind.hpp"
#include <armadillo>
#include <ti_mmwave_rospkg/RadarScan.h>

using namespace arma;

class Rewrite_Radar{

public:
  void radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg);
  void initialize_variables();
  void initialize_sub_pub();

private:
  fmat A;
  fmat B;
  fmat A_best;
  fmat B_best;
  fmat A_weighted;
  fmat B_weighted;
  fmat W;
  fmat actual_weight;
  fmat e_r;
  fmat v_r;
  fmat v_S;
  uvec indices;

  char method;
  char LSmethod;

  float vvel;
  float elevation;
  float azimuth;

  int past_id;

  ros::Publisher pub;
  ros::Subscriber sub;
  ros::NodeHandle nh;

  nav_msgs::Odometry odom;

};

#endif
