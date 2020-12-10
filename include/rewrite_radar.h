#ifndef _REWRITE_RADAR_CLASS_
#define _REWRITE_RADAR_CLASS_


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <armadillo>
#include <ti_mmwave_rospkg/RadarScan.h>

using namespace arma;

class Rewrite_Radar{

public:

  Rewrite_Radar(fmat A_0, fmat B_0, ros::NodeHandle nh)
    {
      A = A_0;
      B = B_0;
      W = B_0;
      e_r = trans(A_0);
      v_r = B_0;
      actual_weight = B_0;
      v_S = trans(A_0);

      vvel = 0.038; // radial velocity tolerance [m/s]
      method = 3; //Options are: 1 = no slip, 2 = pinv or 3 = LS
      LSmethod = 1; //Options are: 1 = normal, 2 = weighted, 3 = 3 best
      past_id = -1;
      pub = nh.advertise<nav_msgs::Odometry>("radar_odom", 100);

      A.print("A initialized:");
      B.print("B initialized:");
      W.print("W initialized:");
      ROS_INFO("vvel = [%f], method = [%i], LSmethod = [%i], past_id = [%i]", vvel, method, LSmethod, past_id);

    };

  void radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg);
  // void initialize_variables();
  void initialize_subscriber(Rewrite_Radar rewrite_radar);

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

  int method = 0;
  int LSmethod;

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
