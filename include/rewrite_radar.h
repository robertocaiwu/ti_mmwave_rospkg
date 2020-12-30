#ifndef _REWRITE_RADAR_CLASS_
#define _REWRITE_RADAR_CLASS_


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <armadillo>
#include <ti_mmwave_rospkg/RadarScan.h>
#include <iostream>

using namespace arma;
using namespace std;

class Rewrite_Radar{

public:

  Rewrite_Radar(fmat A_0, fmat B_0, ros::NodeHandle nh)
    {
      A = A_0;
      B = B_0;
      W = B_0;
      A_weighted = A_0;
      B_weighted = B_0;
      e_r = trans(A_0);
      v_r = B_0;
      actual_weight = B_0;
      v_S = trans(A_0);
      vvel = 0.038; // radial velocity tolerance [m/s]
      method = 3; //Options are: 1 = no slip, 2 = pinv or 3 = LS
      LSmethod = 1; //Options are: 1 = normal, 2 = weighted, 3 = 3 best
      past_id = -1;
      pub = nh.advertise<nav_msgs::Odometry>("radar_odom", 100);
      click = 0;
      vel_meas = B_0;
      az_meas = B_0;
      el_meas = B_0;
      actual_meas = B_0;
      dv = 0.038;
      N = 0;
      delta_angle = 1e-4;
      delta_vel = 1e-4;
      K = datum::pi/12;
      cout << "K = " << K << "\n";
      N_MC = 100;
      A.print("A initialized:");
      B.print("B initialized:");
      W.print("W initialized:");
      ROS_INFO("vvel = [%f], method = [%i], LSmethod = [%i], past_id = [%i], delta_vel = [%f], delta_angle = [%f]",
       vvel, method, LSmethod, past_id, delta_vel, delta_angle);

       // Rotation matrix to vicon frame
      // R = { {-0.9998, 0.0001, 0.0197}, {0.0023, 0.9938, 0.1116}, {-0.0196, 0.1116, -0.9936} };
      // R = inv(R);
    };

  void radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg);
  // void initialize_variables();
  void initialize_subscriber(Rewrite_Radar rewrite_radar);

  // for covariance computation
  fmat approx_error_propagation();
  fmat compute_jacobian_approx();
  fmat compute_M(float d_angl, int I, string variable);

  fmat MC_error_propagation();

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
  fmat R;

  int method;
  int LSmethod;
  int click;

  float K;
  int N_MC;

  float vvel;
  float elevation;
  float azimuth;

  int past_id;

  ros::Publisher pub;
  ros::Subscriber sub;
  ros::NodeHandle nh;
  nav_msgs::Odometry odom;

  // for covariance computation
  int N;

  float dv;
  float delta_angle;
  float delta_vel;

  fmat vel_meas;
  fmat az_meas;
  fmat el_meas;
  fmat actual_meas;

};

#endif
