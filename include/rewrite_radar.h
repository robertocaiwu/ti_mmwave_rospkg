#ifndef _REWRITE_RADAR_CLASS_
#define _REWRITE_RADAR_CLASS_


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <armadillo>
#include <ti_mmwave_rospkg/RadarScan.h>
#include <iostream>

using namespace arma;
using namespace std;

class Rewrite_Radar{

public:

  Rewrite_Radar(fmat A_0, fmat B_0, ros::NodeHandle nh)
    {
      angl_vel = {0, 0, 0};
      r_r2i = {7.5e-3, -38e-3, -110.4e-3};
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
      LSmethod = 2; //Options are: 1 = normal, 2 = weighted
      cov_method = 1; // Options are: 1 = zero, 2 = linearization, 3 = Monte Carlo
      cov_param = 1.0;
      ransac = true; // true is with RANSAC, false without RANSAC
      past_id = -1;
      pub = nh.advertise<nav_msgs::Odometry>("radar_odom", 100);
      points_pub = nh.advertise<nav_msgs::Odometry>("plc_info", 100);
      click = 0;
      vel_meas = B_0;
      az_meas = B_0;
      el_meas = B_0;
      actual_meas = B_0;
      N = 0;
      N_opt = 0;
      outliers = 0;
      delta_angle = 1e-4;
      delta_vel = 1e-4;
      K = datum::pi/12;
      cout << "K = " << K << "\n";
      N_MC = 100;
      A.print("A initialized:");
      B.print("B initialized:");
      W.print("W initialized:");
      ROS_INFO("vvel = [%f], LSmethod = [%i], cov_method = [%i], ransac = [%i] past_id = [%i], delta_vel = [%f], delta_angle = [%f]",
       vvel, LSmethod, cov_method, ransac, past_id, delta_vel, delta_angle);
    };

  void radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  // void initialize_variables();
  void initialize_subscriber(Rewrite_Radar rewrite_radar);

  // for covariance computation
  fmat approx_error_propagation();
  fmat compute_jacobian_approx();
  fmat compute_M(float d_angl, int I, string variable);

  fmat MC_error_propagation();

  fmat RANSAC();

private:
  fmat A;
  fmat B;
  fmat A_best;
  fmat B_best;
  fmat A_weighted;
  fmat B_weighted;
  fmat W;
  fmat W_norm;
  fmat actual_weight;
  fmat e_r;
  fmat v_r;
  fmat v_S;
  vec v_B;
  fmat cov_vS;
  uvec indices;
  fmat R;
  vec angl_vel;
  vec r_r2i; //position vector from radar to imu frame

  bool ransac;
  int LSmethod;
  int cov_method;
  int click;
  float cov_param;

  float K;
  int N_MC;
  
  float vvel;
  float elevation;
  float azimuth;

  int past_id;

  ros::Publisher points_pub;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Subscriber sub_imu;
  ros::NodeHandle nh;
  nav_msgs::Odometry odom;
  nav_msgs::Odometry plc_info;
  // for covariance computation
  int N;
  int N_opt;
  int outliers;

  float dv;
  float delta_angle;
  float delta_vel;

  fmat vel_meas;
  fmat az_meas;
  fmat el_meas;
  fmat actual_meas;

};

#endif
