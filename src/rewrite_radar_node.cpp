#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/bind.hpp"
#include <armadillo>
#include <ti_mmwave_rospkg/RadarScan.h>

using namespace arma;


void imuCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg, const ros::Publisher pub, nav_msgs::Odometry odom, char method,
                                                                          fmat A, fmat B, fmat e_r, fmat v_S, fmat v_r, int past_id)
{
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

    if (msg->range > 0.20)
    {
      e_r(0,0) = msg->x/msg->range;
      e_r(1,0) = msg->y/msg->range;
      e_r(2,0) = msg->z/msg->range;

      switch(method)
      {
        case 'A':
          v_S(0,0) = -msg->velocity*(msg->range/msg->x);

          odom.twist.twist.linear.x = v_S(0,0);
          odom.twist.twist.linear.y = v_S(1,0);
          odom.twist.twist.linear.z = v_S(2,0);

          // odom.twist.covariance[0] = sqrt(vvel);
          // odom.twist.covariance[7] = sqrt(vvel);
          // odom.twist.covariance[14] = sqrt(vvel);
          odom.header.stamp = ros::Time::now();
          pub.publish(odom);

          break;
        case 'B':
          v_S = trans(pinv(e_r)*msg->velocity);

          odom.twist.twist.linear.x = v_S(0,0);
          odom.twist.twist.linear.y = v_S(1,0);
          odom.twist.twist.linear.z = v_S(2,0);

          // odom.twist.covariance[0] = sqrt(vvel);
          // odom.twist.covariance[7] = sqrt(vvel);
          // odom.twist.covariance[14] = sqrt(vvel);
          odom.header.stamp = ros::Time::now();
          pub.publish(odom);

          break;
        case 'C':
          if (msg->point_id > past_id) {
            ROS_INFO("point_id = [%i]", msg->point_id);
            // A.row(j) = trans(e_r);
            // B.row(j) = -msg->velocity;
            A = join_vert(A, trans(e_r));
            v_r(0,0) = -msg->velocity;
            B = join_vert(B, v_r);
            A.print("A:");
            B.print("B:");
          }
          else if (msg->point_id < past_id) {
            // Solve for radar velocity
            A = A.rows(1,past_id+1);
            B = B.rows(1,past_id+1);
            v_S = solve(A,B);
            // A.print("A_end:");
            // B.print("B_end:");
            // v_S.print("v_S:");
            odom.twist.twist.linear.x = v_S(0,0);
            odom.twist.twist.linear.y = v_S(1,0);
            odom.twist.twist.linear.z = v_S(2,0);

            // odom.twist.covariance[0] = sqrt(vvel);
            // odom.twist.covariance[7] = sqrt(vvel);
            // odom.twist.covariance[14] = sqrt(vvel);

            // Publish
            odom.header.stamp = ros::Time::now();
            pub.publish(odom);

            A = join_vert(A, trans(e_r));
            v_r(0,0) = -msg->velocity;
            B = join_vert(B, v_r);
          }
          break;
        }
      }
      past_id++;

      return A
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "rewrite_radar");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("radar_odom", 100);

  fmat A(1,3, fill::zeros);
  fmat B(1,1, fill::zeros);
  fmat e_r(3,1, fill::randu);
  fmat v_S(3,1, fill::zeros);
  fmat v_r(1,1, fill::zeros);

  char method = 'C'; //Options are: A = no slip, B = pinv or C = LS

  int past_id = -1;

  nav_msgs::Odometry odom;

  ros::Subscriber sub = nh.subscribe<ti_mmwave_rospkg::RadarScan>("/ti_mmwave/radar_scan", 100, boost::bind(imuCallback, _1, pub, odom, method,
                                                                                                            A, B, e_r, v_S, v_r, past_id));
  ros::spin();
}
