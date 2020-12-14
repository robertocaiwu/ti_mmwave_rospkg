#include "rewrite_radar.h"


void Rewrite_Radar::radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg) {

  if (msg->range > 0.20)
  {
    // ROS_INFO("distance of point [%i] is enough big", msg->point_id);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.header.stamp = ros::Time::now();

    elevation = msg->elevation*3.14159265359/180;
    azimuth = msg->azimuth*3.14159265359/180;

    e_r(0,0) = msg->x/msg->range;
    e_r(1,0) = msg->y/msg->range;
    e_r(2,0) = msg->z/msg->range;

    v_r(0,0) = -msg->velocity;

    actual_weight(0,0) = cos(elevation)*cos(azimuth);
    // e_r.print("current e_r:");

    if (method == 1) {
      v_S(0,0) = -msg->velocity*(msg->range/msg->x);

      odom.twist.twist.linear.x = v_S(0,0);
      odom.twist.twist.linear.y = v_S(1,0);
      odom.twist.twist.linear.z = v_S(2,0);

      odom.twist.covariance[0] = sqrt(vvel);
      odom.twist.covariance[7] = sqrt(vvel);
      odom.twist.covariance[14] = sqrt(vvel);

      pub.publish(odom);
    }

    else if (method == 2) {
      v_S = trans(pinv(e_r)*msg->velocity);

      odom.twist.twist.linear.x = v_S(0,0);
      odom.twist.twist.linear.y = v_S(1,0);
      odom.twist.twist.linear.z = v_S(2,0);

      odom.twist.covariance[0] = sqrt(vvel);
      odom.twist.covariance[7] = sqrt(vvel);
      odom.twist.covariance[14] = sqrt(vvel);

      pub.publish(odom);
    }

    else if (method == 3) {
      if (LSmethod == 1) {
        ROS_INFO("point_id = [%i]", msg->point_id);
        ROS_INFO("past_id = [%i]", past_id);
        if (past_id < msg->point_id) {
          ROS_INFO("past_id < point_id");
          A.print("A:");
          A = join_vert(A, trans(e_r));
          B = join_vert(B, v_r);
        }
        else if (past_id > msg->point_id) {
          ROS_INFO("past_id > point_id");
          A.print("A_end:");
          B.print("B_end:");

          v_S = solve(A,B);
          odom.twist.twist.linear.x = v_S(0,0);
          odom.twist.twist.linear.y = v_S(1,0);
          odom.twist.twist.linear.z = v_S(2,0);

          odom.twist.covariance[0] = sqrt(vvel);
          odom.twist.covariance[7] = sqrt(vvel);
          odom.twist.covariance[14] = sqrt(vvel);

          pub.publish(odom);

          A = trans(e_r);
          B = v_r;
          past_id = -1;
        }

      }

      else if (LSmethod == 2) {
        if (past_id < msg->point_id) {
          A = join_vert(A, trans(e_r));
          B = join_vert(B, v_r);
          W = join_vert(W, actual_weight);
        }
        else if (past_id > msg->point_id) {

          W = W/as_scalar(sum(W)); // normalize W

          A_weighted = trans(A)*diagmat(W)*A;
          B_weighted = trans(A)*diagmat(W)*B;

          v_S = solve(A_weighted,B_weighted);
          // v_S.print("v_S:");
          odom.twist.twist.linear.x = v_S(0,0);
          odom.twist.twist.linear.y = v_S(1,0);
          odom.twist.twist.linear.z = v_S(2,0);

          odom.twist.covariance[0] = sqrt(vvel);
          odom.twist.covariance[7] = sqrt(vvel);
          odom.twist.covariance[14] = sqrt(vvel);

          pub.publish(odom);

          A = trans(e_r);
          v_r(0,0) = -msg->velocity;
          B = v_r;
          W = actual_weight;
          past_id = -1;
        }
      }

      else if (LSmethod == 3) {
        if (past_id < msg->point_id) {
          A = join_vert(A, trans(e_r));
          B = join_vert(B, v_r);
          W = join_vert(W, actual_weight);
        }
        else if (past_id > msg->point_id) {
          W = W/as_scalar(sum(W)); // normalize W
          // Take the 3 best measured points for LS
          uvec indices = sort_index(W, "descend");
          fmat A_best(3,3, fill::zeros);
          fmat B_best(3,1, fill::zeros);

          indices.print("indices:");
          for (int i = 0 ; i<3; i++) {
            A_best.row(i) = A.row(indices(i));
            B_best(i,0) = B(indices(i),0);
          }
          A_best.print("A_best:");
          B_best.print("B_best:");

          v_S = solve(A_best,B_best);

          odom.twist.twist.linear.x = v_S(0,0);
          odom.twist.twist.linear.y = v_S(1,0);
          odom.twist.twist.linear.z = v_S(2,0);

          odom.twist.covariance[0] = sqrt(vvel);
          odom.twist.covariance[7] = sqrt(vvel);
          odom.twist.covariance[14] = sqrt(vvel);

          pub.publish(odom);

          A = trans(e_r);
          B = v_r;
          W = actual_weight;
          past_id = -1;
        }
      }

      else {
        ROS_ERROR("LSmethod does not exist!");
      }
    }

    else {
      ROS_ERROR("Method [%i] does not exist!", method);
    }
  }
  past_id++;
}

void Rewrite_Radar::initialize_subscriber(Rewrite_Radar rewrite_radar) {
  sub = nh.subscribe<ti_mmwave_rospkg::RadarScan>("/ti_mmwave/radar_scan", 100, &Rewrite_Radar::radarCallback, &rewrite_radar);
  ROS_INFO("Subscriber initialized.");
  ros::spin();
 }
