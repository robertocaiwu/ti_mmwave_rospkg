#include "rewrite_radar.h"


void Rewrite_Radar::radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg) {
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  elevation = msg->elevation*3.14159265359/180;
  azimuth = msg->azimuth*3.14159265359/180;

  // if (method == 0) {
  //   // initialize variables and constants
  //   fmat A(1,3, fill::zeros);
  //   A.print("A initialized:");
  //   A.save("A");
  //   fmat B(1,1, fill::zeros);
  //   B.print("B initialized:");
  //   B.save("B");
  //   fmat W(1,1, fill::zeros);
  //   W.print("W initialized:");
  //   W.save("W");
  //   vvel = 0.038; // radial velocity tolerance [m/s]
  //   method = 3; //Options are: 1 = no slip, 2 = pinv or 3 = LS
  //   LSmethod = 1; //Options are: 1 = normal, 2 = weighted, 3 = 3 best
  //   past_id = -1;
  //   ROS_INFO("vvel = [%f], method = [%i], LSmethod = [%i], past_id = [%i]", vvel, method, LSmethod, past_id);
  // }

  if (msg->range > 0.20)
  {
    ROS_INFO("distance of point [%i] is enough big", msg->point_id);
    e_r(0,0) = msg->x/msg->range;
    e_r(1,0) = msg->y/msg->range;
    e_r(2,0) = msg->z/msg->range;
    e_r.print("current e_r:");

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
          // A.print("A_before:");
          // B.print("B_before:");
          // A.row(j) = trans(e_r);
          // B.row(j) = -msg->velocity;
          ROS_INFO("past_id < point_id");
          // A.load("A");
          // B.load("B");
          A.print("A:");
          A = join_vert(A, trans(e_r));
          v_r(0,0) = -msg->velocity;
          B = join_vert(B, v_r);
          // A.print("A_after:");
          // B.print("B_after:");
          // A.save("A");
          // B.save("B");
        }
        else if (past_id > msg->point_id) {
          ROS_INFO("past_id > point_id");
          // A.load("A");
          // B.load("B");
          // int N = A.n_rows;
          // ROS_INFO("A has [%i] rows.", N);
          // A = A.rows(1,N-1);
          // B = B.rows(1,N-1);
          A.print("A_end:");
          B.print("B_end:");
          v_S = solve(A,B);
          // v_S.print("v_S:");
          odom.twist.twist.linear.x = v_S(0,0);
          odom.twist.twist.linear.y = v_S(1,0);
          odom.twist.twist.linear.z = v_S(2,0);

          odom.twist.covariance[0] = sqrt(vvel);
          odom.twist.covariance[7] = sqrt(vvel);
          odom.twist.covariance[14] = sqrt(vvel);

          pub.publish(odom);

          A = trans(e_r);
          B = v_r;
          // A.save("A");
          // B.save("B");
          past_id = -1;
        }

      }

      else if (LSmethod == 2) {
        if (past_id < msg->point_id) {
          // A.print("A_before:");
          // B.print("B_before:");
          // ROS_INFO("j = [%i]", j);
          // A.row(j) = trans(e_r);
          // B.row(j) = -msg->velocity;
          A = join_vert(A, trans(e_r));
          v_r(0,0) = -msg->velocity;
          B = join_vert(B, v_r);
          actual_weight(0,0) = cos(elevation)*cos(azimuth);
          W = join_vert(W, actual_weight);
          // A.print("A_after:");
          // B.print("B_after:");
        }
        else if (past_id > msg->point_id) {
          A = join_vert(A, trans(e_r));
          v_r(0,0) = -msg->velocity;
          B = join_vert(B, v_r);
          actual_weight(0,0) = cos(elevation)*cos(azimuth);
          W = join_vert(W, actual_weight);
          W = W/as_scalar(sum(W)); // normalize W
          // W.print("W not normalized:");
          // A = A.rows(1,N);
          // B = B.rows(1,N);
          // A.print("A_end:");
          // B.print("B_end:");
          // W.print("normalized W:");
          // diagmat(W.rows(1,N)).print("W_end:");
          // trans(A).print("trans(A)");
          // (trans(A)*diagmat(W.rows(1,N))).print("trans(A)*W:");
          // (trans(A)*diagmat(W.rows(1,N))*A).print("trans(A)*W*A:");
          // (trans(A)*diagmat(W.rows(1,N))*B).print("trans(A)*W*B:");
          int N = A.n_rows;
          fmat A_weighted = trans(A)*diagmat(W.rows(1,N))*A;
          fmat B_weighted = trans(A)*diagmat(W.rows(1,N))*B;
          //
          // A_weighted.print("A_w:");
          // B_weighted.print("B_w

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
          B = v_r;
          W = v_r;
          past_id = 0;
        }
      }

      else if (LSmethod == 3) {
        if (past_id < msg->point_id) {
          // A.print("A_before:");
          // B.print("B_before:");
          // ROS_INFO("j = [%i]", j);
          // A.row(j) = trans(e_r);
          // B.row(j) = -msg->velocity;
          A = join_vert(A, trans(e_r));
          v_r(0,0) = -msg->velocity;
          B = join_vert(B, v_r);
          actual_weight(0,0) = cos(elevation)*cos(azimuth);
          W = join_vert(W, actual_weight);
          // A.print("A_after:");
          // B.print("B_after:");
        }
        else if (past_id > msg->point_id) {
          A = join_vert(A, trans(e_r));
          v_r(0,0) = -msg->velocity;
          B = join_vert(B, v_r);
          actual_weight(0,0) = cos(elevation)*cos(azimuth);
          W = join_vert(W, actual_weight);
          W = W/as_scalar(sum(W)); // normalize W
          // W.print("W not normalized:");
          // A = A.rows(1,N);
          // B = B.rows(1,N);
          // A.print("A_end:");
          // B.print("B_end:");
          // W.print("normalized W:");
          // diagmat(W.rows(1,N)).print("W_end:");
          // trans(A).print("trans(A)");
          // (trans(A)*diagmat(W.rows(1,N))).print("trans(A)*W:");
          // (trans(A)*diagmat(W.rows(1,N))*A).print("trans(A)*W*A:");
          // (trans(A)*diagmat(W.rows(1,N))*B).print("trans(A)*W*B:");
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
          // v_S.print("v_S:");
          odom.twist.twist.linear.x = v_S(0,0);
          odom.twist.twist.linear.y = v_S(1,0);
          odom.twist.twist.linear.z = v_S(2,0);

          odom.twist.covariance[0] = sqrt(vvel);
          odom.twist.covariance[7] = sqrt(vvel);
          odom.twist.covariance[14] = sqrt(vvel);

          pub.publish(odom);

          A = trans(e_r);
          B = v_r;
          W = v_r;
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

// void Rewrite_Radar::initialize_variables() {
//   fmat e_r(3,1, fill::zeros);
//   e_r.print("e_r initialized:");
//   e_r.save("e_r");
//
//   fmat v_r(1,1, fill::zeros);
//   v_r.print("v_r initialized:");
//   fmat v_S(3,1, fill::zeros);
//   v_S.print("v_S initialized:");
//   fmat actual_weight(1,1, fill::zeros);
//   actual_weight.print("actual_weight initialized:");
//   fmat A(1,3, fill::zeros);
//   A.print("A initialized:");
//   fmat B(1,1, fill::zeros);
//   B.print("B initialized:");
//   fmat W(1,1, fill::zeros);
//   W.print("W initialized:");
//   vvel = 0.038; // radial velocity tolerance [m/s]
//   method = 3; //Options are: 1 = no slip, 2 = pinv or 3 = LS
//   LSmethod = 1; //Options are: 1 = normal, 2 = weighted, 3 = 3 best
//   past_id = -1;
//   ROS_INFO("vvel = [%f], method = [%c], LSmethod = [%c], past_id = [%i]", vvel, method, LSmethod, past_id);
// }

void Rewrite_Radar::initialize_subscriber(Rewrite_Radar rewrite_radar) {
  sub = nh.subscribe<ti_mmwave_rospkg::RadarScan>("/ti_mmwave/radar_scan", 100, &Rewrite_Radar::radarCallback, &rewrite_radar);
  ROS_INFO("Subscriber initialized.");
  ros::spin();
 }
