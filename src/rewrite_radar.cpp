#include "rewrite_radar.h"


void Rewrite_Radar::radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg) {
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  elevation = msg->elevation*3.14159265359/180;
  azimuth = msg->azimuth*3.14159265359/180;

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

          odom.twist.covariance[0] = sqrt(vvel);
          odom.twist.covariance[7] = sqrt(vvel);
          odom.twist.covariance[14] = sqrt(vvel);

          pub.publish(odom);

          break;
        case 'B':
          v_S = trans(pinv(e_r)*msg->velocity);

          odom.twist.twist.linear.x = v_S(0,0);
          odom.twist.twist.linear.y = v_S(1,0);
          odom.twist.twist.linear.z = v_S(2,0);

          odom.twist.covariance[0] = sqrt(vvel);
          odom.twist.covariance[7] = sqrt(vvel);
          odom.twist.covariance[14] = sqrt(vvel);

          pub.publish(odom);

          break;
        case 'C':
          switch(LSmethod) {
            case 'A':
              if (past_id < msg->point_id) {
                // A.print("A_before:");
                // B.print("B_before:");
                // ROS_INFO("j = [%i]", j);
                // A.row(j) = trans(e_r);
                // B.row(j) = -msg->velocity;
                A = join_vert(A, trans(e_r));
                v_r(0,0) = -msg->velocity;
                B = join_vert(B, v_r);
                // A.print("A_after:");
                // B.print("B_after:");
              }
              else if (past_id > msg->point_id) {
                A = join_vert(A, trans(e_r));
                v_r(0,0) = -msg->velocity;
                B = join_vert(B, v_r);
                int N = A.n_rows;
                A = A.rows(1,N);
                B = B.rows(1,N);
                // A.print("A_end:");
                // B.print("B_end:");
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

              }

            break;

            case 'B':
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

              }
            break;

            case 'C':
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

              }
            break;
          }
        }
      }
      past_id++;
}

void Rewrite_Radar::initialize_variables() {
  fmat e_r(3,1, fill::zeros);
  fmat v_r(1,1, fill::zeros);
  fmat v_S(3,1, fill::zeros);
  fmat actual_weight(1,1, fill::zeros);
  vvel = 0.038; // radial velocity tolerance [m/s]
  fmat A(1,3, fill::zeros);
  fmat B(1,1, fill::zeros);
  fmat W(1,1, fill::zeros);
  method = 'C'; //Options are: A = no slip, B = pinv or C = LS
  LSmethod = 'A'; //Options are: A = normal, B = weighted, C = 3 best
  past_id = -1;
}

void Rewrite_Radar::initialize_sub_pub() {
  Rewrite_Radar rewrite_radar;
  pub = nh.advertise<nav_msgs::Odometry>("radar_odom", 100);
  sub = nh.subscribe<ti_mmwave_rospkg::RadarScan>("/ti_mmwave/radar_scan", 100, &Rewrite_Radar::radarCallback, &rewrite_radar);
 }
