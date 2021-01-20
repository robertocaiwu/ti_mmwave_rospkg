#include "rewrite_radar.h"


void Rewrite_Radar::radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg) {

  // ROS_INFO("distance of point [%i] is enough big", msg->point_id);
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.header.stamp = ros::Time::now();

  // save azimuth and elevation for plotting
  odom.pose.pose.orientation.x = msg->azimuth;
  odom.pose.pose.orientation.y = msg->elevation;



  e_r(0,0) = msg->x/msg->range;
  e_r(1,0) = msg->y/msg->range;
  e_r(2,0) = msg->z/msg->range;

  v_r(0,0) = -msg->velocity;

  // e_r.print("current e_r:");

  if (method == 1) {
    v_S(0,0) = -msg->velocity*(msg->range/msg->x);

    odom.twist.twist.linear.x = v_S(0,0);
    odom.twist.twist.linear.y = v_S(1,0);
    odom.twist.twist.linear.z = v_S(2,0);

    odom.twist.covariance[0] = vvel;
    odom.twist.covariance[7] = vvel;
    odom.twist.covariance[14] = vvel;

    // pub.publish(odom);
  }

  else if (method == 2) {
    v_S = -trans(pinv(e_r)*msg->velocity);

    odom.twist.twist.linear.x = v_S(0,0);
    odom.twist.twist.linear.y = v_S(1,0);
    odom.twist.twist.linear.z = v_S(2,0);

    odom.twist.covariance[0] = vvel;
    odom.twist.covariance[7] = vvel;
    odom.twist.covariance[14] = vvel;

    // pub.publish(odom);
  }

  else if (method == 3) {

    // if (LSmethod == 1 || LSmethod == 2) {
    elevation = msg->elevation*datum::pi/180;
    azimuth = msg->azimuth*datum::pi/180;
    actual_weight(0,0) = cos(elevation)*cos(azimuth);

    if (past_id < msg->point_id) {

      if (click == 0) {
        // cout << "past_id" << past_id << "\n";
        // cout << "point_id" << msg->point_id << "\n";

        A = trans(e_r);
        B = v_r;
        actual_meas(0,0) = azimuth;
        az_meas = actual_meas;
        actual_meas(0,0) = elevation;
        el_meas = actual_meas;
        vel_meas = v_r;
        W = actual_weight;
        click++;
      }
      else {
      // cout << "past_id" << past_id << "\n";
      // cout << "point_id" << msg->point_id << "\n";

        A = join_vert(A, trans(e_r));
        B = join_vert(B, v_r);
        actual_meas(0,0) = azimuth;
        az_meas = join_horiz(az_meas, actual_meas);
        actual_meas(0,0) = elevation;
        el_meas = join_horiz(el_meas, actual_meas);
        vel_meas = join_horiz(vel_meas, v_r);
        W = join_vert(W, actual_weight);
      }

    }
    else if (past_id >= msg->point_id) {
      N = past_id;

      if (past_id == 1) {
      //   // cout << "past_id" << past_id << "\n";
      //   // cout << "point_id" << msg->point_id << "\n";
      //   // cout << "2D" << "\n";
      //   // 2D case with only x and y linear velocities
      //   // A.print("A_now:");
      //   A = A(span(0,1), span(0,1));
      //
      //   B(0,0) = B(0,0)*cos(el_meas(0,0));
      //   B(1,0) = B(1,0)*cos(el_meas(0,1));
      //   // A.print("A_cut:");
      //   // B.print("B:");
      //   bool solved = solve(v_S, A,B);
      //   // v_S.print("v_S:");
      //   if (solved) {
      //
      //   }
      //   else {
      //     ROS_ERROR("Solve problem in main loop!");
      //   }
      //
      //   odom.twist.twist.linear.x = v_S(0,0);
      //   odom.twist.twist.linear.y = v_S(1,0);
      //   // cout << "vel saved" << "\n";
      //   if (cov_method == 1 || cov_method == 2) {
      //     odom.twist.covariance[0] = vvel;
      //     odom.twist.covariance[7] = vvel;
      //     odom.twist.covariance[14] = vvel;
      //   }
      //   else if (cov_method == 3) {
      //       cov_vS = MC_error_propagation();
      //       // cov_vS.print("cov_vS:");
      //       int k = 0;
      //       for (int i = 0; i<3; i++) {
      //         for (int j = 0; j<3; j++) {
      //           odom.twist.covariance[k] = cov_vS(i,j);
      //           k++;
      //         }
      //         k = k + 3;
      //       }
      //   }
      //   else {
      //     ROS_ERROR("cov_method [%i] does not exist!", LSmethod);
      //   }
      //
      // fmat v_S_1 = trans(pinv(e_r)*vel_meas(0,0));
      // fmat v_S_2 = trans(pinv(e_r)*vel_meas(0,1));
      //
      // v_S = (v_S_1+v_S_2)/2;
      //
      // odom.twist.twist.linear.x = v_S(0,0);
      // odom.twist.twist.linear.y = v_S(1,0);
      // odom.twist.twist.linear.z = v_S(2,0);
      //
      // if (cov_method == 1 || cov_method == 2) {
      //   odom.twist.covariance[0] = vvel;
      //   odom.twist.covariance[7] = vvel;
      //   odom.twist.covariance[14] = vvel;
      // }
      // else if (cov_method == 3) {
      //   //MC sampling
      //   float var_az = K/(12*cos(az_meas(0,0)));
      //   float var_el = K/(12*cos(el_meas(0,0)));
      //   // float var_az = K;
      //   // float var_el = K;
      //
      //   float new_az_data;
      //   float new_el_data;
      //   float new_vel_data;
      //   fmat new_e_r(3,1, fill::zeros);
      //   fmat v_S_collector(3,N_MC, fill::zeros);
      //
      //   for (int i = 0; i<N_MC; i++) {
      //     new_az_data = randn()*sqrt(var_az)+az_meas(0,0);
      //     new_el_data = randn()*sqrt(var_el)+el_meas(0,0);
      //     new_vel_data = randn()*sqrt(vvel)+vel_meas(0,0);
      //
      //     while ( abs(new_az_data-az_meas(0,0)) > var_az || abs(new_el_data-el_meas(0,0)) > var_el
      //     || abs(new_vel_data-vel_meas(0,0)) > vvel) {
      //       new_az_data = randn()*sqrt(var_az)+az_meas(0,0);
      //       new_el_data = randn()*sqrt(var_el)+el_meas(0,0);
      //       new_vel_data = randn()*sqrt(vvel)+vel_meas(0,0);
      //     }
      //
      //     new_e_r(0,0) = cos(new_az_data)*cos(new_el_data);
      //     new_e_r(1,0) = -sin(new_az_data)*cos(new_el_data);
      //     new_e_r(2,0) = cos(new_el_data);
      //
      //     v_S_collector.col(i) = trans(pinv(new_e_r)*new_vel_data);
      //   }
      //
      //   // fmat cov_vS = cov(trans(v_S_collector));
      //   fmat mean_v_S = mean(v_S_collector, 1);
      //   fmat cov_vS = diagmat(mean_v_S);
      //   int k = 0;
      //   for (int i = 0; i<3; i++) {
      //     for (int j = 0; j<3; j++) {
      //       odom.twist.covariance[k] = cov_vS(i,j);
      //       k++;
      //     }
      //     k = k + 3;
      //   }
      // }
      }
      else if (past_id > 1) {
        // cout << "past_id" << past_id << "\n";
        // cout << "point_id" << msg->point_id << "\n";
        // cout << "main loop" << "\n";

        if (LSmethod == 1) {
          // v_S = solve(A,B);
          bool solved = solve(v_S, A,B);
          if (solved) {

          }
          else {
            ROS_ERROR("Solve problem in main loop!");
          }
        }
        else if (LSmethod == 2) {
          W_norm = W/as_scalar(sum(W)); // normalize W

          A_weighted = trans(A)*diagmat(W_norm)*A;
          B_weighted = trans(A)*diagmat(W_norm)*B;

          // A_weighted.print("A_weighted:");

          // v_S = solve(A_weighted,B_weighted);
          bool solved = solve(v_S, A_weighted,B_weighted);
          if (solved) {

          }
          else {
            ROS_ERROR("Solve problem in main loop!");
          }

        }
        else {
          ROS_ERROR("LSmethod [%i] does not exist!", LSmethod);
        }

        odom.twist.twist.linear.x = v_S(0,0);
        odom.twist.twist.linear.y = v_S(1,0);
        odom.twist.twist.linear.z = v_S(2,0);


        // if (cov_method == 1) {
        //   odom.twist.covariance[0] = vvel;
        //   odom.twist.covariance[7] = vvel;
        //   odom.twist.covariance[14] = vvel;
        // }
        // if (cov_method == 2 || cov_method == 3) {
          if (cov_method == 2) {
            cov_vS = approx_error_propagation();
          }
          else if (cov_method == 1 || cov_method == 3) {
            cov_vS = MC_error_propagation();
          }
	  else if(cov_method == 4) {
	    fmat e = A*v_S - B;
	    e.print("estimation error: ");
	    int dim = B.n_rows;
	    cout << "system dimension: " << dim << "\n";
	    if (dim == 3) {
            	cov_vS = MC_error_propagation();
	    }
	    else {
	    	cov_vS = as_scalar(trans(e)*e)*inv(trans(A)*A)/(dim-3);
	    }

	    cov_vS.print("cov_vS:");
	  }
          else {
            ROS_ERROR("cov_method [%i] does not exist!", LSmethod);
          }

          int k = 0;
          for (int i = 0; i<3; i++) {
            for (int j = 0; j<3; j++) {
              odom.twist.covariance[k] = cov_vS(i,j);
              k++;
            }
            k = k + 3;
          }
        // // write size of plc in unused place
        // odom.pose.pose.position.x = past_id+1;
        //
        // pub.publish(odom);
      }
      else if (past_id == msg->point_id) {
        // // cout << "past_id" << past_id << "\n";
        // // cout << "point_id" << msg->point_id << "\n";
        // // cout << "1D" << "\n";

        // v_S = -trans(pinv(e_r)*msg->velocity);

        // odom.twist.twist.linear.x = v_S(0,0);
        // odom.twist.twist.linear.y = -v_S(1,0);
        // odom.twist.twist.linear.z = v_S(2,0);

        // if (cov_method == 1 || cov_method == 2) {
        //   odom.twist.covariance[0] = vvel;
        //   odom.twist.covariance[7] = vvel;
        //   odom.twist.covariance[14] = vvel;
        // }
        // else if (cov_method == 3) {
        //   //MC sampling
        //   float var_az = K/(12*cos(az_meas(0,0)));
        //   float var_el = K/(12*cos(el_meas(0,0)));
        //   // float var_az = K;
        //   // float var_el = K;

        //   float new_az_data;
        //   float new_el_data;
        //   float new_vel_data;
        //   fmat new_e_r(3,1, fill::zeros);
        //   fmat v_S_collector(3,N_MC, fill::zeros);

        //   for (int i = 0; i<N_MC; i++) {
        //     new_az_data = randn()*sqrt(var_az)+az_meas(0,0);
        //     new_el_data = randn()*sqrt(var_el)+el_meas(0,0);
        //     new_vel_data = randn()*sqrt(vvel)+vel_meas(0,0);

        //     while ( abs(new_az_data-az_meas(0,0)) > var_az || abs(new_el_data-el_meas(0,0)) > var_el
        //     || abs(new_vel_data-vel_meas(0,0)) > vvel) {
        //       new_az_data = randn()*sqrt(var_az)+az_meas(0,0);
        //       new_el_data = randn()*sqrt(var_el)+el_meas(0,0);
        //       new_vel_data = randn()*sqrt(vvel)+vel_meas(0,0);
        //     }

        //     new_e_r(0,0) = cos(new_az_data)*cos(new_el_data);
        //     new_e_r(1,0) = -sin(new_az_data)*cos(new_el_data);
        //     new_e_r(2,0) = cos(new_el_data);

        //     v_S_collector.col(i) = trans(pinv(new_e_r)*new_vel_data);
        //   }

        //   // fmat cov_vS = cov(trans(v_S_collector));
        //   fmat mean_v_S = mean(v_S_collector, 1);
        //   fmat cov_vS = diagmat(mean_v_S);
        //   int k = 0;
        //   for (int i = 0; i<3; i++) {
        //     for (int j = 0; j<3; j++) {
        //       odom.twist.covariance[k] = cov_vS(i,j);
        //       k++;
        //     }
        //     k = k + 3;
        //   }
        // }
      }

      // write size of plc in unused place
      odom.pose.pose.position.x = past_id+1;

      pub.publish(odom);
      A = trans(e_r);
      B = v_r;
      past_id = -1;

      actual_meas(0,0) = azimuth;
      az_meas = actual_meas;
      actual_meas(0,0) = elevation;
      el_meas = actual_meas;
      vel_meas = v_r;
      W = actual_weight;
    }

  }

  else {
    ROS_ERROR("Method [%i] does not exist!", method);
  }

  past_id++;
  // cout << "past_id++ " << "\n";

}


void Rewrite_Radar::initialize_subscriber(Rewrite_Radar rewrite_radar) {
  sub = nh.subscribe<ti_mmwave_rospkg::RadarScan>("/ti_mmwave/radar_scan", 100, &Rewrite_Radar::radarCallback, &rewrite_radar);
  ROS_INFO("Subscriber initialized.");
  ros::spin();
 }

// Covariance functions

fmat Rewrite_Radar::approx_error_propagation() {

  fmat diag_cov_az(1,N+1, fill::zeros);
  fmat diag_cov_el(1,N+1, fill::zeros);
  fmat diag_cov_vr(1,N+1, fill::zeros);

  for (int i = 0; i<=N; i++) {
    diag_cov_az(0,i) = datum::pi/(12*cos(az_meas(0,i)));
    diag_cov_el(0,i) = datum::pi/(12*cos(el_meas(0,i)));
    diag_cov_vr(0,i) = vvel;
  }

  fmat diag_cov_meas = join_horiz(diag_cov_az, diag_cov_el);
  diag_cov_meas = join_horiz(diag_cov_meas, diag_cov_vr);

  fmat Cov_meas = diagmat(diag_cov_meas);
  // Cov_meas.print("Cov_meas:");

  fmat F = compute_jacobian_approx();

  fmat Cov_vS_eval = F*Cov_meas*trans(F);

  return Cov_vS_eval*cov_param;

}

fmat Rewrite_Radar::compute_jacobian_approx() {

  fmat F_az(3,N+1, fill::zeros);
  fmat F_el(3,N+1, fill::zeros);
  fmat F_vel(3,N+1, fill::zeros);
  fmat b_plus = trans(vel_meas);
  fmat b_minus = trans(vel_meas);
  fmat M_plus(3,N+1, fill::zeros);
  fmat M_minus(3,N+1, fill::zeros);
  fmat M(3,N+1, fill::zeros);
  fmat W_plus;
  fmat W_minus;
  fmat W_vel;
  fmat W_plus_norm;
  fmat W_minus_norm;
  fmat M_plus_weighted;
  fmat M_minus_weighted;
  fmat b_plus_weighted;
  fmat b_minus_weight;
  fmat W_vel_norm;
  fmat M_weighted;

  if (LSmethod = 1) {
    for (int i = 0; i<=N; i++) {
      // compute jacobian column of azimuth
      M_plus = compute_M(delta_angle, i, "az");
      M_minus = compute_M(-delta_angle, i, "az");

      F_az.col(i) = (solve(M_plus, trans(vel_meas))-solve(M_minus, trans(vel_meas)))/delta_angle;

      // compute jacobian column of elevation
      M_plus = compute_M(delta_angle, i, "el");
      M_minus = compute_M(-delta_angle, i, "el");

      F_el.col(i) = (solve(M_plus, trans(vel_meas))-solve(M_minus, trans(vel_meas)))/delta_angle;

      // compute jacobian column of velocity
      M = compute_M(0.0, i, "vel");

      b_plus(i,0) = vel_meas(0,i) + delta_vel/2;
      b_minus(i,0) = vel_meas(0,i) - delta_vel/2;

      F_vel.col(i) = (solve(M, b_plus)-solve(M, b_minus))/delta_vel;
    }
  }
  else if(LSmethod = 2) {
    for (int i = 0; i<=N; i++) {
      // compute jacobian column of azimuth
      M_plus = compute_M(delta_angle, i, "az");
      M_minus = compute_M(-delta_angle, i, "az");

      W_plus = W;
      W_plus(i,0) = cos(az_meas(0,i)+delta_angle/2)*cos(el_meas(0,i));
      W_plus_norm = W_plus/as_scalar(sum(W_plus));

      W_minus = W;
      W_minus(i,0) = cos(az_meas(0,i)-delta_angle/2)*cos(el_meas(0,i));
      W_minus_norm = W_minus/as_scalar(sum(W_minus));

      M_plus_weighted = trans(M_plus)*diagmat(W_plus_norm)*M_plus;
      M_minus_weighted = trans(M_minus)*diagmat(W_minus_norm)*M_minus;

      b_plus_weighted = trans(M_plus)*diagmat(W_plus_norm)*trans(vel_meas);
      b_minus_weight = trans(M_minus)*diagmat(W_minus_norm)*trans(vel_meas);

      F_az.col(i) = (solve(M_plus_weighted, b_plus_weighted)-solve(M_minus_weighted, b_minus_weight))/delta_angle;

      // compute jacobian column of elevation
      M_plus = compute_M(delta_angle, i, "el");
      M_minus = compute_M(-delta_angle, i, "el");

      W_plus = W;
      W_plus(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i)+delta_angle/2);
      W_plus_norm = W_plus/as_scalar(sum(W_plus));

      W_minus = W;
      W_minus(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i)-delta_angle/2);
      W_minus_norm = W_minus/as_scalar(sum(W_minus));

      M_plus_weighted = trans(M_plus)*diagmat(W_plus_norm)*M_plus;
      M_minus_weighted = trans(M_minus)*diagmat(W_minus_norm)*M_minus;

      b_plus_weighted = trans(M_plus)*diagmat(W_plus_norm)*trans(vel_meas);
      b_minus_weight = trans(M_minus)*diagmat(W_minus_norm)*trans(vel_meas);

      F_el.col(i) = (solve(M_plus_weighted, b_plus_weighted)-solve(M_minus_weighted, b_minus_weight))/delta_angle;

      // compute jacobian column of velocity
      M = compute_M(0.0, i, "vel");

      W_vel = W; //cannot use W because it is part of main loop
      W_vel_norm = W_vel/as_scalar(sum(W_vel));

      b_plus(i,0) = vel_meas(0,i) + delta_vel/2;
      b_minus(i,0) = vel_meas(0,i) - delta_vel/2;

      M_weighted = trans(M)*W_vel*M;

      b_plus_weighted = trans(M)*W_vel*b_plus;
      b_minus_weight = trans(M)*W_vel*b_minus;

      F_vel.col(i) = (solve(M_weighted, b_plus_weighted)-solve(M_weighted, b_minus_weight))/delta_vel;
    }
  }

  fmat F = join_horiz(F_az, F_el);
  F = join_horiz(F, F_vel);

  return F;
}

fmat Rewrite_Radar::compute_M(float d_angl, int I, string variable) {

  fmat M(N+1,3, fill::zeros);

  for (int i = 0; i<=N; i++) {
    // cout << "I = " << I << ",  i = " << i << ",  variable = " << variable << "\n";
    if (i==I) {
      if (variable == "az")  // derivation wrt azimuth
      {
        M(i,0) = cos(az_meas(0,i)+d_angl/2)*cos(el_meas(0,i));
        M(i,1) = -sin(az_meas(0,i)+d_angl/2)*cos(el_meas(0,i));
        M(i,2) = cos(el_meas(0,i));
      }
      else if (variable == "el") // derivative wrt elevation
      {
        M(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i)+d_angl/2);
        M(i,1) = -sin(az_meas(0,i))*cos(el_meas(0,i)+d_angl/2);
        M(i,2) = cos(el_meas(0,i)+d_angl/2);
      }
      else if (variable == "vel") // derivative wrt velocity
      {
        M(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i));
        M(i,1) = -sin(az_meas(0,i))*cos(el_meas(0,i));
        M(i,2) = cos(el_meas(0,i));
      }
      else {
        ROS_ERROR("Compute_M:Derivation variable not properly defined!");
      }
    }
    else {
      M(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i));
      M(i,1) = -sin(az_meas(0,i))*cos(el_meas(0,i));
      M(i,2) = cos(el_meas(0,i));
    }
    // M.print("M:");
  }

  return M;

}

fmat Rewrite_Radar::MC_error_propagation() {

  fmat az_var(N+1,1, fill::zeros);
  fmat el_var(N+1,1, fill::zeros);
  fmat new_az_data(N+1,1, fill::zeros);
  fmat new_el_data(N+1,1, fill::zeros);
  fmat new_vel_data(N+1,1, fill::zeros);
  fmat A_new(N+1,3, fill::zeros);
  fmat v_S_collector(3,N_MC, fill::zeros);
  fmat W_new(N+1,1, fill::zeros);
  fmat A_weighted_new;
  fmat B_weighted_new;

  // Sample N_MC times
  for (int i = 0; i<N_MC; i++) {

    // create new data for sampling
    for (int j = 0; j<=N; j++) {

      // Compute variance of data
      // if (past_id==1) {
      //   az_meas.print("az_meas:");
      //   el_meas.print("el_meas:");
      //   cout << "N = " << N << "\n";
      //   cout << "j = " << j << "\n";
      // }
      if (cov_method == 1) {
        az_var(j,0) = K;
        el_var(j,0) = K;
      }
      else if (cov_method == 3) {
        az_var(j,0) = K/cos(az_meas(0,j));
        el_var(j,0) = K/cos(el_meas(0,j));
      }
      // if (past_id == 1) {
      //   cout << "variance" << "\n";
      // }


      new_az_data(j,0) = randn()*sqrt(az_var(j,0))+az_meas(0,j);
      new_el_data(j,0) = randn()*sqrt(el_var(j,0))+el_meas(0,j);
      new_vel_data(j,0) = randn()*sqrt(vvel)+vel_meas(0,j);

      int c = 0;
      while ( abs(new_az_data(j,0)-az_meas(0,j)) > az_var(j,0) || abs(new_el_data(j,0)-el_meas(0,j)) > el_var(j,0)
      || abs(new_vel_data(j,0)-vel_meas(0,j)) > vvel && c<100) {
        new_az_data(j,0) = randn()*sqrt(az_var(j,0))+az_meas(0,j);
        new_el_data(j,0) = randn()*sqrt(el_var(j,0))+el_meas(0,j);
        new_vel_data(j,0) = randn()*sqrt(vvel)+vel_meas(0,j);
        c++;
      }


      W_new(j,0) = cos(new_az_data(j,0))*cos(new_el_data(j,0));

      if (past_id == 1) {
        A_new(j,0) = cos(new_az_data(j,0))*cos(new_el_data(j,0));
        A_new(j,1) = -sin(new_az_data(j,0))*cos(new_el_data(j,0));
      }
      else {
        A_new(j,0) = cos(new_az_data(j,0))*cos(new_el_data(j,0));
        A_new(j,1) = -sin(new_az_data(j,0))*cos(new_el_data(j,0));
        A_new(j,2) = cos(new_el_data(j,0));
      }

    }

    if (past_id == 1) {
      // A_new.print("A_new before:");
      A_new = A_new(span(0,1), span(0,1));
      // A_new.print("A_new cut:");
    }

    // new_az_data.print("new_az_data:");
    // new_el_data.print("new_el_data:");

    if (LSmethod == 1) {
      // v_S_collector.col(i) = solve(A_new, new_vel_data);
      fmat sol;
      bool solved = solve(sol, A_new, new_vel_data);

      if (past_id == 1) {
        fmat zero(1,1, fill::zeros);
        sol = join_vert(sol, zero);
        // sol.print("sol:");
      }


      v_S_collector.col(i) = sol;
      // if (past_id==1) {
      //   v_S_collector.print("v_S_collector:");
      // }
      if (solved) {

      }
      else {
        ROS_ERROR("Solve problem in covariance!");
      }
    }
    else if (LSmethod == 2) {
      W_new = W_new/as_scalar(sum(W_new)); // normalize W
      // diagmat(W_new).print("W_new:");
      // A_new.print("A_new");
      A_weighted_new = trans(A_new)*diagmat(W_new)*A_new;
      B_weighted_new = trans(A_new)*diagmat(W_new)*new_vel_data;

      fmat sol;
      bool solved = solve(sol, A_weighted_new, B_weighted_new);

      if (past_id == 1) {
        fmat zero(1,1, fill::zeros);
        sol = join_vert(sol, zero);
      }

      // sol.print("sol");

      v_S_collector.col(i) = sol;

      if (solved) {

      }
      else {
        ROS_ERROR("Solve problem in covariance!");
      }
    }

  }
  



  // v_S_collector.print("v_S_collector:");

  // fmat var_vS = var(v_S_collector, 0, 1);
  // var_vS.print("var_v_S:");
  fmat cov_vS = cov(trans(v_S_collector));
  // cov_vS.print("cov_vS:");
  // if (past_id == 1) {
  //   v_S_collector.print("v_S_collector:");
  // }
  // fmat mean_v_S = mean(v_S_collector, 1);
  // mean_v_S.print("mean_v_S:");

  return cov_vS*cov_param;

}

fmat Rewrite_Radar::RANSAC() {
  srand (time(NULL));
  int i = 0;
  fmat measu_sum(1,3, fill::zeros);
  fmat extr_meas(3,3, fill::zeros);
  
  while (i<k)
  {
    // extract 3 random measurements
    meas_num = round(rand(1,3)*N); // array with rendomly chosen measurements number
     

  }
  

}