#include "rewrite_radar.h"


void Rewrite_Radar::radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg) {

  // ROS_INFO("distance of point [%i] is enough big", msg->point_id);
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.header.stamp = ros::Time::now();

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

    // if (LSmethod == 1 || LSmethod == 2) {
    elevation = msg->elevation*datum::pi/180;
    azimuth = msg->azimuth*datum::pi/180;
    actual_weight(0,0) = cos(elevation)*cos(azimuth);

    if (past_id < msg->point_id) {

      if (click == 0) {
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
    else if (past_id > msg->point_id) {

      if (past_id <= 1) {

      }
      else if (past_id > 1) {
        N = past_id;

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
          W = W/as_scalar(sum(W)); // normalize W

          A_weighted = trans(A)*diagmat(W)*A;
          B_weighted = trans(A)*diagmat(W)*B;

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


        // rotate to vicon frame
        // v_S = inv(R)*v_S;
        odom.twist.twist.linear.x = v_S(0,0);
        odom.twist.twist.linear.y = v_S(1,0);
        odom.twist.twist.linear.z = v_S(2,0);

        //compute covariance approximation
        // ROS_INFO("N = [%i]", N);
        // az_meas.print("az_meas:");
        // el_meas.print("el_meas:");
        // vel_meas.print("vel_meas:");

        // fmat Cov_vS_approx = approx_error_propagation();
        // // Cov_vS_approx.print("Cov_vS_approx:");


        // odom.twist.covariance[0] = 0.0;
        // odom.twist.covariance[7] = 0.0;
        // odom.twist.covariance[14] = 0.0;

        fmat cov_vS = MC_error_propagation();
        // cov_vS.print("cov_vS:");
        // odom.twist.covariance[0] = var_v_S(0,0);
        // odom.twist.covariance[7] = var_v_S(1,0);
        // odom.twist.covariance[14] = var_v_S(2,0);

        int k = 0;
        for (int i = 0; i<3; i++) {
          for (int j = 0; j<3; j++) {
            odom.twist.covariance[k] = cov_vS(i,j);
            k++;
          }
          k = k + 3;
        }

        // write size of plc in unused place
        odom.pose.pose.position.x = N+1;

        pub.publish(odom);

      }
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
    else if (past_id == msg->point_id) {
      // ROS_INFO("past_id = point_id = [%i]", past_id);
      // past_id = past_id-1;

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
    diag_cov_vr(0,i) = dv;
  }

  fmat diag_cov_meas = join_horiz(diag_cov_az, diag_cov_el);
  diag_cov_meas = join_horiz(diag_cov_meas, diag_cov_vr);

  fmat Cov_meas = diagmat(diag_cov_meas);
  // Cov_meas.print("Cov_meas:");

  fmat F = compute_jacobian_approx();

  fmat Cov_vS_eval = F*Cov_meas*trans(F);

  return Cov_vS_eval;

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

  for (int i = 0; i<=N; i++) {
    M_plus = compute_M(delta_angle, i, "az");
    M_minus = compute_M(-delta_angle, i, "az");

    // M_plus.print("M_plus:");
    // M_minus.print("M_minus:");
    // trans(vel_meas).print("b:");
    fmat F_az_i = (solve(M_plus, trans(vel_meas))-solve(M_minus, trans(vel_meas)))/delta_angle;
    // F_az_i.print("F_az_i:");
    F_az.col(i) = F_az_i;
  }
  // F_az.print("F_az:");

  for (int i = 0; i<=N; i++) {
    M_plus = compute_M(delta_angle, i, "el");
    M_minus = compute_M(-delta_angle, i, "el");
    F_el.col(i) = (solve(M_plus, trans(vel_meas))-solve(M_minus, trans(vel_meas)))/delta_angle;
  }
  // F_el.print("F_el:");

  for (int i = 0; i<=N; i++) {
    M = compute_M(0.0, i, "vel");
    b_plus(i,0) = vel_meas(0,i) + delta_vel/2;
    b_minus(i,0) = vel_meas(0,i) - delta_vel/2;
    F_vel.col(i) = (solve(M, b_plus)-solve(M, b_minus))/delta_vel;
  }
  // F_vel.print("F_vel:");

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
  // cout << "N = " << N << "\n";
  // az_meas.print("az_meas:");
  // el_meas.print("el_meas:");
  // vel_meas.print("vel_meas:");

  // Compute variance of data
  for (int i = 0; i<=N; i++) {
    az_var(i,0) = K/cos(az_meas(0,i));
    el_var(i,0) = K/cos(el_meas(0,i));
  }

  // az_var.print("az_var:");
  // el_var.print("el_var");

  // Sample N_MC times

  for (int i = 0; i<N_MC; i++) {

    // create new data for sampling
    for (int j = 0; j<=N; j++) {
      new_az_data(j,0) = randn()*sqrt(az_var(j,0))+az_meas(0,j);
      new_el_data(j,0) = randn()*sqrt(el_var(j,0))+el_meas(0,j);
      new_vel_data(j,0) = randn()*sqrt(vvel)+vel_meas(0,j);

      W_new(j,0) = cos(new_az_data(j,0))*cos(new_el_data(j,0));

      A_new(j,0) = cos(new_az_data(j,0))*cos(new_el_data(j,0));
      A_new(j,1) = -sin(new_az_data(j,0))*cos(new_el_data(j,0));
      A_new(j,2) = cos(new_el_data(j,0));
    }

    // new_az_data.print("new_az_data:");
    // new_el_data.print("new_el_data:");

    if (LSmethod == 1) {
      // v_S_collector.col(i) = solve(A_new, new_vel_data);
      fmat sol;
      bool solved = solve(sol, A_new, new_vel_data);

      v_S_collector.col(i) = sol;

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
  // fmat mean_v_S = mean(v_S_collector, 1);

  return cov_vS;

}
