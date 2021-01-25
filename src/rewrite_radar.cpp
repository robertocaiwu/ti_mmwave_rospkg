#include "rewrite_radar.h"

void Rewrite_Radar::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  angl_vel(0) = msg->angular_velocity.x;
  angl_vel(1) = msg->angular_velocity.y;
  angl_vel(2) = msg->angular_velocity.z;
}


void Rewrite_Radar::radarCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr& msg) {

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.header.stamp = ros::Time::now();

  // save azimuth and elevation for plotting
  plc_info.pose.pose.orientation.x = msg->azimuth;
  plc_info.pose.pose.orientation.y = msg->elevation;

  e_r(0,0) = msg->x/msg->range;
  e_r(1,0) = msg->y/msg->range;
  e_r(2,0) = msg->z/msg->range;

  v_r(0,0) = -msg->velocity;

  elevation = msg->elevation*datum::pi/180;
  azimuth = -msg->azimuth*datum::pi/180;
  actual_weight(0,0) = cos(elevation)*cos(azimuth);

  if (past_id < msg->point_id) { // The new point information is part of the current plc

    if (click == 0) { // The new point is the first point of the first measured point cloud: the collectors need to be initialized
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
    else { // Stacking data onto each other in the data collector matrices and vectors
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
  else if (past_id >= msg->point_id) { // The new point is part of a new pointcloud, estimate the velocity and covariance 
                                       // now that all the information about the old plc are saved.

    if (past_id > 1) { // The estimator works only if there were at least 3 points in the plc

      N = past_id;

      // Estimate the radar velocity vector

      if (ransac) { // Remove outliers
        v_S = RANSAC();
      }
      else {
        if (LSmethod == 1) { // Normal Least Squares
          v_S = solve(A,B);
        }
        else if (LSmethod == 2) { // Weighted Least Squares
          W_norm = W/as_scalar(sum(W));

          A_weighted = trans(A)*diagmat(W_norm)*A;
          B_weighted = trans(A)*diagmat(W_norm)*B;

          v_S = solve(A_weighted,B_weighted);
        }
        else {
        ROS_ERROR("LSmethod [%i] does not exist!", LSmethod);
        }
      }


      // Transform velocity vector into base_link frame
      v_B = v_S + cross(angl_vel, r_r2i);
      odom.twist.twist.linear.x = v_B(0);
      odom.twist.twist.linear.y = v_B(1);
      odom.twist.twist.linear.z = v_B(2);

      // Compute 2D covariance matrix
      if (cov_method == 1) { // Zero covariance method
        cov_vS = {{0.0, 0.0}, {0.0, 0.0}};
      }
      if (cov_method == 2) { // System linearization method
        cov_vS = approx_error_propagation();
      }
      else if (cov_method == 3) { // Monte Carlo sampling
        cov_vS = MC_error_propagation();
      }    
      else {
        ROS_ERROR("cov_method [%i] does not exist!", cov_method);
      }

      // Save 2D covariance matric into odom message
      int k = 0;
      for (int i = 0; i<2; i++) {
        for (int j = 0; j<2; j++) {
          odom.twist.covariance[k] = cov_vS(i,j);
          k++;
        }
        k = k + 4;
      }

      // Publish information about rada estimation
      // Save variances
      odom.pose.pose.position.x = odom.twist.covariance[0];
      odom.pose.pose.position.y = odom.twist.covariance[7];
      pub.publish(odom);
    }

    // Publish information about plc
    // write size of plc in unused place
    plc_info.pose.pose.position.x = past_id+1;
    plc_info.header.stamp = ros::Time::now();
    points_pub.publish(plc_info);

    // Reset the data collector matrices and vectors with the current point data of the new plc
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

  past_id++;

}


void Rewrite_Radar::initialize_subscriber(Rewrite_Radar rewrite_radar) {
  sub = nh.subscribe<ti_mmwave_rospkg::RadarScan>("/ti_mmwave/radar_scan", 100, &Rewrite_Radar::radarCallback, &rewrite_radar);
  sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu_ok", 100, &Rewrite_Radar::imuCallback, &rewrite_radar);
  ROS_INFO("Subscriber initialized.");
  ros::spin();
 }

// Covariance functions

fmat Rewrite_Radar::approx_error_propagation() {

  fmat diag_cov_az(1,N+1, fill::zeros);
  fmat diag_cov_el(1,N+1, fill::zeros);
  fmat diag_cov_vr(1,N+1, fill::zeros);

  // Create measurement covariance matrix
  // calculate the variance of each measurement data and save it into a vector for each data type
  for (int i = 0; i<=N; i++) {
    diag_cov_az(0,i) = ((K/(cos(az_meas(0,i))))/3)^2;
    diag_cov_el(0,i) = ((K/(cos(el_meas(0,i))))/3)^2;
    diag_cov_vr(0,i) = (vvel/3)^2;
  }
  // combine the vector horizontally to one
  fmat diag_cov_meas = join_horiz(diag_cov_az, diag_cov_el);
  diag_cov_meas = join_horiz(diag_cov_meas, diag_cov_vr);
  // create a matrix with the measurment variance vector as diagonal
  fmat Cov_meas = diagmat(diag_cov_meas);

  // compute jacobian matrix
  fmat F = compute_jacobian_approx();

  // calculate velocity covariance
  fmat Cov_vS_eval = F*Cov_meas*trans(F);
  
  // cancel the z entries
  Cov_vS_eval = Cov_vS_eval(span(0,1), span(0,1));


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
  fmat W_plus;
  fmat W_minus;
  fmat W_vel;
  fmat W_plus_norm;
  fmat W_minus_norm;
  fmat M_plus_weighted;
  fmat M_minus_weighted;
  fmat b_plus_weighted;
  fmat b_minus_weighted;
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
      b_minus_weighted = trans(M_minus)*diagmat(W_minus_norm)*trans(vel_meas);

      F_az.col(i) = (solve(M_plus_weighted, b_plus_weighted)-solve(M_minus_weighted, b_minus_weighted))/delta_angle;

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
      b_minus_weighted = trans(M_minus)*diagmat(W_minus_norm)*trans(vel_meas);

      F_el.col(i) = (solve(M_plus_weighted, b_plus_weighted)-solve(M_minus_weighted, b_minus_weighted))/delta_angle;

      // compute jacobian column of velocity
      M = compute_M(0.0, i, "vel");

      W_vel = W; //cannot use W because it is part of main loop
      W_vel_norm = W_vel/as_scalar(sum(W_vel));

      b_plus(i,0) = vel_meas(0,i) + delta_vel/2;
      b_minus(i,0) = vel_meas(0,i) - delta_vel/2;

      M_weighted = trans(M)*W_vel*M;

      b_plus_weighted = trans(M)*W_vel*b_plus;
      b_minus_weighted = trans(M)*W_vel*b_minus;

      F_vel.col(i) = (solve(M_weighted, b_plus_weighted)-solve(M_weighted, b_minus_weighted))/delta_vel;
    }
  }

  fmat F = join_horiz(F_az, F_el);
  F = join_horiz(F, F_vel);

  return F;
}

fmat Rewrite_Radar::compute_M(float d_angl, int I, string variable) {

  fmat M(N+1,3, fill::zeros);

  for (int i = 0; i<=N; i++) {

    if (i==I) {
      if (variable == "az")  // derivation wrt azimuth
      {
        M(i,0) = cos(az_meas(0,i)+d_angl/2)*cos(el_meas(0,i));
        M(i,1) = sin(az_meas(0,i)+d_angl/2)*cos(el_meas(0,i));
        M(i,2) = sin(el_meas(0,i));
      }
      else if (variable == "el") // derivative wrt elevation
      {
        M(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i)+d_angl/2);
        M(i,1) = sin(az_meas(0,i))*cos(el_meas(0,i)+d_angl/2);
        M(i,2) = sin(el_meas(0,i)+d_angl/2);
      }
      else if (variable == "vel") // derivative wrt velocity
      {
        M(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i));
        M(i,1) = sin(az_meas(0,i))*cos(el_meas(0,i));
        M(i,2) = sin(el_meas(0,i));
      }
      else {
        ROS_ERROR("Compute_M:Derivation variable not properly defined!");
      }
    }
    else {
      M(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i));
      M(i,1) = sin(az_meas(0,i))*cos(el_meas(0,i));
      M(i,2) = sin(el_meas(0,i));
    }

  }

  return M;

}
fmat Rewrite_Radar::MC_error_propagation() {


  fmat az_std(N+1,1, fill::zeros);
  fmat el_std(N+1,1, fill::zeros);
  fmat az_res(N+1,1, fill::zeros);
  fmat el_res(N+1,1, fill::zeros);

  fmat new_az_data(N+1,1, fill::zeros);
  fmat new_el_data(N+1,1, fill::zeros);
  fmat new_vel_data(N+1,1, fill::zeros);
  fmat A_new(N+1,3, fill::zeros);
  fmat v_S_collector(3,N_MC, fill::zeros);
  fmat W_new(N+1,1, fill::zeros);
  fmat A_weighted_new;
  fmat B_weighted_new;
  int c;
  float vr_std = vvel/3;

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
        az_std(j,0) = K/3;
        el_std(j,0) = K/3;
        az_res(j,0) = K;
        el_res(j,0) = K;

      }
      else if (cov_method == 3) {
        az_std(j,0) = (K/cos(az_meas(0,j)))/3;
        el_std(j,0) = (K/cos(el_meas(0,j)))/3;
        az_res(j,0) = K/cos(az_meas(0,j));
        el_res(j,0) = K/cos(el_meas(0,j));
      }
      // if (past_id == 1) {
      //   cout << "variance" << "\n";
      // }

      // cout << "std" << "\n";
      new_az_data(j,0) = randn()*az_std(j,0)+az_meas(0,j);
      new_el_data(j,0) = randn()*el_std(j,0)+el_meas(0,j);
      new_vel_data(j,0) = randn()*vr_std+vel_meas(0,j);

      // cout << "new data" << "\n";
      if ( abs(new_az_data(j,0)-az_meas(0,j)) > az_res(j,0) || abs(new_el_data(j,0)-el_meas(0,j)) > el_res(j,0)
      || abs(new_vel_data(j,0)-vel_meas(0,j)) > vvel && c<100) {
        // cout << "outliers found" << "\n";
        c = 0;
        while ( abs(new_az_data(j,0)-az_meas(0,j)) > az_res(j,0) || abs(new_el_data(j,0)-el_meas(0,j)) > el_res(j,0)
        || abs(new_vel_data(j,0)-vel_meas(0,j)) > vvel && c<100) {
        new_az_data(j,0) = randn()*az_std(j,0)+az_meas(0,j);
        new_el_data(j,0) = randn()*el_std(j,0)+el_meas(0,j);
        new_vel_data(j,0) = randn()*vr_std+vel_meas(0,j);
        c++;
        }
      }
      // cout << "no outliers" << "\n";

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

  cov_vS = cov_vS(span(0,1), span(0,1));

  return cov_vS*cov_param;

}

fmat Rewrite_Radar::RANSAC() {
  int outliers = 0;
  fmat error;
  fmat A_opt = A;
  fmat B_opt = B;
  fmat W_opt = W;
  fmat vS_opt;
  fmat e_sorted;
  bool outlier_found = false;
  uvec sorted_index;
  int t;
  fmat A_opt_up;
  fmat A_opt_down;
  fmat B_opt_up;
  fmat B_opt_down;
  float final_MSE;
  fmat az_meas_left;
  fmat az_meas_right;
  fmat el_meas_left;
  fmat el_meas_right;
  fmat vel_meas_left;
  fmat vel_meas_right;
  fmat A_opt_weighted;
  fmat B_opt_weighted;
  fmat W_opt_norm;
  fmat W_opt_up;
  fmat W_opt_down;

  N_opt = N;

  for(int i = 0; i<N; i++) {

    //compute velocity
    if (LSmethod = 1) {
      vS_opt = solve(A_opt, B_opt);
    }
    else if (LSmethod = 2) {
      W_opt_norm = W_opt/as_scalar(sum(W_opt)); // normalize W

      A_opt_weighted = trans(A_opt)*diagmat(W_opt_norm)*A_opt;
      B_opt_weighted = trans(A_opt)*diagmat(W_opt_norm)*B_opt;

      vS_opt = solve(A_opt_weighted, B_opt_weighted);
    }

    //compute error
    error = abs(A_opt*vS_opt-B_opt);

    if (N_opt == 2) {
      final_MSE = as_scalar(sqrt(trans(error)*error));
      odom.pose.pose.orientation.w = final_MSE;
      plc_info.pose.pose.position.y = outliers;
      plc_info.pose.pose.position.z = N_opt+1;
      N = N_opt;
      A = A_opt;
      B = B_opt;
      W = W_opt;
      return vS_opt;
    }
    
    //check for outliers
    for (int j = 0; j<N_opt; j++) {
      if (error(j,0) > vvel) {
        outlier_found = true;
      }
    }

    if (outlier_found) {
      sorted_index = sort_index(error,"descend");
      t = sorted_index(0);
      // remove outlier form the measurement vactors and the system matrices
      if (t==0){
        A_opt = A_opt.rows(1,N_opt);
        B_opt = B_opt.rows(1,N_opt);
        W_opt = W_opt.rows(1,N_opt);

        az_meas = az_meas.cols(1,N_opt);
        el_meas = el_meas.cols(1,N_opt);
        vel_meas= vel_meas.cols(1,N_opt);
      }
      else if (t==N_opt) {
        A_opt = A_opt.rows(0,N_opt-1);
        B_opt = B_opt.rows(0,N_opt-1);
        W_opt = W_opt.rows(0,N_opt-1);

        az_meas = az_meas.cols(0,N_opt-1);
        el_meas = el_meas.cols(0,N_opt-1);
        vel_meas= vel_meas.cols(0,N_opt-1);

      }
      else if (t> 0 && t<N_opt) {
        A_opt_up = A_opt.rows(0,t-1);
        A_opt_down = A_opt.rows(t+1,N_opt);
        A_opt = join_vert(A_opt_up, A_opt_down);

        B_opt_up = B_opt.rows(0,t-1);
        B_opt_down = B_opt.rows(t+1,N_opt);
        B_opt = join_vert(B_opt_up, B_opt_down);

        W_opt_up = W_opt.rows(0,t-1);
        W_opt_down = W_opt.rows(t+1,N_opt);
        W_opt = join_vert(W_opt_up, W_opt_down);

        az_meas_left = az_meas.cols(0,t-1);
        az_meas_right = az_meas.cols(t+1,N_opt);
        az_meas = join_horiz(az_meas_left, az_meas_right);

        el_meas_left = el_meas.cols(0,t-1);
        el_meas_right = el_meas.cols(t+1,N_opt);
        el_meas = join_horiz(el_meas_left, el_meas_right);

        vel_meas_left = vel_meas.cols(0,t-1);
        vel_meas_right = vel_meas.cols(t+1,N_opt);
        vel_meas = join_horiz(vel_meas_left, vel_meas_right);

      }
      else {
        ROS_ERROR("wrong sorted index!");
      }

      N_opt = N_opt-1;
      outliers++;
      outlier_found = false;
    }
    else {
      final_MSE = as_scalar(sqrt(trans(error)*error));
      odom.pose.pose.orientation.w = final_MSE;
      plc_info.pose.pose.position.y = outliers;
      plc_info.pose.pose.position.z = N_opt+1;
      N = N_opt;
      A = A_opt;
      B = B_opt;
      W = W_opt;
      return vS_opt;
    }
  }
  final_MSE = as_scalar(sqrt(trans(error)*error));
  odom.pose.pose.orientation.w = final_MSE;
  plc_info.pose.pose.position.y = outliers;
  plc_info.pose.pose.position.z = N_opt+1;
  N = N_opt;
  A = A_opt;
  B = B_opt;
  W = W_opt;
  return vS_opt;

}