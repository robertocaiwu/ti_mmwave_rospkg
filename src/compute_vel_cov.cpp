#include "compute_vel_cov.h"

fmat Compute_Vel_Cov::approx_error_propagation() {

  fmat diag_cov_az(1,N, fill::zeros);
  fmat diag_cov_el(1,N, fill::zeros);
  fmat diag_cov_vr(1,N, fill::zeros);

  for (int i = 0; i<N; i++) {
    diag_cov_az(0,i) = datum::pi/(12*cos(az_meas(0,i)));
    diag_cov_el(0,i) = datum::pi/(12*cos(el_meas(0,i)));
    diag_cov_vr(0,i) = dv;
  }

  fmat diag_cov_meas = joint_horiz(diag_cov_az, diag_cov_el, diag_cov_vr);
  fmat Cov_meas = diagmat(diag_cov_meas);

  fmat F = compute_jacobian_approx();

  fmat Cov_vS_eval = F*Cov_means*trans(F);

  return Cov_vS_eval;

}

fmat Compute_Vel_Cov::compute_jacobian_approx() {

  fmat F_az(3,N, fill::zeros);
  fmat F_el(3,N, fill::zeros);
  fmat F_vel(3,N, fill::zeros);
  fmat b_plus = trans(vel_meas);
  fmat b_minus = trans(vel_meas);
  fmat A_plus(3,N, fill::zeros);
  fmat A_minus(3,N, fill::zeros);
  fmat A(3,N, fill::zeros);

  for (int i = 0; i<N; i++) {
    A_plus = compute_A(delta_angle, i, "az");
    A_minus = compute_A(-delta_angle, i, "az");
    F_az.rows(i) = (solve(A_plus, vel_meas)-solve(A_minus, vel_meas))/delta_angle;
  }

  for (int i = 0; i<N; i++) {
    A_plus = compute_A(delta_angle, i, "el");
    A_minus = compute_A(-delta_angle, i, "el");
    F_el.rows(i) = (solve(A_plus, vel_meas)-solve(A_minus, vel_meas))/delta_angle;
  }

  for (int i = 0; i<N; i++) {
    A = compute_A(0.0, i, "vel");
    b_plus(i,0) = vel_meas(0,i) + delta_vel/2;
    b_minus(i,0) = vel_meas(0,i) - delta_vel/2;
    F_vel.rows(i) = (solve(A, b_plus)-solve(A, b_minus))/delta_vel;
  }

  fmat F = joint_horiz(F_az, F_el, F_vel);

  return F;

}

fmat Compute_Vel_Cov::compute_A(float d_angl, int I, string variable) {

  fmat A(3,N, fill::zeros);

  for (int i = 0; i<N; i++) {
    if (i==I) {
      if (variable == "az")  // derivation wrt azimuth
      {
        A(i,0) = cos(az_meas(0,i)+d_angl/2)*cos(el_meas(0,i));
        A(i,1) = -sin(az_meas(0,i)+d_angl/2)*cos(el_meas(0,i));
        A(i,2) = cos(el_meas(0,i));
      }
      else if (variable == "el" // derivative wrt elevation)
      {
        A(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i)+d_angl/2);
        A(i,1) = -sin(az_meas(0,i))*cos(el_meas(0,i)+d_angl/2);
        A(i,2) = cos(el_meas(0,i)+d_angl/2);
      }
      else if (variable == "vel") // derivative wrt velocity
      {
        A(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i));
        A(i,1) = -sin(az_meas(0,i))*cos(el_meas(0,i));
        A(i,2) = cos(el_meas(0,i));
      }
      else {
        ROS_ERROR("Compute_A:Derivation variable not properly defined!")
      }
    }
    else {
      A(i,0) = cos(az_meas(0,i))*cos(el_meas(0,i));
      A(i,1) = -sin(az_meas(0,i))*cos(el_meas(0,i));
      A(i,2) = cos(el_meas(0,i));
    }
  }

  return A;

}
