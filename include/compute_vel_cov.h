#ifndef _COMPUTE_VEL_COV_
#define _COMPUTE_VEL_COV_


#include <armadillo>
#include <iostream>

using namespace arma;
using namespace std;

class Compute_Vel_Cov{

public:

  Compute_Vel_Cov(fmat mat_init)
    {
      vel_meas = mat_init;
      az_meas = mat_init;
      el_meas = mat_init;
      dv = 0.039;
      N = 0;
      delta_angle = 1e-4;
      delta_vel = 1e-4;
    };

  fmat approx_error_propagation();
  fmat compute_jacobian_approx();
  fmat compute_A(float d_angl, int I, string variable);

private:
  int N;

  float dv;
  float delta_angle;
  float delta_vel;

  fmat vel_meas;
  fmat az_meas;
  fmat el_meas;

};

#endif
